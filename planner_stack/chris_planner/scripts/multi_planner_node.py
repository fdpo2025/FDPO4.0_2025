#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int32, Bool, UInt32, Int32MultiArray
import heapq

import os
import sys

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(CURRENT_DIR)

if PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_DIR)


# Add the modules directory to the path
import rospkg
rospack = rospkg.RosPack()
chris_planner_path = rospack.get_path('chris_planner')

# Add modules to Python path
# In development: modules are in source tree
# After installation: modules are in lib/python3/dist-packages/chris_planner/
modules_path = os.path.join(chris_planner_path, 'modules')
if os.path.exists(modules_path):
    # Development mode: add parent directory so we can import modules
    sys.path.insert(0, os.path.dirname(modules_path))
else:
    # Installed mode: modules should be in the Python path already
    # But we can also try to add the package path
    pass

# Import chris_planner modules
try:
    import modules.planner as planner_module
    import modules.factory as factory_module
    import modules.yaml_utils as yaml_utils
except ImportError:
    # Fallback: try importing from chris_planner package
    import chris_planner.modules.planner as planner_module
    import chris_planner.modules.factory as factory_module
    import chris_planner.modules.yaml_utils as yaml_utils


class MultiPlannerNode:

    def __init__(self):

        rospy.init_node("multi_planner_node")

        self.package_path = rospack.get_path('chris_planner')

        graph_file = os.path.join(self.package_path, 'files', 'inputs', 'graph.yaml')
        factory_components_file = os.path.join(self.package_path, 'files', 'inputs', 'factory_components.yaml')

        graph_dict = yaml_utils.load_file(graph_file)
        factory_components_dict = yaml_utils.load_file(factory_components_file)

        planning_method = rospy.get_param('~planning_method', 'astar')

        self.planner = planner_module.Planner(graph_dict, factory_components_dict, method=planning_method)
        self.factory = self.planner.factory

        self.boxes = None

        self.robots = {
            "r1": {
                "node": 31,
                "box": -1,
                "goal": None,
                "path": [],
                "busy": False
            },
            "r2": {
                "node": 31,
                "box": -1,
                "goal": None,
                "path": [],
                "busy": False
            }
        }

        self.reserved_nodes = {}
        self.reserved_goals = {}

        # publishers
        self.pub_r1 = rospy.Publisher(
            "/robot1_planned_paths", Int32MultiArray, queue_size=10)

        self.pub_r2 = rospy.Publisher(
            "/robot2_planned_paths", Int32MultiArray, queue_size=10)

        # subscribers
        rospy.Subscriber("/color_sequence", String, self.sequence_cb)

        rospy.Subscriber("/robot1/current_pose", UInt32, self.robot1_node_cb)
        rospy.Subscriber("/robot2/current_pose", UInt32, self.robot2_node_cb)
       
    # -----------------------------------------------------

    def sequence_cb(self, msg):

        seq = msg.data.strip()

        rospy.loginfo(f"Received sequence {seq}")

        boxtypes = self.sequence_to_boxtypes(seq)

        initial_state = self.factory.initial_state(boxtypes)
        _, _, self.boxes = initial_state

        self.plan_for_robot("r1")
        self.plan_for_robot("r2")

    # -----------------------------------------------------

    def build_state(self, robot_id):

        r = self.robots[robot_id]

        return (r["node"], r["box"], self.boxes)

    # -----------------------------------------------------

    def plan_for_robot(self, robot_id):

        state = self.build_state(robot_id)
        robot_node = state[0]

        rospy.loginfo(f"[{robot_id}] state = {state}")

        valid_nodes = self.factory.valid_destinations(state)
        rospy.loginfo(f"[{robot_id}] valid_nodes before filter = {valid_nodes}")

        valid_nodes = [
            n for n in valid_nodes
            if n not in self.reserved_goals
        ]
        rospy.loginfo(f"[{robot_id}] valid_nodes after filter = {valid_nodes}")

        if not valid_nodes:
            rospy.logwarn(f"No valid nodes for {robot_id}")
            return

        best_path = None
        best_goal = None
        best_cost = float("inf")

        reserved_by_other = {
            n for n, r in self.reserved_nodes.items()
            if r != robot_id
        }

        for node in valid_nodes:

            path = self.shortest_path_avoiding(robot_node, node, reserved_by_other)

            rospy.loginfo(f"[{robot_id}] candidate goal={node}, avoided path={path}")

            if not path:
                continue

            cost = self.path_cost(path)

            if cost < best_cost:
                best_cost = cost
                best_path = path
                best_goal = node

        rospy.loginfo(f"[{robot_id}] best_path={best_path}, best_goal={best_goal}")

        if best_path is None:
            rospy.logwarn(f"No collision free path for {robot_id}")
            return

        self.reserve_path(robot_id, best_path, best_goal)
        self.publish_path(robot_id, best_path)

        self.robots[robot_id]["path"] = best_path
        self.robots[robot_id]["goal"] = best_goal
        self.robots[robot_id]["busy"] = True

        rospy.loginfo(f"{robot_id} planned path {best_path}")

    # -----------------------------------------------------

    def reserve_path(self, robot_id, path, goal):

        for n in path:

            # ignorar o nó inicial partilhado
            if n == 31:
                continue

            self.reserved_nodes[n] = robot_id

        self.reserved_goals[goal] = robot_id

    # -----------------------------------------------------

    def publish_path(self, robot_id, path):

        msg = Int32MultiArray()
        msg.data = path

        if robot_id == "r1":
            self.pub_r1.publish(msg)
        else:
            self.pub_r2.publish(msg)

    # -----------------------------------------------------

    def release_node(self, robot_id, node):

        if node in self.reserved_nodes:
            if self.reserved_nodes[node] == robot_id:
                del self.reserved_nodes[node]

    # -----------------------------------------------------

    def robot1_node_cb(self, msg):

        node = msg.data
        self.update_robot_position("r1", node)

    def robot2_node_cb(self, msg):

        node = msg.data
        self.update_robot_position("r2", node)

    # -----------------------------------------------------

    def update_robot_position(self, robot_id, node):

        r = self.robots[robot_id]

        r["node"] = node

        self.release_node(robot_id, node)

        # verificar se chegou ao destino
        if r["goal"] is not None and node == r["goal"]:
            self.goal_reached(robot_id)

    # -----------------------------------------------------

    def robot1_goal_cb(self, msg):

        if msg.data:
            self.goal_reached("r1")

    def robot2_goal_cb(self, msg):

        if msg.data:
            self.goal_reached("r2")

    # -----------------------------------------------------

    def goal_reached(self, robot_id):

        r = self.robots[robot_id]

        goal = r["goal"]

        rospy.loginfo(f"{robot_id} reached goal {goal}")

        state = self.build_state(robot_id)

        new_state = self.factory.update_state(state, goal)

        r["node"] = new_state[0]
        r["box"] = new_state[1]
        self.boxes = new_state[2]

        if goal in self.reserved_goals:
            del self.reserved_goals[goal]

        r["busy"] = False
        r["goal"] = None
        r["path"] = []

        self.plan_for_robot(robot_id)

    def sequence_to_boxtypes(self, seq):
        color_map = {'R': 0, 'G': 1, 'B': 2}
        boxtypes = []

        for c in seq.strip():
            c = c.upper()
            if c in color_map:
                boxtypes.append(color_map[c])
            else:
                rospy.logwarn(f"Cor inválida '{c}', ignorada")

        return boxtypes
    
    def shortest_path_avoiding(self, start, goal, blocked_nodes):
        """
        Dijkstra que evita nós em blocked_nodes.
        Permite start e goal mesmo que estejam no conjunto bloqueado.
        """
        blocked = set(blocked_nodes)
        blocked.discard(start)
        blocked.discard(goal)

        dist = {start: 0.0}
        prev = {start: None}
        pq = [(0.0, start)]

        while pq:
            curr_dist, u = heapq.heappop(pq)

            if curr_dist > dist.get(u, float("inf")):
                continue

            if u == goal:
                break

            for v, w in self.factory.graph.adj.get(u, []):
                if v in blocked:
                    continue

                new_dist = curr_dist + w
                if new_dist < dist.get(v, float("inf")):
                    dist[v] = new_dist
                    prev[v] = u
                    heapq.heappush(pq, (new_dist, v))

        if goal not in dist:
            return []

        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = prev[node]

        return path[::-1]
    
    def path_cost(self, path):
        if len(path) < 2:
            return 0.0

        total = 0.0
        for i in range(len(path) - 1):
            u = path[i]
            v = path[i + 1]

            for neigh, w in self.factory.graph.adj.get(u, []):
                if neigh == v:
                    total += w
                    break

        return total


if __name__ == "__main__":

    node = MultiPlannerNode()

    rospy.spin()