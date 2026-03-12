#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int32, Bool, Int32MultiArray

from modules.planner import Planner


class MultiPlannerNode:

    def __init__(self):

        rospy.init_node("multi_planner_node")

        self.planner = Planner()
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

        rospy.Subscriber("/robot1/current_node", Int32, self.robot1_node_cb)
        rospy.Subscriber("/robot2/current_node", Int32, self.robot2_node_cb)
       
    # -----------------------------------------------------

    def sequence_cb(self, msg):

        seq = msg.data.strip()

        rospy.loginfo(f"Received sequence {seq}")

        boxtypes = self.factory.sequence_to_boxtypes(seq)

        self.boxes = self.factory.initial_state(boxtypes)

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

        valid_nodes = self.factory.valid_destinations(state)

        valid_nodes = [
            n for n in valid_nodes
            if n not in self.reserved_goals
        ]

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

            path = self.factory.shortest_path(robot_node, node)

            if any(n in reserved_by_other for n in path):
                continue

            cost = len(path)

            if cost < best_cost:
                best_cost = cost
                best_path = path
                best_goal = node

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


if __name__ == "__main__":

    node = MultiPlannerNode()

    rospy.spin()