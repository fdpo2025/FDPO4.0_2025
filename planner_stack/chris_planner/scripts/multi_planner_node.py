#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, UInt32, Int32MultiArray
import heapq

import os
import sys

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(CURRENT_DIR)

if PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_DIR)

import rospkg
rospack = rospkg.RosPack()
chris_planner_path = rospack.get_path('chris_planner')

modules_path = os.path.join(chris_planner_path, 'modules')
if os.path.exists(modules_path):
    sys.path.insert(0, os.path.dirname(modules_path))

try:
    import modules.planner as planner_module
    import modules.factory as factory_module
    import modules.yaml_utils as yaml_utils
except ImportError:
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

        self.special_block_nodes = set()

        for key, value in factory_components_dict.items():
            if isinstance(value, list):
                self.special_block_nodes.update(value)
            elif isinstance(value, dict):
                for subvalue in value.values():
                    if isinstance(subvalue, list):
                        self.special_block_nodes.update(subvalue)

        self.boxes = None

        self.robots = {
            "r1": {
                "node": 31,                # nó lógico
                "current_node": 31,        # nó físico atual
                "last_node": None,
                "box": factory_module.EMPTY,
                "goal": None,
                "path": [],                # path completo para reservas
                "compact_path": [],        # path reduzido para publicação
                "busy": False,
                "waiting_replan": False,
                "task_type": None,         # "pickup" ou "dropoff"
                "reserved_pickup_node": None
            },
            "r2": {
                "node": 31,
                "current_node": 31,
                "last_node": None,
                "box": factory_module.EMPTY,
                "goal": None,
                "path": [],
                "compact_path": [],
                "busy": False,
                "waiting_replan": False,
                "task_type": None,
                "reserved_pickup_node": None
            }
        }

        self.reserved_nodes = {}
        self.reserved_goals = {}

        # Regra para entregas intermitentes no armazém de saída
        self.output_nodes = {35, 36, 37, 38}        

        # Conta quantos pickups já foram planeados desde o início da sequência
        self.pickup_plan_count = 0

        self.pub_r1 = rospy.Publisher("/robot1_planned_paths", Int32MultiArray, queue_size=10)
        self.pub_r2 = rospy.Publisher("/robot2_planned_paths", Int32MultiArray, queue_size=10)
        self.pub_state = rospy.Publisher("/planner/logical_state", String, queue_size=10)

        rospy.Subscriber("/color_sequence", String, self.sequence_cb)
        rospy.Subscriber("/robot1/current_pose", UInt32, self.robot1_node_cb)
        rospy.Subscriber("/robot2/current_pose", UInt32, self.robot2_node_cb)

    # -----------------------------------------------------

    def publish_state_snapshot(self, robot_id, label, state):
        msg = String()
        msg.data = f"{robot_id} | {label} | logical state={state}"
        self.pub_state.publish(msg)

    # -----------------------------------------------------

    def publish_logical_state(self, robot_id):
        state = self.build_state(robot_id)
        self.publish_state_snapshot(robot_id, "current", state)

    # -----------------------------------------------------

    def sequence_cb(self, msg):

        seq = msg.data.strip()

        rospy.loginfo(f"Received sequence {seq}")

        boxtypes = self.sequence_to_boxtypes(seq)

        initial_state = self.factory.initial_state(boxtypes)
        _, _, self.boxes = initial_state

        self.reserved_nodes = {}
        self.reserved_goals = {}

        self.output_drop_plan_count = 0
        self.required_second_output_node = None
        self.pickup_plan_count = 0

        for robot_id in self.robots:
            self.robots[robot_id]["node"] = 31
            self.robots[robot_id]["current_node"] = 31
            self.robots[robot_id]["last_node"] = None
            self.robots[robot_id]["box"] = factory_module.EMPTY
            self.robots[robot_id]["goal"] = None
            self.robots[robot_id]["path"] = []
            self.robots[robot_id]["compact_path"] = []
            self.robots[robot_id]["busy"] = False
            self.robots[robot_id]["waiting_replan"] = False
            self.robots[robot_id]["task_type"] = None
            self.robots[robot_id]["reserved_pickup_node"] = None

        rospy.logwarn(f"Initial boxes = {self.boxes}")

        self.publish_logical_state("r1")
        self.publish_logical_state("r2")

        self.plan_for_robot("r1")
        self.plan_for_robot("r2")

    # -----------------------------------------------------

    def build_state(self, robot_id):

        r = self.robots[robot_id]
        return (r["node"], r["box"], self.boxes)

    # -----------------------------------------------------

    def plan_for_robot(self, robot_id):

        r = self.robots[robot_id]

        if r["busy"]:
            rospy.loginfo(f"{robot_id} is already busy")
            return False

        state = self.build_state(robot_id)
        self.publish_state_snapshot(robot_id, "before planning", state)

        robot_node = r["current_node"]

        rospy.loginfo(f"[{robot_id}] planning from physical node={robot_node}, logical state={state}")
        
        valid_nodes = self.factory.valid_destinations(state)
        rospy.loginfo(f"[{robot_id}] valid_nodes before filter = {valid_nodes}")

        unavailable_pickups = self.get_unavailable_pickup_nodes(robot_id)

        valid_nodes = [
            n for n in valid_nodes
            if n not in self.reserved_goals
            and n not in unavailable_pickups
        ]

        # Se estiver a fazer dropoff, aplicar regra especial do armazém de saída
        if r["box"] != factory_module.EMPTY:
            valid_nodes = self.apply_output_warehouse_rule(valid_nodes)

        rospy.loginfo(f"[{robot_id}] valid_nodes after filter = {valid_nodes}")

        if not valid_nodes:
            rospy.logwarn(f"No valid nodes for {robot_id}")
            r["waiting_replan"] = True
            self.publish_state_snapshot(robot_id, "planning failed - no valid nodes", state)
            return False

        best_path = None
        best_goal = None
        best_cost = float("inf")

        reserved_by_other = {
            n for n, owner in self.reserved_nodes.items()
            if owner != robot_id
        }

        extra_blocked = self.get_extra_blocked_nodes(robot_id)
        blocked_nodes = reserved_by_other | extra_blocked

        if r["box"] == factory_module.EMPTY:
            candidate_groups = self.split_pickup_candidates_by_priority(robot_id, valid_nodes)
        else:
            candidate_groups = [valid_nodes]

        rospy.loginfo(f"[{robot_id}] candidate_groups = {candidate_groups}")

        for candidate_nodes in candidate_groups:
            local_best_path = None
            local_best_goal = None
            local_best_cost = float("inf")

            for node in candidate_nodes:
                if r["box"] == factory_module.EMPTY:
                    # Restrição extra para vermelhas (TYPE_A -> máquina A)
                    if not self.can_pickup_box_for_machine(
                        robot_id=robot_id,
                        pickup_node=node,
                        source_box_type=factory_module.TYPE_A,
                        machine_inputs=self.factory.machineA_inputs,
                        machine_outputs=self.factory.machineA_outputs,
                        label="red"
                    ):
                        rospy.loginfo(f"[{robot_id}] pickup at node {node} blocked by red machine capacity")
                        continue

                    # Restrição extra para verdes (TYPE_B -> máquina B)
                    if not self.can_pickup_box_for_machine(
                        robot_id=robot_id,
                        pickup_node=node,
                        source_box_type=factory_module.TYPE_B,
                        machine_inputs=self.factory.machineB_inputs,
                        machine_outputs=self.factory.machineB_outputs,
                        label="green"
                    ):
                        rospy.loginfo(f"[{robot_id}] pickup at node {node} blocked by green machine capacity")
                        continue

                path = self.shortest_path_avoiding(robot_node, node, blocked_nodes)

                rospy.loginfo(f"[{robot_id}] candidate goal={node}, avoided path={path}")

                if not path:
                    continue

                cost = self.path_cost(path)

                if cost < local_best_cost:
                    local_best_cost = cost
                    local_best_path = path
                    local_best_goal = node

            # Se encontrou solução neste grupo prioritário, para aqui
            if local_best_path is not None:
                best_path = local_best_path
                best_goal = local_best_goal
                best_cost = local_best_cost
                break

        rospy.loginfo(f"[{robot_id}] best_path={best_path}, best_goal={best_goal}")

        if best_path is None:
            rospy.logwarn(f"No collision free path for {robot_id}")
            rospy.logwarn(f"[{robot_id}] reserved_nodes(all) = {dict(sorted(self.reserved_nodes.items()))}")
            rospy.logwarn(f"[{robot_id}] reserved_goals = {dict(sorted(self.reserved_goals.items()))}")
            rospy.logwarn(f"[{robot_id}] blocked_nodes = {sorted(blocked_nodes)}")
            r["waiting_replan"] = True
            self.publish_state_snapshot(robot_id, "planning failed - no path", state)
            return False

        task_type = "pickup" if r["box"] == factory_module.EMPTY else "dropoff"

        if task_type == "pickup":
            ok = self.reserve_box_at_planning(robot_id, best_goal)
            if not ok:
                rospy.logwarn(f"[{robot_id}] Failed to reserve box at node {best_goal}")
                r["waiting_replan"] = True
                self.publish_state_snapshot(robot_id, "planning failed - reserve pickup", state)
                return False

            self.pickup_plan_count += 1
            rospy.loginfo(f"[{robot_id}] pickup_plan_count = {self.pickup_plan_count}")

        full_path = self.extend_path_with_previous_node(best_path)
        compact_path = self.extend_path_with_previous_node(
            self.compact_existing_path(best_path)
        )

        self.reserve_path(robot_id, full_path, best_goal)
        self.publish_path(robot_id, compact_path)

        r["path"] = full_path
        r["compact_path"] = compact_path
        r["goal"] = best_goal
        r["busy"] = True
        r["waiting_replan"] = False
        r["task_type"] = task_type

        rospy.loginfo(f"{robot_id} planned full_path {full_path}")
        rospy.loginfo(f"{robot_id} planned compact_path {compact_path}")
        rospy.loginfo(f"{robot_id} task_type={task_type}")
        rospy.logwarn(f"[{robot_id}] boxes after planning = {self.boxes}")

        self.publish_logical_state(robot_id)
        return True

    # -----------------------------------------------------

    def reserve_box_at_planning(self, robot_id, pickup_node):
        r = self.robots[robot_id]

        if pickup_node not in self.factory.index_of:
            rospy.logerr(f"[{robot_id}] pickup node {pickup_node} is not a special node")
            return False

        i_pickup = self.factory.index_of[pickup_node]
        box_type = self.boxes[i_pickup]

        if box_type == factory_module.EMPTY:
            rospy.logerr(f"[{robot_id}] no box to reserve at node {pickup_node}")
            return False

        rospy.logwarn(f"[{robot_id}] reserving pickup node {pickup_node} (no state change yet)")
        r["reserved_pickup_node"] = pickup_node
        return True

    # -----------------------------------------------------

    def reserve_path(self, robot_id, path, goal):

        for n in path:
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
        released = False

        if node in self.reserved_nodes and self.reserved_nodes[node] == robot_id:
            del self.reserved_nodes[node]
            released = True

        return released

    # -----------------------------------------------------

    def robot1_node_cb(self, msg):
        self.update_robot_position("r1", msg.data)

    def robot2_node_cb(self, msg):
        self.update_robot_position("r2", msg.data)

    # -----------------------------------------------------

    def update_robot_position(self, robot_id, node):

        r = self.robots[robot_id]
        previous_physical_node = r["current_node"]

        if node == previous_physical_node:
            return

        r["last_node"] = previous_physical_node
        r["current_node"] = node

        released = self.release_nodes_before_current(robot_id, node)

        if released:
            self.try_replan_waiting_robot(robot_id)

        self.publish_logical_state(robot_id)

        if r["goal"] is not None and node == r["goal"]:
            rospy.logwarn(f"[{robot_id}] GOAL DETECTED at node {node}")
            self.goal_reached(robot_id)

    # -----------------------------------------------------

    def goal_reached(self, robot_id):

        r = self.robots[robot_id]
        goal = r["goal"]
        task_type = r["task_type"]

        self.release_all_path_nodes_except_current(robot_id, goal)

        rospy.loginfo(f"{robot_id} reached goal {goal} with task_type={task_type}")

        if task_type == "pickup":
            state = (r["node"], r["box"], self.boxes)
            self.publish_state_snapshot(robot_id, "before pickup update", state)
            rospy.logwarn(f"[{robot_id}] BEFORE pickup update: state={state}, goal={goal}")

            new_state = self.factory.update_state(state, goal)

            self.publish_state_snapshot(robot_id, "after pickup update", new_state)

            r["node"] = new_state[0]
            r["box"] = new_state[1]
            self.boxes = new_state[2]
            r["reserved_pickup_node"] = None

            rospy.logwarn(
                f"[{robot_id}] AFTER pickup update: node={r['node']} "
                f"box={r['box']} boxes={self.boxes}"
            )

        elif task_type == "dropoff":
            state = (r["node"], r["box"], self.boxes)
            self.publish_state_snapshot(robot_id, "before dropoff update", state)
            rospy.logwarn(f"[{robot_id}] BEFORE dropoff update: state={state}, goal={goal}")

            new_state = self.factory.update_state(state, goal)

            self.publish_state_snapshot(robot_id, "after dropoff update", new_state)

            r["node"] = new_state[0]
            r["box"] = new_state[1]
            self.boxes = new_state[2]
            r["reserved_pickup_node"] = None

            rospy.logwarn(
                f"[{robot_id}] AFTER dropoff update: node={r['node']} "
                f"box={r['box']} boxes={self.boxes}"
            )

        r["busy"] = False
        r["goal"] = None
        r["path"] = []
        r["compact_path"] = []
        r["task_type"] = None

        self.publish_logical_state(robot_id)

        self.plan_for_robot(robot_id)
        self.try_replan_waiting_robot(robot_id)

    # -----------------------------------------------------

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

    # -----------------------------------------------------

    def shortest_path_avoiding(self, start, goal, blocked_nodes):
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

    # -----------------------------------------------------

    def compact_existing_path(self, path):
        if not path:
            return []

        if len(path) <= 2:
            return list(path)

        def colinear(p1, p2, p3, eps=0.001):
            a = (p3[0] - p1[0], p3[1] - p1[1])
            b = (p2[0] - p1[0], p2[1] - p1[1])
            denom = (a[0] ** 2 + a[1] ** 2) ** 0.5
            if denom == 0:
                return True
            dist_to_line = abs(a[0] * b[1] - a[1] * b[0]) / denom
            return dist_to_line < eps

        path_compact = [path[0]]
        i = 1

        if path[0] in self.factory.special_nodes and len(path) > 1:
            path_compact.append(path[1])
            i += 1

        while i < len(path) - 2:
            last_idx = path_compact[-1]
            p1 = self.factory.points_map[last_idx]
            p2 = self.factory.points_map[path[i]]
            p3 = self.factory.points_map[path[i + 1]]

            if colinear(p1, p2, p3):
                i += 1
            else:
                path_compact.append(path[i])
                i += 1

        path_compact.extend(path[i:])
        return path_compact

    # -----------------------------------------------------

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

    # -----------------------------------------------------

    def try_replan_waiting_robot(self, freed_by_robot_id):
        other_robot = "r2" if freed_by_robot_id == "r1" else "r1"

        r = self.robots[other_robot]

        if r["busy"]:
            return

        if r["goal"] is not None:
            return

        if not r["waiting_replan"]:
            return

        rospy.loginfo(f"Trying replanning for waiting robot {other_robot}")
        self.plan_for_robot(other_robot)

    # -----------------------------------------------------

    def extend_path_with_previous_node(self, path):

        if len(path) < 2:
            return path

        return path + [path[-2]]

    # -----------------------------------------------------

    def get_extra_blocked_nodes(self, robot_id):
        blocked = set()

        reserved_by_other = {
            n for n, owner in self.reserved_nodes.items()
            if owner != robot_id
        }

        # Bloqueio dos vizinhos de nós especiais já existente
        for node in reserved_by_other:
            if node in self.special_block_nodes:
                for neigh, _ in self.factory.graph.adj.get(node, []):
                    blocked.add(neigh)

        # Regra antiga já existente
        if 12 in reserved_by_other or 26 in reserved_by_other:
            blocked.add(19)

        # Nova regra:
        # se o outro robô tiver no path os nós 11 e 27 consecutivos
        # (em qualquer ordem), bloquear o 19 enquanto ambos ainda estiverem reservados
        other_robot = "r2" if robot_id == "r1" else "r1"
        other_path = self.robots[other_robot]["path"]

        has_11_27_transition = False
        for i in range(len(other_path) - 1):
            a = other_path[i]
            b = other_path[i + 1]

            if (a == 11 and b == 27) or (a == 27 and b == 11):
                has_11_27_transition = True
                break

        if has_11_27_transition:
            both_still_reserved = (
                self.reserved_nodes.get(11) == other_robot and
                self.reserved_nodes.get(27) == other_robot
            )

            if both_still_reserved:
                blocked.add(19)
                rospy.loginfo(
                    f"[{robot_id}] blocking node 19 because {other_robot} "
                    f"has active transition 11<->27 and both nodes are still reserved"
                )

        return blocked

    # -----------------------------------------------------

    def apply_output_warehouse_rule(self, valid_nodes):
        output_candidates = [n for n in valid_nodes if n in self.output_nodes]

        # se não há destinos no armazém de saída, não há nada a fazer
        if not output_candidates:
            return valid_nodes

        # 1) verificar se já existe alguma caixa fisicamente colocada no output
        output_has_box = False
        for node in self.output_nodes:
            idx = self.factory.index_of[node]
            if self.boxes[idx] != factory_module.EMPTY:
                output_has_box = True
                break

        # 2) verificar se já existe algum robô com dropoff planeado para o output
        output_has_reserved_dropoff = False
        for other_id, other in self.robots.items():
            if (
                other["goal"] is not None
                and other["task_type"] == "dropoff"
                and other["goal"] in self.output_nodes
            ):
                output_has_reserved_dropoff = True
                break

        # enquanto não houver caixa colocada nem dropoff já planeado,
        # só permitir 36 e 37
        if not output_has_box and not output_has_reserved_dropoff:
            filtered = [
                n for n in valid_nodes
                if n not in self.output_nodes or n in {36, 37}
            ]

            rospy.loginfo(
                f"[OUTPUT_RULE] output vazio e sem dropoff reservado. "
                f"valid_nodes before={valid_nodes}, after={filtered}"
            )
            return filtered

        # a partir do momento em que já exista uma caixa no output
        # ou já exista um robô a caminho de um nó do output,
        # todos os nós do output ficam disponíveis
        rospy.loginfo(
            f"[OUTPUT_RULE] output já ativo "
            f"(has_box={output_has_box}, has_reserved_dropoff={output_has_reserved_dropoff}). "
            f"Todos os nós de saída disponíveis."
        )
        return valid_nodes

    # -----------------------------------------------------

    def release_all_path_nodes_except_current(self, robot_id, current_node):
        r = self.robots[robot_id]
        released_any = False

        if not r["path"]:
            return released_any

        if current_node in self.reserved_goals and self.reserved_goals[current_node] == robot_id:
            del self.reserved_goals[current_node]
            rospy.loginfo(f"[{robot_id}] released current reserved goal {current_node} on goal arrival")

        for n in r["path"]:
            if n == current_node:
                continue

            if n in self.reserved_nodes and self.reserved_nodes[n] == robot_id:
                del self.reserved_nodes[n]
                released_any = True
                rospy.loginfo(f"[{robot_id}] released path node {n} on goal arrival")

            if n in self.reserved_goals and self.reserved_goals[n] == robot_id:
                del self.reserved_goals[n]
                rospy.loginfo(f"[{robot_id}] released reserved goal {n} on goal arrival")

        return released_any

    # -----------------------------------------------------

    def release_nodes_before_current(self, robot_id, current_node):
        r = self.robots[robot_id]
        released_any = False

        if not r["path"]:
            return released_any

        path = list(r["path"])

        if current_node not in path:
            rospy.logwarn(f"[{robot_id}] current_node {current_node} not in reserved path {path}")
            return released_any

        current_idx = path.index(current_node)

        for n in path[:current_idx]:
            if n in self.reserved_nodes and self.reserved_nodes[n] == robot_id:
                del self.reserved_nodes[n]
                released_any = True
                rospy.loginfo(f"[{robot_id}] released past node {n}")

            if n in self.reserved_goals and self.reserved_goals[n] == robot_id:
                del self.reserved_goals[n]
                rospy.loginfo(f"[{robot_id}] released past reserved goal {n}")

        return released_any
    
    # -----------------------------------------------------

    def get_unavailable_pickup_nodes(self, robot_id):
        blocked = set()

        for other_id, other in self.robots.items():
            if other_id == robot_id:
                continue

            # se o outro robô está a carregar caixa, o nó lógico dele
            # não pode ser considerado pickup disponível
            if other["box"] != factory_module.EMPTY:
                blocked.add(other["node"])

            # se quiseres ser ainda mais conservador
            if other["goal"] is not None and other["task_type"] == "pickup":
                blocked.add(other["goal"])

            if other["reserved_pickup_node"] is not None:
                blocked.add(other["reserved_pickup_node"])

        return blocked

    # -----------------------------------------------------

    def get_box_type_at_node(self, node):
        if node not in self.factory.index_of:
            return None

        idx = self.factory.index_of[node]
        return self.boxes[idx]

    # -----------------------------------------------------

    def get_robot_active_task_box_type(self, robot_id):
        r = self.robots[robot_id]

        # Se já vai a transportar uma caixa, essa é a cor da tarefa ativa
        if r["box"] != factory_module.EMPTY:
            return r["box"]

        # Se ainda vai buscar a caixa mas já reservou pickup, usar a cor dessa caixa
        if r["reserved_pickup_node"] is not None:
            return self.get_box_type_at_node(r["reserved_pickup_node"])

        return None

    # -----------------------------------------------------

    def get_pickup_priority_mode(self, robot_id):
        """
        Devolve:
          - ("prefer_exact", box_type)
          - ("avoid_exact", box_type)
          - None
        """

        other_robot = "r2" if robot_id == "r1" else "r1"
        other_color = self.get_robot_active_task_box_type(other_robot)

        # Regra especial da 2ª tarefa da partida:
        # se a 1ª cor foi verde, tentar uma não-verde
        # se a 1ª cor foi não-verde, tentar verde
        if self.pickup_plan_count == 1 and other_color is not None:
            if other_color == factory_module.TYPE_B:
                rospy.loginfo(
                    f"[{robot_id}] second pickup priority: first task is GREEN, "
                    f"so prioritizing NON-GREEN"
                )
                return ("avoid_exact", factory_module.TYPE_B)
            else:
                rospy.loginfo(
                    f"[{robot_id}] second pickup priority: first task is NON-GREEN, "
                    f"so prioritizing GREEN"
                )
                return ("prefer_exact", factory_module.TYPE_B)

        # Regra geral:
        # evitar escolher a mesma cor do outro robô, se houver alternativa
        if other_color is not None:
            rospy.loginfo(
                f"[{robot_id}] other robot active color is {other_color}, "
                f"so prioritizing a different color"
            )
            return ("avoid_exact", other_color)

        return None

    # -----------------------------------------------------

    def split_pickup_candidates_by_priority(self, robot_id, valid_nodes):
        """
        Devolve grupos de candidatos por ordem de prioridade.
        Exemplo:
          [preferred_nodes, fallback_nodes]
        ou:
          [valid_nodes]
        """

        mode = self.get_pickup_priority_mode(robot_id)

        if mode is None:
            return [valid_nodes]

        mode_type, target_box_type = mode

        exact_nodes = []
        other_nodes = []
        unknown_nodes = []

        for node in valid_nodes:
            box_type = self.get_box_type_at_node(node)

            if box_type is None:
                unknown_nodes.append(node)
            elif box_type == target_box_type:
                exact_nodes.append(node)
            else:
                other_nodes.append(node)

        if mode_type == "prefer_exact":
            preferred = exact_nodes
            fallback = other_nodes + unknown_nodes

            rospy.loginfo(
                f"[{robot_id}] pickup priority prefer_exact({target_box_type}): "
                f"preferred={preferred}, fallback={fallback}"
            )

            if preferred:
                return [preferred, fallback] if fallback else [preferred]
            return [valid_nodes]

        if mode_type == "avoid_exact":
            preferred = other_nodes + unknown_nodes
            fallback = exact_nodes

            rospy.loginfo(
                f"[{robot_id}] pickup priority avoid_exact({target_box_type}): "
                f"preferred={preferred}, fallback={fallback}"
            )

            if preferred:
                return [preferred, fallback] if fallback else [preferred]
            return [valid_nodes]

        return [valid_nodes]
        
    # -----------------------------------------------------
    
    def can_pickup_box_for_machine(self, robot_id, pickup_node, source_box_type, machine_inputs, machine_outputs, label):
        if pickup_node not in self.factory.index_of:
            return False

        i_pickup = self.factory.index_of[pickup_node]
        box_type = self.boxes[i_pickup]

        # Esta restrição só se aplica ao tipo certo
        if box_type != source_box_type:
            return True

        # Contar quantas linhas da máquina estão realmente disponíveis
        # Uma linha está disponível se input e output estiverem ambos livres
        available_lines = 0

        for node_input, node_output in zip(machine_inputs, machine_outputs):
            i_input = self.factory.index_of[node_input]
            i_output = self.factory.index_of[node_output]

            if self.boxes[i_input] == factory_module.EMPTY and self.boxes[i_output] == factory_module.EMPTY:
                available_lines += 1

        # Contar quantas caixas deste tipo já estão em trânsito
        same_type_in_transit = 0

        for other_id, other in self.robots.items():
            if other_id == robot_id:
                continue

            # outro robô já transporta uma caixa deste tipo
            if other["box"] == source_box_type:
                same_type_in_transit += 1
                continue

            # outro robô já reservou pickup de uma caixa deste tipo
            if other["reserved_pickup_node"] is not None:
                reserved_node = other["reserved_pickup_node"]

                if reserved_node in self.factory.index_of:
                    idx = self.factory.index_of[reserved_node]
                    if self.boxes[idx] == source_box_type:
                        same_type_in_transit += 1

        rospy.loginfo(
            f"[{robot_id}] {label} pickup check at node {pickup_node}: "
            f"available_lines={available_lines}, same_type_in_transit={same_type_in_transit}"
        )

        return available_lines > same_type_in_transit


if __name__ == "__main__":

    node = MultiPlannerNode()
    rospy.spin()