import modules.factory as f
import heapq


class Planner:
    def __init__(self, graph_dict, factory_components_dict, method="closest"):
        self.factory = f.FactoryModel(graph_dict, factory_components_dict)
        self.graph = self.factory.graph

        self.method = method

        # map method string to policy function
        self.policies = {
            "astar": None,
            "closest": self.policy_closest,
            "onebyone": self.policy_onebyone,
            "greedy_h": self.policy_greedy_h,
        }

        self.h_cache = {}  # heuristic cache

        # nó lógico -> escolha entre dois nós físicos
        self.endpoint_pairs = {
            0:  {"access": 8,  "small": 0,   "large": 100, "small_if_prev_in": [9],        "large_if_prev_in": [16]},
            1:  {"access": 9,  "small": 1,   "large": 101, "small_if_prev_in": [10],       "large_if_prev_in": [8, 16]},
            2:  {"access": 10, "small": 2,   "large": 102, "small_if_prev_in": [11],       "large_if_prev_in": [9]},
            3:  {"access": 11, "small": 3,   "large": 103, "small_if_prev_in": [4, 12, 27],    "large_if_prev_in": [10]},
            13: {"access": 12, "small": 13,  "large": 113, "small_if_prev_in": [4],        "large_if_prev_in": [11, 19]},
            14: {"access": 15, "small": 14,  "large": 114, "small_if_prev_in": [7],        "large_if_prev_in": [22]},
            17: {"access": 16, "small": 17,  "large": 117, "small_if_prev_in": [8],        "large_if_prev_in": [23]},
            18: {"access": 19, "small": 18,  "large": 118, "small_if_prev_in": [12, 20],   "large_if_prev_in": [26]},
            20: {"access": 19, "small": 20,  "large": 120, "small_if_prev_in": [12, 18],   "large_if_prev_in": [26]},
            21: {"access": 22, "small": 21,  "large": 121, "small_if_prev_in": [15],       "large_if_prev_in": [29, 30]},
            24: {"access": 23, "small": 24,  "large": 124, "small_if_prev_in": [16],       "large_if_prev_in": [31]},
            25: {"access": 26, "small": 25,  "large": 125, "small_if_prev_in": [19],       "large_if_prev_in": [27, 34]},
            35: {"access": 27, "small": 35,  "large": 135, "small_if_prev_in": [26, 34, 11],   "large_if_prev_in": [28]},
            36: {"access": 28, "small": 36,  "large": 136, "small_if_prev_in": [27],       "large_if_prev_in": [29]},
            37: {"access": 29, "small": 37,  "large": 137, "small_if_prev_in": [28],       "large_if_prev_in": [22, 30]},
            38: {"access": 30, "small": 38,  "large": 138, "small_if_prev_in": [29],       "large_if_prev_in": [22]},
        }

        self.compute_heuristic_cache()

    def methods(self):
        return [k for k in self.policies]

    def plan_initial(self, boxtypes, robot_start_id=None):
        initial_state = self.factory.initial_state(boxtypes, robot_start_id)
        return self.plan(initial_state)

    def plan(self, state):
        if self.method == "astar":
            return self.plan_astar(state)

        robot_node_id, robot_box_type, boxes = state
        high_level_path = [robot_node_id]
        low_level_paths = []
        low_level_paths_compact = []

        count = 0
        total_cost = 0

        while (not self.factory.terminal_state(state)) and (count <= 26):
            robot_node_id, robot_box_type, boxes = state
            node_to = self.policy(state)

            if node_to is None:
                print("node_to is none!!!")
                print(f"valid nodes: {self.factory.valid_destinations(state)}")
                break

            physical_node_to = self.resolve_physical_destination(robot_node_id, node_to)

            high_level_path.append(physical_node_to)
            low_level_paths.append(self.factory.shortest_path(robot_node_id, physical_node_to, coords=False))
            low_level_paths_compact.append(self.factory.shortest_path_compact(robot_node_id, physical_node_to, coords=False))

            state = self.factory.update_state(state, node_to)

            count += 1
            total_cost += self.graph.distance(robot_node_id, physical_node_to)

        return high_level_path, low_level_paths, low_level_paths_compact, total_cost

    def plan_step(self, state):
        node_to = self.policy(state)
        if node_to is None:
            return None, None, None, None

        physical_node_to = self.resolve_physical_destination(state[0], node_to)

        high_level_path = [state[0], physical_node_to]
        low_level_path = self.factory.shortest_path(state[0], physical_node_to)
        low_level_path_compact = self.factory.shortest_path_compact(state[0], physical_node_to)
        dist = self.graph.distance(state[0], physical_node_to)

        return high_level_path, low_level_path, low_level_path_compact, dist

    def policy(self, state):
        if self.policies.get(self.method, None) is None:
            return None
        return self.policies[self.method](state)

    def policy_closest(self, state):
        self.factory.validate_state(state)
        if self.factory.terminal_state(state):
            return None
        robot_node_id = state[0]
        valid_nodes = self.factory.valid_destinations(state)
        return min(valid_nodes, key=lambda node: self.graph.distance(robot_node_id, node))

    def policy_onebyone(self, state):
        self.factory.validate_state(state)
        if self.factory.terminal_state(state):
            return None
        robot_node_id, robot_box_type, _ = state
        valid_nodes = self.factory.valid_destinations(state)
        if robot_box_type == f.EMPTY:
            for node in valid_nodes:
                if node in self.factory.machineA_outputs or node in self.factory.machineB_outputs:
                    return node
        return min(valid_nodes, key=lambda node: self.graph.distance(robot_node_id, node))

    def policy_greedy_h(self, state):
        self.factory.validate_state(state)
        if self.factory.terminal_state(state):
            return None

        robot_node_id, _, _ = state
        valid_nodes = self.factory.valid_destinations(state)

        if not valid_nodes:
            return None

        final_node = None
        fmin = float("inf")
        for node_to in valid_nodes:
            state_to = self.factory.update_state(state, node_to)
            h = self.heuristic(state_to)
            g = self.graph.distance(robot_node_id, node_to)
            f_score = g + h

            if f_score < fmin:
                fmin = f_score
                final_node = node_to

        return final_node

    def resolve_physical_destination(self, node_from, logical_node_to):
        if logical_node_to not in self.endpoint_pairs:
            return logical_node_to

        pair = self.endpoint_pairs[logical_node_to]
        access = pair["access"]

        path_to_access = self.factory.shortest_path(node_from, access, coords=False)

        if not path_to_access or len(path_to_access) < 2:
            return pair["small"]

        prev_before_access = path_to_access[-2]

        small_if_prev_in = pair.get("small_if_prev_in", [])
        large_if_prev_in = pair.get("large_if_prev_in", [])

        if prev_before_access in small_if_prev_in:
            return pair["small"]

        if prev_before_access in large_if_prev_in:
            return pair["large"]

        d_small = self.graph.distance(node_from, pair["small"])
        d_large = self.graph.distance(node_from, pair["large"])
        return pair["small"] if d_small <= d_large else pair["large"]

    def logical_node(self, physical_node):
        for logical, pair in self.endpoint_pairs.items():
            if physical_node == pair["small"] or physical_node == pair["large"]:
                return logical
        return physical_node

    # -------------------#
    # heuristic
    # -------------------#
    def compute_heuristic_cache(self):
        # heuristic for TYPE_C
        self.h_cache[f.TYPE_C] = {}
        for node in self.factory.output_warehouse:
            i = self.factory.index_of[node]
            self.h_cache[f.TYPE_C][i] = 0

        for node in (self.factory.input_warehouse + self.factory.machineB_outputs):
            i = self.factory.index_of[node]
            self.h_cache[f.TYPE_C][i] = float("inf")
            for node_to in self.factory.output_warehouse:
                self.h_cache[f.TYPE_C][i] = min(
                    self.h_cache[f.TYPE_C][i],
                    self.graph.distance(node, node_to)
                )

        # heuristic for TYPE_B
        self.h_cache[f.TYPE_B] = {}
        for node in (self.factory.input_warehouse + self.factory.machineA_outputs):
            i = self.factory.index_of[node]
            self.h_cache[f.TYPE_B][i] = float("inf")
            for node_to, node_to_output in zip(self.factory.machineB_inputs, self.factory.machineB_outputs):
                j_output = self.factory.index_of[node_to_output]
                self.h_cache[f.TYPE_B][i] = min(
                    self.h_cache[f.TYPE_B][i],
                    self.graph.distance(node, node_to) + self.h_cache[f.TYPE_C][j_output]
                )

        # heuristic for TYPE_A
        self.h_cache[f.TYPE_A] = {}
        for node in self.factory.input_warehouse:
            i = self.factory.index_of[node]
            self.h_cache[f.TYPE_A][i] = float("inf")
            for node_to, node_to_output in zip(self.factory.machineA_inputs, self.factory.machineA_outputs):
                j_output = self.factory.index_of[node_to_output]
                self.h_cache[f.TYPE_A][i] = min(
                    self.h_cache[f.TYPE_A][i],
                    self.graph.distance(node, node_to) + self.h_cache[f.TYPE_B][j_output]
                )

    def heuristic(self, state):
        if self.factory.terminal_state(state):
            return 0

        robot_node_id, robot_box_type, boxes = state
        h_total = 0
        closest_box = float("inf")

        for i in range(len(boxes)):
            boxtype = boxes[i]
            if boxtype in [f.TYPE_A, f.TYPE_B, f.TYPE_C]:
                h_total += self.h_cache[boxtype][i]
                if (robot_box_type == f.EMPTY) and (self.h_cache[boxtype][i] > 0):
                    closest_box = min(
                        closest_box,
                        self.graph.distance(robot_node_id, self.factory.special_nodes[i])
                    )

        if robot_box_type == f.EMPTY:
            h_total += closest_box

        return h_total

    # --------------------#
    # astar search
    # --------------------#
    def plan_astar(self, state):
        g_cost = {state: 0.0}
        prev_state = {state: None}
        move_used = {}

        h = self.heuristic(state)
        f_score = g_cost[state] + h
        pq = [(f_score, g_cost[state], state)]

        final_state = None

        while pq:
            _, gu, u = heapq.heappop(pq)

            if g_cost.get(u, float("inf")) < gu:
                continue

            if self.factory.terminal_state(u):
                final_state = u
                break

            for av in self.factory.valid_destinations(u):
                physical_av = self.resolve_physical_destination(u[0], av)
                v = self.factory.update_state(u, av)
                gv = gu + self.graph.distance(u[0], physical_av)

                if gv < g_cost.get(v, float("inf")):
                    g_cost[v] = gv
                    prev_state[v] = u
                    move_used[v] = physical_av

                    hv = self.heuristic(v)
                    fv = gv + hv
                    heapq.heappush(pq, (fv, gv, v))

        if final_state is None:
            return [state[0]], [], [], 0

        high_level_path = []
        low_level_paths = []
        low_level_paths_compact = []

        curr_state = final_state
        while prev_state[curr_state] is not None:
            prev = prev_state[curr_state]
            physical_to = move_used[curr_state]

            high_level_path.append(physical_to)
            low_level_paths.append(self.factory.shortest_path(prev[0], physical_to, coords=False))
            low_level_paths_compact.append(self.factory.shortest_path_compact(prev[0], physical_to, coords=False))

            curr_state = prev

        high_level_path.append(state[0])

        high_level_path = high_level_path[::-1]
        low_level_paths = low_level_paths[::-1]
        low_level_paths_compact = low_level_paths_compact[::-1]

        total_cost = 0
        for i in range(1, len(high_level_path)):
            node_from = high_level_path[i - 1]
            node_to = high_level_path[i]
            total_cost += self.graph.distance(node_from, node_to)

        return high_level_path, low_level_paths, low_level_paths_compact, total_cost

    def convert_paths2path(self, low_level_paths):
        final_path = []
        for i, p in enumerate(low_level_paths):
            if i == 0:
                final_path.extend(p)
            else:
                final_path.extend(p[1:])
        return final_path