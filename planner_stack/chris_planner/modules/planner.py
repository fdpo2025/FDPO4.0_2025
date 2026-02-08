import modules.factory as f
import heapq

class Planner:
    def __init__(self, graph_dict, factory_components_dict, method = "closest"):
        self.factory = f.FactoryModel(graph_dict, factory_components_dict)
        self.graph = self.factory.graph

        self.method = method

        # map method string to policy function
        self.policies = {
            "astar":None,
            "closest": self.policy_closest,
            "onebyone": self.policy_onebyone,
            "greedy_h": self.policy_greedy_h,
        }

        #self.policy_index = {}
        #for i, p in enumerate(self.policies):
        #    self.policy_index[p] = i

        self.h_cache = {} # heuristic cache
        self.compute_heuristic_cache()

    def methods(self):
        return [k for k in self.policies]


    def plan_initial(self, boxtypes, robot_start_id = None):
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
            #print(f"valid nodes: {planner.factory.valid_destinations(state)}")
            #print(f"node to: {node_to}")
            if node_to is None:
                print("node_to is none!!!")
                print(f"valid nodes: {self.factory.valid_destinations(state)}")
                break
            high_level_path.append(node_to)
            low_level_paths.append(self.factory.shortest_path(robot_node_id, node_to, coords=False))
            low_level_paths_compact.append(self.factory.shortest_path_compact(robot_node_id, node_to, coords=False))
            
            #print(f"valid actions: {planner.factory.valid_destinations(state)}")

            state = self.factory.update_state(state, node_to)
            
            #print(high_level_path)
            #print()
            #for path in low_level_paths:
            #    print(path)
            #print()

            count+=1
            total_cost += self.graph.distance(robot_node_id, node_to)
        
        return high_level_path, low_level_paths, low_level_paths_compact, total_cost

    def plan_step(self, state):
        node_to = self.policy(state)
        if node_to is None:
            return None, None, None, None
        high_level_path = [state[0], node_to]
        low_level_path = self.factory.shortest_path(state[0], node_to)
        low_level_path_compact = self.factory.shortest_path_compact(state[0], node_to)
        dist = self.graph.distance(state[0], node_to)
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
        return min(valid_nodes, key = lambda node : self.graph.distance(robot_node_id, node))

    def policy_onebyone(self, state):
        self.factory.validate_state(state)
        if self.factory.terminal_state(state):
            return None
        robot_node_id, robot_box_type, _ = state
        valid_nodes = self.factory.valid_destinations(state)
        if robot_box_type == f.EMPTY:
            for node in valid_nodes:
                if node in self.factory.machineA_outputs or \
                   node in self.factory.machineB_outputs:
                    return node
        # if no node picked yet, pick the closest one
        return min(valid_nodes, key = lambda node : self.graph.distance(robot_node_id, node))
    
    def policy_greedy_h(self, state):
        self.factory.validate_state(state)
        if self.factory.terminal_state(state):
            return None
        robot_node_id, _, _ = state
        valid_nodes = self.factory.valid_destinations(state)

        if not valid_nodes :   # <-- important
            return None

        final_node = None
        fmin = float("inf")
        for node_to in valid_nodes:
            state_to = self.factory.update_state(state, node_to)
            h = self.heuristic(state_to)
            g = self.graph.distance(robot_node_id, node_to)
            f = g + h
            #print(f"heusistics: h: {h}, g: {g}, f: {f}")
            if f < fmin:
                fmin = f
                final_node = node_to
        return final_node

    #-------------------#
    # heuristic
    #-------------------#
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
                j = self.factory.index_of[node_to]
                self.h_cache[f.TYPE_C][i] = min(self.h_cache[f.TYPE_C][i], self.graph.distance(node, node_to))
        
        # heuristic for TYPE_B
        self.h_cache[f.TYPE_B] = {}
        for node in (self.factory.input_warehouse + self.factory.machineA_outputs):
            i = self.factory.index_of[node]
            self.h_cache[f.TYPE_B][i] = float("inf")
            for node_to, node_to_output in zip(self.factory.machineB_inputs, self.factory.machineB_outputs):
                j = self.factory.index_of[node_to]
                j_output = self.factory.index_of[node_to_output]
                self.h_cache[f.TYPE_B][i] = min(self.h_cache[f.TYPE_B][i], self.graph.distance(node, node_to) + self.h_cache[f.TYPE_C][j_output])
        
        # heuristic for TYPE_A
        self.h_cache[f.TYPE_A] = {}
        for node in self.factory.input_warehouse:
            i = self.factory.index_of[node]
            self.h_cache[f.TYPE_A][i] = float("inf")
            for node_to, node_to_output in zip(self.factory.machineA_inputs, self.factory.machineA_outputs):
                j = self.factory.index_of[node_to]
                j_output = self.factory.index_of[node_to_output]
                self.h_cache[f.TYPE_A][i] = min(self.h_cache[f.TYPE_A][i], self.graph.distance(node, node_to) + self.h_cache[f.TYPE_B][j_output])


    def heuristic(self, state):
        if self.factory.terminal_state(state):
            return 0
        robot_node_id, robot_box_type, boxes = state
        h_total = 0
        closest_box = float('inf')
        for i in range(len(boxes)):
            boxtype = boxes[i]
            if boxtype in [f.TYPE_A, f.TYPE_B, f.TYPE_C]:
                h_total = h_total + self.h_cache[boxtype][i]
                if (robot_box_type == f.EMPTY) and \
                    self.h_cache[boxtype][i] > 0: # checking if the node is not at the output
                    closest_box = min(closest_box, self.graph.distance(robot_node_id, self.factory.special_nodes[i]))
        if robot_box_type == f.EMPTY:
            h_total = h_total + closest_box
        return h_total
    
    #--------------------#
    # astar search
    #--------------------#
    def plan_astar(self, state):
        g = {state: 0.0} # dict with shortest paths
        prev_state = {state : None}
        h = self.heuristic(state)
        f = g[state] + h
        pq = [(f, g[state], state)]

        final_state = None

        while pq:
            _, gu, u = heapq.heappop(pq)
            if g.get(u, float('inf')) < gu:
                continue
            g[u] = gu
            if self.factory.terminal_state(u):
                final_state = u
                break
            # expand
            for av in self.factory.valid_destinations(u):
                v = self.factory.update_state(u, av)
                gv = gu + self.graph.distance(u[0], v[0])
                if gv < g.get(v, float('inf')):
                    g[v] = gv
                    prev_state[v] = u
                    hv = self.heuristic(v)
                    fv = gv + hv
                    heapq.heappush(pq, (fv, gv, v))
        
        # reconstruct backwards
        high_level_path = [final_state[0]]
        low_level_paths = []
        low_level_paths_compact = []
        curr_state = prev_state[final_state]
        while curr_state:
            high_level_path.append(curr_state[0])
            curr_state = prev_state[curr_state]
            a = high_level_path[-1]
            b = high_level_path[-2]
            low_level_paths_compact.append(self.factory.shortest_path_compact(a, b, coords=False))
            low_level_paths.append(self.factory.shortest_path(a, b, coords=False))
        
        # reverse
        high_level_path = high_level_path[::-1]
        low_level_paths = low_level_paths[::-1]
        low_level_paths_compact = low_level_paths_compact[::-1]

        # total cost
        total_cost = 0
        for i in range(1, len(high_level_path)):
            node_from = high_level_path[i-1]
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
