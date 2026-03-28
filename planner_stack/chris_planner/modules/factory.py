import modules.graph as g

EMPTY = -1
TYPE_A = 0
TYPE_B = 1
TYPE_C = 2

BT = {-1 : "EMPTY", 0 : "TYPE_A", 1 : "TYPE_B", 2 : "TYPE_C"}

class FactoryModel:
    def __init__(self, graph_dict, factory_components_dict):

        # create a graph object
        self.graph = g.Graph(graph_dict)

        # store the node_id -> (x, y) mapping
        self.points_map = {int(k): (float(p["x"]), float(p["y"])) for k, p in graph_dict["points_map"].items()}

        # store the components lists of nodes
        self.input_warehouse = [int(x) for x in factory_components_dict["input_warehouse"]]
        self.machineA_inputs = [int(x) for x in factory_components_dict["machineA"]["inputs"]]
        self.machineA_outputs = [int(x) for x in factory_components_dict["machineA"]["outputs"]]
        self.machineB_inputs = [int(x) for x in factory_components_dict["machineB"]["inputs"]]
        self.machineB_outputs = [int(x) for x in factory_components_dict["machineB"]["outputs"]]
        self.output_warehouse = [int(x) for x in factory_components_dict["output_warehouse"]]
        self.robot_start_node = int(factory_components_dict["robot_start_node"])

        # group the node lists of components into one single list of special nodes
        self.special_nodes = self.input_warehouse + self.machineA_inputs + self.machineA_outputs + \
                             self.machineB_inputs + self.machineB_outputs + self.output_warehouse
        # compute the reverse mapping
        self.index_of = {node_id : i for i, node_id in enumerate(self.special_nodes)}

    def validate_state(self, state):
        robot_node_id, robot_box_type, boxes = state
        if not isinstance(robot_node_id, int):
            raise ValueError("robot_node_id must be an integer")
        if not isinstance(robot_box_type, int):
            raise ValueError("robot_box_type must be an integer")
        if not isinstance(boxes, tuple):
            raise ValueError("boxes must be a tuple")
        if len(boxes) != len(self.special_nodes):
            raise ValueError("len(boxes) must be equal to len(self.special_nodes)")
        if robot_box_type not in [EMPTY, TYPE_A, TYPE_B, TYPE_C]:
            raise ValueError(f"{robot_box_type} not valid box type from {[EMPTY, TYPE_A, TYPE_B, TYPE_C]}")
        box_count = 0
        for box in boxes:
            if box != EMPTY:
                box_count += 1
        if box_count > len(self.output_warehouse):
            raise ValueError(f"box_count {box_count} must not exceed len(self.output_warehouse) {len(self.output_warehouse)}")
        '''
        # check if robot is carrying something that does not exist
        if robot_node_id in self.special_nodes:
            r_id = self.index_of[robot_node_id]
            box = boxes[r_id]
            if (robot_box_type != EMPTY) and (robot_box_type != box):
                raise ValueError(f"robot carrying box {BT[robot_box_type]}, but actual box is {BT[box]}.") 
        '''
        # don't allow boxes in machine inputs
        for i in range(len(boxes)):
            b = boxes[i]
            node = self.special_nodes[i]
            if (b != EMPTY) and (node in self.machineA_inputs + self.machineB_inputs):
                raise ValueError(f"box {BT[b]} at machine input, node {node}, not allowed.")
            
        # assert correct type in each node
        for i in range(len(boxes)):
            b = boxes[i]
            node = self.special_nodes[i]
            if (b == TYPE_A) and (node not in self.input_warehouse) or \
               (b == TYPE_B) and (node not in self.input_warehouse + self.machineA_outputs) or \
               (b == TYPE_C) and (node not in self.input_warehouse + self.machineB_outputs + self.output_warehouse):
                raise ValueError(f"boxtype {BT[b]} not allowed in node {node}.")


    def valid_destinations(self, state):
        robot_node_id, robot_box_type, boxes = state
        # when robot has a box
        
        valid_indices = []
        valid_boxtypes = []
        if robot_box_type in [TYPE_A, TYPE_B, TYPE_C]:
            if robot_box_type == TYPE_A:
                for i, node_id_input in enumerate(self.machineA_inputs):
                    node_id_output = self.machineA_outputs[i]
                    i_input = self.index_of[node_id_input]
                    i_output = self.index_of[node_id_output]
                    if (boxes[i_input] == EMPTY) and (boxes[i_output] == EMPTY):
                        valid_indices.append(i_input)
            elif robot_box_type == TYPE_B:
                for i, node_id_input in enumerate(self.machineB_inputs):
                    node_id_output = self.machineB_outputs[i]
                    i_input = self.index_of[node_id_input]
                    i_output = self.index_of[node_id_output]
                    if (boxes[i_input] == EMPTY) and (boxes[i_output] == EMPTY):
                        valid_indices.append(i_input)
            elif robot_box_type == TYPE_C:
                for i, node_id in enumerate(self.output_warehouse):
                    i_output = self.index_of[node_id]
                    if boxes[i_output] == EMPTY:
                        valid_indices.append(i_output)
        elif robot_box_type == EMPTY:
            # compute valid boxtypes
            for i, node_id_input in enumerate(self.machineA_inputs):
                node_id_output = self.machineA_outputs[i]
                i_input = self.index_of[node_id_input]
                i_output = self.index_of[node_id_output]
                if (boxes[i_input] == EMPTY) and (boxes[i_output] == EMPTY):
                    valid_boxtypes.append(0)
                    break
            for i, node_id_input in enumerate(self.machineB_inputs):
                node_id_output = self.machineB_outputs[i]
                i_input = self.index_of[node_id_input]
                i_output = self.index_of[node_id_output]
                if (boxes[i_input] == EMPTY) and (boxes[i_output] == EMPTY):
                    valid_boxtypes.append(1)
                    break
            for i, node_id in enumerate(self.output_warehouse):
                i_output = self.index_of[node_id]
                if boxes[i_output] == EMPTY:
                    valid_boxtypes.append(2)
                    break
            # compute valid indices
            for i, node_id in enumerate(self.special_nodes):
                # don't go to output warehouse without a box
                if node_id in self.output_warehouse:
                    continue
                if boxes[i] in valid_boxtypes:
                    valid_indices.append(i)
        
        return [self.special_nodes[i] for i in valid_indices]

    def update_state(self, state, node_to):
        robot_node_id, robot_box_type, boxes = state
        boxes_list = list(boxes)

        self.validate_state(state)

        valid_nodes = self.valid_destinations(state)
        if node_to not in valid_nodes:
            raise ValueError(f"{node_to} not a valid action from {valid_nodes}")

        i_to = self.index_of[node_to]

        # neste modelo, update_state é só para quando o robô já transporta uma caixa
        if robot_box_type == EMPTY:
            raise ValueError("update_state should not be used for pickup when using reservation-at-planning")

        # se robô tem caixa
        if robot_node_id in self.index_of:
            prev_idx = self.index_of[robot_node_id]
            boxes_list[prev_idx] = EMPTY

        if node_to in self.machineA_inputs:
            i = self.machineA_inputs.index(node_to)
            node_output = self.machineA_outputs[i]
            i_output = self.index_of[node_output]
            boxes_list[i_output] = robot_box_type + 1

        elif node_to in self.machineB_inputs:
            i = self.machineB_inputs.index(node_to)
            node_output = self.machineB_outputs[i]
            i_output = self.index_of[node_output]
            boxes_list[i_output] = robot_box_type + 1

        else:
            boxes_list[i_to] = robot_box_type

        robot_node_id = node_to
        robot_box_type = EMPTY

        return robot_node_id, robot_box_type, tuple(boxes_list)

    def cost(self, state, node_to):
        robot_node_id, robot_box_type, boxes = state
        return self.graph.distance(robot_node_id, node_to)

    def shortest_path(self, node_from, node_to, coords = False):
        path_indices = self.graph.shortest_path(node_from, node_to)
        if coords:
            return [(i, self.points_map[i]) for i in path_indices]
        else:
            return path_indices

    def shortest_path_compact(self, node_from, node_to, coords = False):
        path_indices = self.graph.shortest_path(node_from, node_to)
        if not path_indices:
            return []
        
        def colinear(p1, p2, p3, eps = 0.001):
            a = (p3[0]-p1[0], p3[1]-p1[1])
            b = (p2[0]-p1[0], p2[1]-p1[1])
            dist_to_line = abs(a[0]*b[1]-a[1]*b[0])/(a[0]**2+a[1]**2)**0.5
            return dist_to_line < eps

        path_indices_compact = [path_indices[0]]

        i = 1

        if node_from in self.special_nodes:
            path_indices_compact.append(path_indices[1])
            i+=1

        while i < len(path_indices)-2:
            last_idx = path_indices_compact[-1]
            p1 = self.points_map[last_idx]
            p2 = self.points_map[path_indices[i]]
            p3 = self.points_map[path_indices[i+1]]
            if colinear(p1, p2, p3):
                i += 1
            else:
                path_indices_compact.append(path_indices[i])
                i += 1
        path_indices_compact.extend(path_indices[i:])
        if coords:
            return [(i, self.points_map[i]) for i in path_indices_compact]
        else:
            return path_indices_compact


    def initial_state(self, boxtypes, robot_node_id = None):

        if robot_node_id is None:
            robot_node_id = self.robot_start_node

        # check if robot_node_id in the graph nodes
        if robot_node_id not in self.graph.adj:
            raise ValueError(f"{robot_node_id} not in {self.graph.adj.keys()}")
        # check if len of boxtypes is len of input warehouse
        if len(boxtypes) != len(self.input_warehouse):
            raise ValueError(f"len(boxtypes) {len(boxtypes)} must be equal to len(self.input_warehouse) {len(self.input_warehouse)}")
        # check boxtypes
        for boxtype in boxtypes:
            if boxtype not in [EMPTY, TYPE_A, TYPE_B, TYPE_C]:
                raise ValueError(f"{boxtype} not in {[EMPTY, TYPE_A, TYPE_B, TYPE_C]}")


        robot_box_type = EMPTY
        boxes_list = [EMPTY] * len(self.special_nodes)
        
        
        for i in range(len(boxtypes)):
            input_warehouse_node = self.input_warehouse[i]
            i_input = self.index_of[input_warehouse_node]
            boxes_list[i_input] = boxtypes[i]
        
        boxes = tuple(boxes_list)
        return robot_node_id, robot_box_type, boxes

    def terminal_state(self, state):
        robot_node_id, robot_box_type, boxes = state
        for i, box in enumerate(boxes):
            if (box == TYPE_A) or (box == TYPE_B):
                return False
            if (box == TYPE_C) and (self.special_nodes[i] not in self.output_warehouse):
                return False
        return True
    
    def state2dict(self, state):
        robot_node_id, robot_box_type, boxes = state
        state_dict = {}
        state_dict["robot_node_id"] = robot_node_id
        state_dict["robot_boxtype"] = robot_box_type
        state_dict["boxes"] = {}
        for i, boxtype in enumerate(boxes):
            if boxtype in [TYPE_A, TYPE_B, TYPE_C]:
                node = self.special_nodes[i]
                state_dict["boxes"][node] = boxtype
        return state_dict
    
    def dict2state(self, state_dict):
        robot_node_id = state_dict["robot_node_id"]
        robot_box_type = state_dict["robot_boxtype"]
        boxes = [state_dict["boxes"].get(node, EMPTY) for node in self.special_nodes]
        return robot_node_id, robot_box_type, tuple(boxes)
    
    def state_dict(self, boxtypes_dict, robot_boxtype=EMPTY, robot_node_id=None):
        
        if robot_node_id is None:
            robot_node_id = self.robot_start_node
        s_dict = {}
        s_dict["robot_node_id"] = robot_node_id
        s_dict["robot_boxtype"] = robot_boxtype
        s_dict["boxes"] = boxtypes_dict

        state = self.dict2state(s_dict)
        self.validate_state(state)

        return s_dict

        