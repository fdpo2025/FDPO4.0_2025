import heapq
import modules.yaml_utils as yu


class Graph:
    def __init__(self, graph_dict):
        graph = graph_dict
        adj = graph['adj']
        self.adj = {}
        for node_from, neighbors in adj.items():
            node_from = int(node_from)
            self.adj[node_from] = []
            for neighbor in neighbors:
                node_to = int(neighbor['node_to'])
                dist = float(neighbor['dist'])
                self.adj[node_from].append((node_to, dist))
        self.shortest_distances_cache = {}
        self.prev_node_cache = {}

    def shortest_distances(self, node_from):
        if node_from in self.shortest_distances_cache:
            return self.shortest_distances_cache[node_from]
        distances = {node_from: 0}
        prev_node = {node_from: None}
        queue = [(0, node_from)] # priority queue by (dist, node)
        while queue:
            (u_dist, u) = heapq.heappop(queue)
            if (u in distances) and distances[u] < u_dist: # outdated
                continue
            distances[u] = u_dist
            for (v, d) in self.adj.get(u, []):
                v_dist = u_dist + d
                if (v not in distances) or (v_dist < distances[v]):
                    distances[v] = v_dist
                    prev_node[v] = u
                    heapq.heappush(queue, (v_dist, v))
        self.shortest_distances_cache[node_from] = distances
        self.prev_node_cache[node_from] = prev_node
        return distances

    def shortest_path(self, node_from, node_to):
        if node_from not in self.shortest_distances_cache:
            self.shortest_distances(node_from)
        path = [node_to]
        while path[-1] != node_from:
            node_prev = self.prev_node_cache[node_from][path[-1]]
            path.append(node_prev)
        path = path[::-1]
        return path

    def distance(self, node_from, node_to):
        if node_from not in self.shortest_distances_cache:
            self.shortest_distances(node_from)
        return self.shortest_distances_cache[node_from][node_to]
    


    