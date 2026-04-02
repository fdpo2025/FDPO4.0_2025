#pragma once

#include <vector>
#include <unordered_map>
#include <utility>

#include <yaml-cpp/yaml.h>

class Graph {
public:
    explicit Graph(const YAML::Node& graph_dict);

    const std::vector<std::pair<int, double>>& neighbors(int node) const;

    const std::unordered_map<int, double>& shortestDistances(int node_from);
    std::vector<int> shortestPath(int node_from, int node_to);
    double distance(int node_from, int node_to);

    std::unordered_map<int, std::vector<std::pair<int, double>>> adj;

private:
    void dijkstra(int node_from);

    std::unordered_map<int, std::unordered_map<int, double>> dist_cache_;
    std::unordered_map<int, std::unordered_map<int, int>> prev_cache_;

    static const std::vector<std::pair<int, double>> empty_neighbors_;
};
