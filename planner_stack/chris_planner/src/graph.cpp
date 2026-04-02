#include "graph.h"

#include <queue>
#include <algorithm>
#include <limits>
#include <stdexcept>

const std::vector<std::pair<int, double>> Graph::empty_neighbors_;

Graph::Graph(const YAML::Node& graph_dict)
{
    const auto& adj_node = graph_dict["adj"];
    for (auto it = adj_node.begin(); it != adj_node.end(); ++it) {
        int node_from = it->first.as<int>();
        adj[node_from] = {};
        for (const auto& neighbor : it->second) {
            int node_to = neighbor["node_to"].as<int>();
            double dist = neighbor["dist"].as<double>();
            adj[node_from].emplace_back(node_to, dist);
        }
    }
}

const std::vector<std::pair<int, double>>& Graph::neighbors(int node) const
{
    auto it = adj.find(node);
    if (it != adj.end()) return it->second;
    return empty_neighbors_;
}

void Graph::dijkstra(int node_from)
{
    if (dist_cache_.count(node_from)) return;

    auto& distances = dist_cache_[node_from];
    auto& prev_node = prev_cache_[node_from];

    using Pair = std::pair<double, int>;
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> pq;

    distances[node_from] = 0.0;
    prev_node[node_from] = -1;
    pq.push({0.0, node_from});

    while (!pq.empty()) {
        auto [u_dist, u] = pq.top();
        pq.pop();

        auto dit = distances.find(u);
        if (dit != distances.end() && dit->second < u_dist) continue;

        for (auto& [v, d] : neighbors(u)) {
            double v_dist = u_dist + d;
            auto vit = distances.find(v);
            if (vit == distances.end() || v_dist < vit->second) {
                distances[v] = v_dist;
                prev_node[v] = u;
                pq.push({v_dist, v});
            }
        }
    }
}

const std::unordered_map<int, double>& Graph::shortestDistances(int node_from)
{
    dijkstra(node_from);
    return dist_cache_[node_from];
}

std::vector<int> Graph::shortestPath(int node_from, int node_to)
{
    dijkstra(node_from);
    auto& prev = prev_cache_[node_from];

    std::vector<int> path;
    int current = node_to;
    while (current != node_from) {
        path.push_back(current);
        auto it = prev.find(current);
        if (it == prev.end()) return {};
        current = it->second;
    }
    path.push_back(node_from);
    std::reverse(path.begin(), path.end());
    return path;
}

double Graph::distance(int node_from, int node_to)
{
    dijkstra(node_from);
    auto& distances = dist_cache_[node_from];
    auto it = distances.find(node_to);
    if (it == distances.end()) return std::numeric_limits<double>::infinity();
    return it->second;
}
