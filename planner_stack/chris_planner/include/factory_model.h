#pragma once

#include "graph.h"

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <tuple>
#include <functional>
#include <cmath>

constexpr int EMPTY  = -1;
constexpr int TYPE_A =  0;
constexpr int TYPE_B =  1;
constexpr int TYPE_C =  2;

inline std::string boxTypeName(int bt) {
    switch (bt) {
        case EMPTY:  return "EMPTY";
        case TYPE_A: return "TYPE_A";
        case TYPE_B: return "TYPE_B";
        case TYPE_C: return "TYPE_C";
        default:     return "UNKNOWN";
    }
}

struct State {
    int robot_node;
    int robot_box_type;
    std::vector<int> boxes;
    size_t cached_hash = 0;

    bool operator==(const State& o) const {
        return robot_node == o.robot_node &&
               robot_box_type == o.robot_box_type &&
               boxes == o.boxes;
    }

    void recomputeHash() {
        size_t h = std::hash<int>()(robot_node);
        h ^= std::hash<int>()(robot_box_type) + 0x9e3779b9 + (h << 6) + (h >> 2);
        for (int b : boxes)
            h ^= std::hash<int>()(b) + 0x9e3779b9 + (h << 6) + (h >> 2);
        cached_hash = h;
    }
};

struct StateHash {
    size_t operator()(const State& s) const {
        return s.cached_hash;
    }
};

class FactoryModel {
public:
    FactoryModel(const YAML::Node& graph_dict, const YAML::Node& factory_components_dict);

    void validateState(const State& state) const;
    std::vector<int> validDestinations(const State& state) const;
    State updateState(const State& state, int node_to) const;
    bool terminalState(const State& state) const;

    State initialState(const std::vector<int>& boxtypes, int robot_node_id = -1) const;

    std::vector<int> shortestPath(int node_from, int node_to) const;
    std::vector<int> shortestPathCompact(int node_from, int node_to, const std::vector<int>& precomputed_path = {}) const;
    double cost(const State& state, int node_to) const;

    Graph graph;
    std::unordered_map<int, std::pair<double, double>> points_map;

    std::vector<int> input_warehouse;
    std::vector<int> machineA_inputs;
    std::vector<int> machineA_outputs;
    std::vector<int> machineB_inputs;
    std::vector<int> machineB_outputs;
    std::vector<int> output_warehouse;
    int robot_start_node;

    std::vector<int> special_nodes;
    std::unordered_map<int, int> index_of;

    std::unordered_set<int> machine_inputs_set;
    std::unordered_set<int> input_set;
    std::unordered_set<int> machA_out_set;
    std::unordered_set<int> machB_out_set;
    std::unordered_set<int> output_set;
    std::unordered_set<int> special_set;

private:
    static bool colinear(
        const std::pair<double,double>& p1,
        const std::pair<double,double>& p2,
        const std::pair<double,double>& p3,
        double eps = 0.001);
};
