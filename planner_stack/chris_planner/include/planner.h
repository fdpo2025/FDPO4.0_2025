#pragma once

#include "factory_model.h"

#include <vector>
#include <string>
#include <unordered_map>
#include <functional>
#include <limits>

struct PlanResult {
    std::vector<int> high_level_path;
    std::vector<std::vector<int>> low_level_paths;
    std::vector<std::vector<int>> low_level_paths_compact;
    double total_cost;
};

class Planner {
public:
    Planner(const YAML::Node& graph_dict,
            const YAML::Node& factory_components_dict,
            const std::string& method = "closest");

    PlanResult planInitial(const std::vector<int>& boxtypes, int robot_start_id = -1);
    PlanResult plan(const State& state);

    std::vector<int> convertPaths2Path(const std::vector<std::vector<int>>& paths) const;

    FactoryModel factory;

private:
    std::string method_;

    int policy(const State& state);
    int policyClosest(const State& state);
    int policyOneByOne(const State& state);
    int policyGreedyH(const State& state);

    PlanResult planAstar(const State& state);

    // Heuristic
    void computeHeuristicCache();
    double heuristic(const State& state);

    // h_cache_[box_type][special_node_index] = estimated cost
    std::unordered_map<int, std::unordered_map<int, double>> h_cache_;
};
