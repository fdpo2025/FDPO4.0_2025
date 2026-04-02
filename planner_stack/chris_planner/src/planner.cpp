#include "planner.h"

#include <queue>
#include <algorithm>
#include <iostream>
#include <limits>
#include <tuple>

Planner::Planner(const YAML::Node& graph_dict,
                 const YAML::Node& factory_components_dict,
                 const std::string& method)
    : factory(graph_dict, factory_components_dict)
    , method_(method)
{
    computeHeuristicCache();
}

PlanResult Planner::planInitial(const std::vector<int>& boxtypes, int robot_start_id)
{
    State initial = factory.initialState(boxtypes, robot_start_id);
    return plan(initial);
}

PlanResult Planner::plan(const State& state)
{
    if (method_ == "astar")
        return planAstar(state);

    PlanResult result;
    result.high_level_path.push_back(state.robot_node);
    result.total_cost = 0.0;

    State current = state;
    int count = 0;

    while (!factory.terminalState(current) && count <= 26) {
        int node_to = policy(current);
        if (node_to < 0) {
            std::cerr << "node_to is none!!!" << std::endl;
            break;
        }
        result.high_level_path.push_back(node_to);
        result.low_level_paths.push_back(factory.shortestPath(current.robot_node, node_to));
        result.low_level_paths_compact.push_back(factory.shortestPathCompact(current.robot_node, node_to));

        result.total_cost += factory.graph.distance(current.robot_node, node_to);
        current = factory.updateState(current, node_to);
        ++count;
    }

    return result;
}

int Planner::policy(const State& state)
{
    if (method_ == "closest")   return policyClosest(state);
    if (method_ == "onebyone")  return policyOneByOne(state);
    if (method_ == "greedy_h")  return policyGreedyH(state);
    return -1;
}

int Planner::policyClosest(const State& state)
{
    factory.validateState(state);
    if (factory.terminalState(state)) return -1;

    auto valid = factory.validDestinations(state);
    if (valid.empty()) return -1;

    int best = valid[0];
    double best_dist = factory.graph.distance(state.robot_node, best);
    for (int i = 1; i < static_cast<int>(valid.size()); ++i) {
        double d = factory.graph.distance(state.robot_node, valid[i]);
        if (d < best_dist) { best_dist = d; best = valid[i]; }
    }
    return best;
}

int Planner::policyOneByOne(const State& state)
{
    factory.validateState(state);
    if (factory.terminalState(state)) return -1;

    auto valid = factory.validDestinations(state);
    if (valid.empty()) return -1;

    if (state.robot_box_type == EMPTY) {
        std::unordered_set<int> machA_out(factory.machineA_outputs.begin(), factory.machineA_outputs.end());
        std::unordered_set<int> machB_out(factory.machineB_outputs.begin(), factory.machineB_outputs.end());
        for (int node : valid)
            if (machA_out.count(node) || machB_out.count(node))
                return node;
    }

    int best = valid[0];
    double best_dist = factory.graph.distance(state.robot_node, best);
    for (int i = 1; i < static_cast<int>(valid.size()); ++i) {
        double d = factory.graph.distance(state.robot_node, valid[i]);
        if (d < best_dist) { best_dist = d; best = valid[i]; }
    }
    return best;
}

int Planner::policyGreedyH(const State& state)
{
    factory.validateState(state);
    if (factory.terminalState(state)) return -1;

    auto valid = factory.validDestinations(state);
    if (valid.empty()) return -1;

    int final_node = -1;
    double fmin = std::numeric_limits<double>::infinity();

    for (int node_to : valid) {
        State state_to = factory.updateState(state, node_to);
        double h = heuristic(state_to);
        double g = factory.graph.distance(state.robot_node, node_to);
        double f = g + h;
        if (f < fmin) {
            fmin = f;
            final_node = node_to;
        }
    }
    return final_node;
}

// -------------------------
// Heuristic
// -------------------------

void Planner::computeHeuristicCache()
{
    constexpr double INF = std::numeric_limits<double>::infinity();

    // TYPE_C heuristic
    auto& hc = h_cache_[TYPE_C];
    for (int node : factory.output_warehouse)
        hc[factory.index_of.at(node)] = 0.0;

    {
        std::vector<int> sources;
        sources.insert(sources.end(), factory.input_warehouse.begin(), factory.input_warehouse.end());
        sources.insert(sources.end(), factory.machineB_outputs.begin(), factory.machineB_outputs.end());
        for (int node : sources) {
            int i = factory.index_of.at(node);
            hc[i] = INF;
            for (int node_to : factory.output_warehouse)
                hc[i] = std::min(hc[i], factory.graph.distance(node, node_to));
        }
    }

    // TYPE_B heuristic
    auto& hb = h_cache_[TYPE_B];
    {
        std::vector<int> sources;
        sources.insert(sources.end(), factory.input_warehouse.begin(), factory.input_warehouse.end());
        sources.insert(sources.end(), factory.machineA_outputs.begin(), factory.machineA_outputs.end());
        for (int node : sources) {
            int i = factory.index_of.at(node);
            hb[i] = INF;
            for (int k = 0; k < static_cast<int>(factory.machineB_inputs.size()); ++k) {
                int node_to = factory.machineB_inputs[k];
                int node_to_output = factory.machineB_outputs[k];
                int j_output = factory.index_of.at(node_to_output);
                hb[i] = std::min(hb[i],
                    factory.graph.distance(node, node_to) + hc[j_output]);
            }
        }
    }

    // TYPE_A heuristic
    auto& ha = h_cache_[TYPE_A];
    for (int node : factory.input_warehouse) {
        int i = factory.index_of.at(node);
        ha[i] = INF;
        for (int k = 0; k < static_cast<int>(factory.machineA_inputs.size()); ++k) {
            int node_to = factory.machineA_inputs[k];
            int node_to_output = factory.machineA_outputs[k];
            int j_output = factory.index_of.at(node_to_output);
            ha[i] = std::min(ha[i],
                factory.graph.distance(node, node_to) + hb[j_output]);
        }
    }
}

double Planner::heuristic(const State& state)
{
    if (factory.terminalState(state)) return 0.0;

    double h_total = 0.0;
    double closest_box = std::numeric_limits<double>::infinity();

    for (int i = 0; i < static_cast<int>(state.boxes.size()); ++i) {
        int bt = state.boxes[i];
        if (bt == TYPE_A || bt == TYPE_B || bt == TYPE_C) {
            auto it = h_cache_.find(bt);
            if (it != h_cache_.end()) {
                auto jt = it->second.find(i);
                if (jt != it->second.end()) {
                    h_total += jt->second;
                    if (state.robot_box_type == EMPTY && jt->second > 0.0)
                        closest_box = std::min(closest_box,
                            factory.graph.distance(state.robot_node, factory.special_nodes[i]));
                }
            }
        }
    }

    if (state.robot_box_type == EMPTY)
        h_total += closest_box;

    return h_total;
}

// -------------------------
// A* search
// -------------------------

PlanResult Planner::planAstar(const State& initial_state)
{
    using Entry = std::tuple<double, double, State>;
    auto cmp = [](const Entry& a, const Entry& b) {
        return std::get<0>(a) > std::get<0>(b);
    };
    std::priority_queue<Entry, std::vector<Entry>, decltype(cmp)> pq(cmp);

    std::unordered_map<State, double, StateHash> g;
    std::unordered_map<State, State, StateHash> prev_state;

    g[initial_state] = 0.0;
    double h0 = heuristic(initial_state);
    pq.push({h0, 0.0, initial_state});

    State final_state;
    bool found = false;

    while (!pq.empty()) {
        auto [f_val, gu, u] = pq.top();
        pq.pop();

        auto it = g.find(u);
        if (it != g.end() && it->second < gu) continue;

        if (factory.terminalState(u)) {
            final_state = u;
            found = true;
            break;
        }

        for (int av : factory.validDestinations(u)) {
            State v = factory.updateState(u, av);
            double gv = gu + factory.graph.distance(u.robot_node, v.robot_node);
            auto vit = g.find(v);
            if (vit == g.end() || gv < vit->second) {
                g[v] = gv;
                prev_state[v] = u;
                double hv = heuristic(v);
                pq.push({gv + hv, gv, v});
            }
        }
    }

    // Reconstruct backwards
    PlanResult result;
    result.total_cost = 0.0;

    if (!found) return result;

    std::vector<int> hlp = {final_state.robot_node};
    State curr = final_state;
    while (prev_state.count(curr)) {
        State prev = prev_state[curr];
        hlp.push_back(prev.robot_node);
        curr = prev;
    }
    std::reverse(hlp.begin(), hlp.end());
    result.high_level_path = hlp;

    for (int i = 0; i < static_cast<int>(hlp.size()) - 1; ++i) {
        int a = hlp[i], b = hlp[i + 1];
        result.low_level_paths.push_back(factory.shortestPath(a, b));
        result.low_level_paths_compact.push_back(factory.shortestPathCompact(a, b));
        result.total_cost += factory.graph.distance(a, b);
    }

    return result;
}

std::vector<int> Planner::convertPaths2Path(const std::vector<std::vector<int>>& paths) const
{
    std::vector<int> final_path;
    for (int i = 0; i < static_cast<int>(paths.size()); ++i) {
        if (i == 0)
            final_path.insert(final_path.end(), paths[i].begin(), paths[i].end());
        else
            final_path.insert(final_path.end(), paths[i].begin() + 1, paths[i].end());
    }
    return final_path;
}
