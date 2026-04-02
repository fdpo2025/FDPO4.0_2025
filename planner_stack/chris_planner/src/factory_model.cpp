#include "factory_model.h"

#include <stdexcept>
#include <algorithm>
#include <sstream>
#include <cmath>

namespace {

std::vector<int> yamlToIntVec(const YAML::Node& node) {
    std::vector<int> v;
    for (const auto& item : node)
        v.push_back(item.as<int>());
    return v;
}

} // namespace

FactoryModel::FactoryModel(const YAML::Node& graph_dict,
                           const YAML::Node& factory_components_dict)
    : graph(graph_dict)
{
    const auto& pm = graph_dict["points_map"];
    for (auto it = pm.begin(); it != pm.end(); ++it) {
        int id = it->first.as<int>();
        double x = it->second["x"].as<double>();
        double y = it->second["y"].as<double>();
        points_map[id] = {x, y};
    }

    input_warehouse   = yamlToIntVec(factory_components_dict["input_warehouse"]);
    machineA_inputs   = yamlToIntVec(factory_components_dict["machineA"]["inputs"]);
    machineA_outputs  = yamlToIntVec(factory_components_dict["machineA"]["outputs"]);
    machineB_inputs   = yamlToIntVec(factory_components_dict["machineB"]["inputs"]);
    machineB_outputs  = yamlToIntVec(factory_components_dict["machineB"]["outputs"]);
    output_warehouse  = yamlToIntVec(factory_components_dict["output_warehouse"]);
    robot_start_node  = factory_components_dict["robot_start_node"].as<int>();

    // Build special_nodes = input + machineA_in + machineA_out + machineB_in + machineB_out + output
    auto append = [&](const std::vector<int>& v) {
        special_nodes.insert(special_nodes.end(), v.begin(), v.end());
    };
    append(input_warehouse);
    append(machineA_inputs);
    append(machineA_outputs);
    append(machineB_inputs);
    append(machineB_outputs);
    append(output_warehouse);

    for (int i = 0; i < static_cast<int>(special_nodes.size()); ++i)
        index_of[special_nodes[i]] = i;

    machine_inputs_set.insert(machineA_inputs.begin(), machineA_inputs.end());
    machine_inputs_set.insert(machineB_inputs.begin(), machineB_inputs.end());
    input_set.insert(input_warehouse.begin(), input_warehouse.end());
    machA_out_set.insert(machineA_outputs.begin(), machineA_outputs.end());
    machB_out_set.insert(machineB_outputs.begin(), machineB_outputs.end());
    output_set.insert(output_warehouse.begin(), output_warehouse.end());
    special_set.insert(special_nodes.begin(), special_nodes.end());
}

void FactoryModel::validateState(const State& state) const
{
    if (static_cast<int>(state.boxes.size()) != static_cast<int>(special_nodes.size()))
        throw std::runtime_error("boxes size must equal special_nodes size");

    if (state.robot_box_type != EMPTY && state.robot_box_type != TYPE_A &&
        state.robot_box_type != TYPE_B && state.robot_box_type != TYPE_C)
        throw std::runtime_error("Invalid robot_box_type: " + std::to_string(state.robot_box_type));

    int box_count = 0;
    for (int b : state.boxes)
        if (b != EMPTY) ++box_count;

    if (box_count > static_cast<int>(output_warehouse.size()))
        throw std::runtime_error("box_count exceeds output_warehouse size");

    auto it_idx = index_of.find(state.robot_node);
    if (it_idx != index_of.end()) {
        int r_id = it_idx->second;
        int box = state.boxes[r_id];
        if (state.robot_box_type != EMPTY && state.robot_box_type != box)
            throw std::runtime_error("Robot carrying " + boxTypeName(state.robot_box_type) +
                                     " but actual box is " + boxTypeName(box));
    }

    for (int i = 0; i < static_cast<int>(state.boxes.size()); ++i) {
        int b = state.boxes[i];
        int node = special_nodes[i];
        if (b != EMPTY && machine_inputs_set.count(node))
            throw std::runtime_error("Box " + boxTypeName(b) + " at machine input node " + std::to_string(node));
    }

    for (int i = 0; i < static_cast<int>(state.boxes.size()); ++i) {
        int b = state.boxes[i];
        int node = special_nodes[i];
        if (b == TYPE_A && !input_set.count(node))
            throw std::runtime_error("TYPE_A not allowed at node " + std::to_string(node));
        if (b == TYPE_B && !input_set.count(node) && !machA_out_set.count(node))
            throw std::runtime_error("TYPE_B not allowed at node " + std::to_string(node));
        if (b == TYPE_C && !input_set.count(node) && !machB_out_set.count(node) && !output_set.count(node))
            throw std::runtime_error("TYPE_C not allowed at node " + std::to_string(node));
    }
}

std::vector<int> FactoryModel::validDestinations(const State& state) const
{
    int robot_box_type = state.robot_box_type;
    const auto& boxes = state.boxes;

    std::vector<int> valid_indices;

    if (robot_box_type == TYPE_A || robot_box_type == TYPE_B || robot_box_type == TYPE_C) {
        if (robot_box_type == TYPE_A) {
            for (int i = 0; i < static_cast<int>(machineA_inputs.size()); ++i) {
                int i_input  = index_of.at(machineA_inputs[i]);
                int i_output = index_of.at(machineA_outputs[i]);
                if (boxes[i_input] == EMPTY && boxes[i_output] == EMPTY)
                    valid_indices.push_back(i_input);
            }
        } else if (robot_box_type == TYPE_B) {
            for (int i = 0; i < static_cast<int>(machineB_inputs.size()); ++i) {
                int i_input  = index_of.at(machineB_inputs[i]);
                int i_output = index_of.at(machineB_outputs[i]);
                if (boxes[i_input] == EMPTY && boxes[i_output] == EMPTY)
                    valid_indices.push_back(i_input);
            }
        } else { // TYPE_C
            for (int i = 0; i < static_cast<int>(output_warehouse.size()); ++i) {
                int i_output = index_of.at(output_warehouse[i]);
                if (boxes[i_output] == EMPTY)
                    valid_indices.push_back(i_output);
            }
        }
    } else { // EMPTY
        std::vector<int> valid_boxtypes;

        for (int i = 0; i < static_cast<int>(machineA_inputs.size()); ++i) {
            int i_input  = index_of.at(machineA_inputs[i]);
            int i_output = index_of.at(machineA_outputs[i]);
            if (boxes[i_input] == EMPTY && boxes[i_output] == EMPTY) {
                valid_boxtypes.push_back(TYPE_A);
                break;
            }
        }
        for (int i = 0; i < static_cast<int>(machineB_inputs.size()); ++i) {
            int i_input  = index_of.at(machineB_inputs[i]);
            int i_output = index_of.at(machineB_outputs[i]);
            if (boxes[i_input] == EMPTY && boxes[i_output] == EMPTY) {
                valid_boxtypes.push_back(TYPE_B);
                break;
            }
        }
        for (int i = 0; i < static_cast<int>(output_warehouse.size()); ++i) {
            int i_output = index_of.at(output_warehouse[i]);
            if (boxes[i_output] == EMPTY) {
                valid_boxtypes.push_back(TYPE_C);
                break;
            }
        }

        std::unordered_set<int> valid_bt_set(valid_boxtypes.begin(), valid_boxtypes.end());

        for (int i = 0; i < static_cast<int>(special_nodes.size()); ++i) {
            if (output_set.count(special_nodes[i]))
                continue;
            if (valid_bt_set.count(boxes[i]))
                valid_indices.push_back(i);
        }
    }

    std::vector<int> result;
    result.reserve(valid_indices.size());
    for (int idx : valid_indices)
        result.push_back(special_nodes[idx]);
    return result;
}

State FactoryModel::updateState(const State& state, int node_to) const
{
    validateState(state);

    auto valid = validDestinations(state);
    if (std::find(valid.begin(), valid.end(), node_to) == valid.end())
        throw std::runtime_error(std::to_string(node_to) + " not a valid destination");

    State new_state = state;
    int i_to = index_of.at(node_to);

    if (new_state.robot_box_type == EMPTY) {
        new_state.robot_node = node_to;
        new_state.robot_box_type = new_state.boxes[i_to];
    } else {
        int prev_idx = index_of.at(new_state.robot_node);
        new_state.boxes[prev_idx] = EMPTY;

        auto itA = std::find(machineA_inputs.begin(), machineA_inputs.end(), node_to);
        if (itA != machineA_inputs.end()) {
            int line = static_cast<int>(std::distance(machineA_inputs.begin(), itA));
            int out_node = machineA_outputs[line];
            int i_output = index_of.at(out_node);
            new_state.boxes[i_output] = new_state.robot_box_type + 1;
        } else {
            auto itB = std::find(machineB_inputs.begin(), machineB_inputs.end(), node_to);
            if (itB != machineB_inputs.end()) {
                int line = static_cast<int>(std::distance(machineB_inputs.begin(), itB));
                int out_node = machineB_outputs[line];
                int i_output = index_of.at(out_node);
                new_state.boxes[i_output] = new_state.robot_box_type + 1;
            } else {
                new_state.boxes[i_to] = new_state.robot_box_type;
            }
        }

        new_state.robot_node = node_to;
        new_state.robot_box_type = EMPTY;
    }

    new_state.recomputeHash();
    return new_state;
}

bool FactoryModel::terminalState(const State& state) const
{
    for (int i = 0; i < static_cast<int>(state.boxes.size()); ++i) {
        int b = state.boxes[i];
        if (b == TYPE_A || b == TYPE_B) return false;
        if (b == TYPE_C && !output_set.count(special_nodes[i])) return false;
    }
    return true;
}

State FactoryModel::initialState(const std::vector<int>& boxtypes, int robot_node_id) const
{
    if (robot_node_id < 0) robot_node_id = robot_start_node;

    if (graph.adj.find(robot_node_id) == graph.adj.end())
        throw std::runtime_error(std::to_string(robot_node_id) + " not in graph");

    if (static_cast<int>(boxtypes.size()) != static_cast<int>(input_warehouse.size()))
        throw std::runtime_error("boxtypes size must equal input_warehouse size");

    for (int bt : boxtypes)
        if (bt != EMPTY && bt != TYPE_A && bt != TYPE_B && bt != TYPE_C)
            throw std::runtime_error("Invalid boxtype: " + std::to_string(bt));

    State s;
    s.robot_node = robot_node_id;
    s.robot_box_type = EMPTY;
    s.boxes.assign(special_nodes.size(), EMPTY);

    for (int i = 0; i < static_cast<int>(boxtypes.size()); ++i) {
        int node = input_warehouse[i];
        int idx  = index_of.at(node);
        s.boxes[idx] = boxtypes[i];
    }

    s.recomputeHash();
    return s;
}

std::vector<int> FactoryModel::shortestPath(int node_from, int node_to) const
{
    return const_cast<Graph&>(graph).shortestPath(node_from, node_to);
}

bool FactoryModel::colinear(
    const std::pair<double,double>& p1,
    const std::pair<double,double>& p2,
    const std::pair<double,double>& p3,
    double eps)
{
    double ax = p3.first  - p1.first;
    double ay = p3.second - p1.second;
    double bx = p2.first  - p1.first;
    double by = p2.second - p1.second;
    double denom = std::sqrt(ax * ax + ay * ay);
    if (denom == 0.0) return true;
    double dist_to_line = std::abs(ax * by - ay * bx) / denom;
    return dist_to_line < eps;
}

std::vector<int> FactoryModel::shortestPathCompact(int node_from, int node_to, const std::vector<int>& precomputed_path) const
{
    std::vector<int> owned_path;
    if (precomputed_path.empty()) owned_path = shortestPath(node_from, node_to);
    const auto& path = precomputed_path.empty() ? owned_path : precomputed_path;
    if (path.empty()) return {};

    std::vector<int> compact;
    compact.push_back(path[0]);

    int i = 1;
    if (special_set.count(node_from) && static_cast<int>(path.size()) > 1) {
        compact.push_back(path[1]);
        i = 2;
    }

    while (i < static_cast<int>(path.size()) - 2) {
        int last_idx = compact.back();
        auto p1 = points_map.at(last_idx);
        auto p2 = points_map.at(path[i]);
        auto p3 = points_map.at(path[i + 1]);

        if (colinear(p1, p2, p3))
            ++i;
        else {
            compact.push_back(path[i]);
            ++i;
        }
    }

    for (int j = i; j < static_cast<int>(path.size()); ++j)
        compact.push_back(path[j]);

    return compact;
}

double FactoryModel::cost(const State& state, int node_to) const
{
    return const_cast<Graph&>(graph).distance(state.robot_node, node_to);
}
