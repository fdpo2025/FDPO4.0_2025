#include "chris_planner_node.h"

#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <stdexcept>

ChrisPlannerNode::ChrisPlannerNode(ros::NodeHandle& nh)
    : nh_(nh)
    , running_(false)
{
    std::string package_path = ros::package::getPath("chris_planner");
    if (package_path.empty()) {
        ROS_FATAL("chris_planner: ros::package::getPath retornou vazio (ROS_PACKAGE_PATH?)");
        throw std::runtime_error("chris_planner: package path empty");
    }
    approach_topology_.loadFromFile(package_path +
                                    "/files/inputs/warehouse_approach_topology.yaml");

    std::string graph_file = package_path + "/files/inputs/graph.yaml";
    std::string factory_file = package_path + "/files/inputs/factory_components.yaml";

    ROS_INFO("Loading graph from: %s", graph_file.c_str());
    ROS_INFO("Loading factory components from: %s", factory_file.c_str());

    YAML::Node graph_dict = YAML::LoadFile(graph_file);
    YAML::Node factory_dict = YAML::LoadFile(factory_file);

    std::string planning_method;
    nh_.param<std::string>("planning_method", planning_method, "astar");
    ROS_INFO("Using planning method: %s", planning_method.c_str());

    planner_ = std::make_unique<Planner>(graph_dict, factory_dict, planning_method);
    ROS_INFO("Chris planner initialized successfully");

    const auto& f = planner_->factory;
    for (int n : f.input_warehouse)   warehouse_nodes_.insert(n);
    for (int n : f.machineA_inputs)   warehouse_nodes_.insert(n);
    for (int n : f.machineA_outputs)  warehouse_nodes_.insert(n);
    for (int n : f.machineB_inputs)   warehouse_nodes_.insert(n);
    for (int n : f.machineB_outputs)  warehouse_nodes_.insert(n);
    for (int n : f.output_warehouse)  warehouse_nodes_.insert(n);

    color_seq_sub_ = nh_.subscribe("/color_sequence", 1,
                                   &ChrisPlannerNode::colorSequenceCallback, this);
    planned_paths_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/planned_paths", 100, true);

    ROS_INFO("Chris planner node ready, waiting for /color_sequence messages...");
}

std::vector<int> ChrisPlannerNode::colorSequenceToBoxtypes(const std::string& seq) const
{
    std::vector<int> boxtypes;
    for (char c : seq) {
        char upper = std::toupper(c);
        switch (upper) {
            case 'R': boxtypes.push_back(TYPE_A); break;
            case 'G': boxtypes.push_back(TYPE_B); break;
            case 'B': boxtypes.push_back(TYPE_C); break;
            default:
                ROS_WARN("Unknown color '%c', skipping", c);
                break;
        }
    }
    return boxtypes;
}

void ChrisPlannerNode::colorSequenceCallback(const std_msgs::String::ConstPtr& msg)
{
    if (running_) {
        ROS_WARN("Planner is already running, ignoring new request");
        return;
    }
    running_ = true;

    std::string color_seq = msg->data;
    ROS_INFO("Received color sequence: %s", color_seq.c_str());

    try {
        auto boxtypes = colorSequenceToBoxtypes(color_seq);
        ROS_INFO("Converted to %zu boxtypes", boxtypes.size());

        if (boxtypes.empty()) {
            ROS_WARN("No valid boxtypes found in color sequence");
            running_ = false;
            return;
        }

        PlanResult result = planner_->planInitial(boxtypes);

        ROS_INFO("Planning completed. High-level path length: %zu", result.high_level_path.size());
        ROS_INFO("Total cost: %.3f", result.total_cost);

        auto final_path = resolveApproachSides(
            planner_->convertPaths2Path(result.low_level_paths_compact));

        ROS_INFO("Final path length: %zu", final_path.size());

        std_msgs::Int32MultiArray path_msg;
        path_msg.data = final_path;
        planned_paths_pub_.publish(path_msg);

        ROS_INFO("Published path with %zu nodes to /planned_paths", final_path.size());

    } catch (const std::exception& e) {
        ROS_ERROR("Error during planning: %s", e.what());
    }

    running_ = false;
}

std::vector<int> ChrisPlannerNode::resolveApproachSides(const std::vector<int>& path) const
{
    auto result = path;
    for (int idx = 0; idx < static_cast<int>(result.size()); ++idx) {
        if (!warehouse_nodes_.count(result[idx])) continue;
        if (idx < 2) continue;

        int wh = result[idx];
        int neighbor = result[idx - 1];
        int pred = result[idx - 2];
        bool right = approach_topology_.isRightApproach(wh, neighbor, pred);
        result[idx] = right ? wh : wh + 100;
    }
    return result;
}
