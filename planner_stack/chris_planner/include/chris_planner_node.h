#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>

#include "planner.h"
#include "warehouse_approach_topology.h"

#include <string>
#include <memory>
#include <unordered_set>

class ChrisPlannerNode {
public:
    explicit ChrisPlannerNode(ros::NodeHandle& nh);

private:
    void colorSequenceCallback(const std_msgs::String::ConstPtr& msg);
    std::vector<int> colorSequenceToBoxtypes(const std::string& seq) const;
    std::vector<int> resolveApproachSides(const std::vector<int>& path) const;

    ros::NodeHandle& nh_;
    ros::Subscriber color_seq_sub_;
    ros::Publisher planned_paths_pub_;

    std::unique_ptr<Planner> planner_;
    bool running_;
    std::unordered_set<int> warehouse_nodes_;
    WarehouseApproachTopology approach_topology_;
};
