#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>

#include "planner.h"

#include <string>
#include <memory>

class ChrisPlannerNode {
public:
    explicit ChrisPlannerNode(ros::NodeHandle& nh);

private:
    void colorSequenceCallback(const std_msgs::String::ConstPtr& msg);
    std::vector<int> colorSequenceToBoxtypes(const std::string& seq) const;

    ros::NodeHandle& nh_;
    ros::Subscriber color_seq_sub_;
    ros::Publisher planned_paths_pub_;

    std::unique_ptr<Planner> planner_;
    bool running_;
};
