#pragma once

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <plan_handler/NavPlan.h>
#include <plan_handler/ControllerPoint.h>
#include <vector>

struct Pose {

    double x, y;
};

struct ControllerPoint {

    double x;
    double y;
    double line_switch_ratio = 0.75;
    double vel_lin_nom = 0.1;
    bool backwards = false;
   
};

class PlanHandlerNode {

    public:
        PlanHandlerNode(ros::NodeHandle& nh_);

    private:
        ros::NodeHandle& nh;

        ros::Subscriber plannedPathsSub;
        ros::Publisher navPlanPub;
        void plannedPathsCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);

        bool has_box, pick_box;
        std::vector<Pose> factory_coordinates;
        std::vector<Pose> warehouse_coordinates;
        std::vector<bool> is_warehouse_coordinate; // works like a hashtable 
        bool is_last_warehouse, is_current_warehouse, fe_warehouse_coordinate;   // falling edge

        std::vector<ControllerPoint> plan_stack;
        
        std::string planned_paths_topic;
        int queue_size;
};

