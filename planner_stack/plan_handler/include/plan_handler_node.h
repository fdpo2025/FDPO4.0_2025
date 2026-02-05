#pragma once

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <plan_handler/NavPlan.h>
#include <plan_handler/ControllerPoint.h>
#include <plan_handler/CompletionFeedback.h>
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
    bool pick_box = false; // if falss: drops box
    bool should_pub = false;
   
};

class PlanHandlerNode {

    public:
        PlanHandlerNode(ros::NodeHandle& nh_);

    private:
        ros::NodeHandle& nh;

        ros::Subscriber plannedPathsSub;
        ros::Subscriber navCompletionFeedbackSub;
        ros::Publisher navPlanPub;
        ros::Publisher pickBoxPub;
        void plannedPathsCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
        void navCompletionFeedbackCallback(const plan_handler::CompletionFeedback::ConstPtr& msg);

        bool has_box;
        bool last_pick_box_state;  // Estado anterior do pick_box para detectar mudanças
        std::vector<Pose> factory_coordinates;
        std::vector<Pose> warehouse_coordinates;
        std::vector<bool> is_warehouse_coordinate; // works like a hashtable 
        bool is_last_warehouse, is_current_warehouse, fe_warehouse_coordinate;   // falling edge

        std::vector<ControllerPoint> plan_stack;
        
        std::string planned_paths_topic;
        int queue_size;
};

