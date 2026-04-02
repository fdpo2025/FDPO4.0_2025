#pragma once

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <plan_handler/NavPlan.h>
#include <plan_handler/ControllerPoint.h>
#include <plan_handler/CompletionFeedback.h>
#include <vector>
#include <unordered_map>
#include <unordered_set>

struct Pose {

    double x, y;
};

struct ControllerPoint {

    double x;
    double y;
    double line_switch_ratio = 0.75;
    double vel_lin_nom = 0.1;
    bool backwards = false;
    bool pick_box = false; // if false: drops box
    bool should_pub = false;
    bool is_warehouse = false; // Se o ponto final é uma warehouse
    int node_id;
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
        bool last_pick_box_state;
        std::unordered_map<int, Pose> factory_coordinates;
        std::unordered_set<int> is_warehouse_coordinate;
        std::unordered_set<int> is_process_warehouse;
        std::unordered_set<int> is_input_warehouse;
        std::unordered_set<int> is_output_warehouse;
        bool is_last_warehouse, is_current_warehouse, fe_warehouse_coordinate;

        bool was_last_warehouse_process;
        std::vector<ControllerPoint> plan_stack;
        
        std::string planned_paths_topic;
        int queue_size;
};

