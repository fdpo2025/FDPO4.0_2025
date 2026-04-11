#pragma once

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <plan_handler/NavPlan.h>
#include <plan_handler/ControllerPoint.h>
#include <plan_handler/CompletionFeedback.h>

#include <vector>
#include <map>
#include <set>

struct Pose {
    double x, y;
};

/** Parâmetros de follow line vindos de /follow_line/plan_handler (follow_line_parameters.yaml) */
struct PlanHandlerFollowLineConfig {
    double vel_lin_nom_warehouse_process = 0.025;
    double vel_lin_nom_warehouse_other = 0.06;
    double line_switch_before_pick = 0.95;
    double line_switch_plan_stack_before_pick = 1.0;
    double line_switch_drop_process = 0.95;
    double line_switch_drop_other = 0.60;
    double line_switch_after_warehouse_process = 1.0;
    double line_switch_after_warehouse = 0.7;
    double line_switch_normal = 0.75;
    double vel_lin_nom_after_warehouse = 0.1;
    double vel_lin_nom_normal = -1.0;
};

struct ControllerPoint {
    double x;
    double y;
    double line_switch_ratio = 0.75;
    double vel_lin_nom = -1.0;
    bool backwards = false;
    bool pick_box = false; // if false: drops box
    bool should_pub = false;
    bool is_warehouse = false; // Se o ponto final é uma warehouse
    bool is_process_warehouse = false;
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
    ros::Publisher targetIdSendPub;
    ros::Publisher stopWaitingSendPub;
    ros::Timer stopWaitingResetTimer;
    bool stopWaitingResetTimerValid{false};

    void plannedPathsCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void publishRadioStopWaitingPulse(uint32_t target_robot_id);
    void navCompletionFeedbackCallback(const plan_handler::CompletionFeedback::ConstPtr& msg);

    bool has_box;
    bool last_pick_box_state;
    bool is_last_warehouse, is_current_warehouse, fe_warehouse_coordinate;
    bool was_last_warehouse_process;

    // Mudança principal: mapa id -> coordenadas
    std::map<int, Pose> factory_coordinates;

    // Este pode continuar vector porque só usas índices 0..15
    std::vector<Pose> warehouse_coordinates;

    // Mudança principal: sets para marcar memberships
    std::set<int> is_warehouse_coordinate;
    std::set<int> is_process_warehouse;
    std::set<int> is_input_warehouse;
    std::set<int> is_output_warehouse;

    std::vector<ControllerPoint> plan_stack;

    PlanHandlerFollowLineConfig fl_;

    std::string planned_paths_topic;
    int queue_size;
};