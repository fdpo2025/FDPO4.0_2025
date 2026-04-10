#include "comunicacoes_node.h"

ComunicacoesNode::ComunicacoesNode(ros::NodeHandle& nh, int robot_id)
    : nh_(nh)
    , robot_id_(robot_id)
{
    pub_planned_paths_ = nh_.advertise<std_msgs::Int32MultiArray>("/planned_paths", 10);
    pub_robot1_pose_   = nh_.advertise<std_msgs::UInt32>("/robot1/current_pose", 10);
    pub_robot2_pose_   = nh_.advertise<std_msgs::UInt32>("/robot2/current_pose", 10);
    pub_cp_send_       = nh_.advertise<std_msgs::UInt32>("/cp_send", 10);
    pub_np_send_       = nh_.advertise<std_msgs::UInt32>("/np_send", 10);
    pub_wt_send_       = nh_.advertise<std_msgs::Bool>("/wt_send", 10);
    pub_target_id_send_ = nh_.advertise<std_msgs::UInt32>("/target_id_send", 10);
    pub_stop_waiting_send_ = nh_.advertise<std_msgs::Bool>("/stop_waiting_send", 10);

    // Fleet index 0-based (same as Pico ROBOT_ID / pi_pico_driver).
    if (robot_id_ == 0) {
        ROS_INFO("comunicacoes_node: fleet index 0 (hub / primary)");

        sub_r1_path_  = nh_.subscribe("/robot1_planned_paths", 10,
                                      &ComunicacoesNode::robot1PlannedPathCb, this);
        sub_this_pose_= nh_.subscribe("/this_current_pose", 10,
                                      &ComunicacoesNode::thisCurrentPoseRobot1Cb, this);
        sub_cp_rcv_   = nh_.subscribe("/cp_rcv", 10,
                                      &ComunicacoesNode::cpRcvCb, this);
        sub_np_rcv_   = nh_.subscribe("/np_rcv", 10,
                                      &ComunicacoesNode::npRcvCb, this);
        sub_wt_rcv_   = nh_.subscribe("/wt_rcv", 10,
                                      &ComunicacoesNode::wtRcvCb, this);
        sub_target_id_cmd_ = nh_.subscribe("/target_id_cmd", 10,
                                           &ComunicacoesNode::targetIdCmdCb, this);
        sub_stop_waiting_cmd_ = nh_.subscribe("/stop_waiting_cmd", 10,
                                              &ComunicacoesNode::stopWaitingCmdCb, this);

    } else if (robot_id_ == 1) {
        ROS_INFO("comunicacoes_node: fleet index 1 (secondary)");

        sub_this_pose_= nh_.subscribe("/this_current_pose", 10,
                                      &ComunicacoesNode::thisCurrentPoseRobot2Cb, this);

    } else {
        ROS_ERROR("comunicacoes_node: robot_id must be 0 or 1 (fleet index; same as Pico)");
        ros::shutdown();
    }
}

// =====================================================================
// Robot 1 callbacks
// =====================================================================

void ComunicacoesNode::robot1PlannedPathCb(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    ROS_INFO_THROTTLE(5.0, "[R1] Local path received");
    pub_planned_paths_.publish(*msg);
}

void ComunicacoesNode::thisCurrentPoseRobot1Cb(const std_msgs::UInt32::ConstPtr& msg)
{
    ROS_INFO_THROTTLE(5.0, "[R1] Local position received: %u", msg->data);
    pub_robot1_pose_.publish(*msg);
}

void ComunicacoesNode::cpRcvCb(const std_msgs::UInt32::ConstPtr& msg)
{
    ROS_INFO_THROTTLE(5.0, "[R1] Position received from robot 2: %u", msg->data);
    pub_robot2_pose_.publish(*msg);
}

void ComunicacoesNode::npRcvCb(const std_msgs::UInt32::ConstPtr& msg)
{
    ROS_INFO_THROTTLE(5.0, "[R1] NP recebido do robot remoto: %u", msg->data);
}

void ComunicacoesNode::wtRcvCb(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO_THROTTLE(5.0, "[R1] WT recebido do robot remoto: %u", msg->data ? 1 : 0);
}

void ComunicacoesNode::targetIdCmdCb(const std_msgs::UInt32::ConstPtr& msg)
{
    pub_target_id_send_.publish(*msg);
}

void ComunicacoesNode::stopWaitingCmdCb(const std_msgs::Bool::ConstPtr& msg)
{
    pub_stop_waiting_send_.publish(*msg);
}

// =====================================================================
// Robot 2 callbacks
// =====================================================================

void ComunicacoesNode::thisCurrentPoseRobot2Cb(const std_msgs::UInt32::ConstPtr& msg)
{
    ROS_INFO_THROTTLE(5.0, "[R2] Local position to send: %u", msg->data);
    pub_cp_send_.publish(*msg);
    std_msgs::UInt32 np_msg;
    np_msg.data = 0;
    pub_np_send_.publish(np_msg);

    std_msgs::Bool wt_msg;
    wt_msg.data = false;
    pub_wt_send_.publish(wt_msg);
}
