#include "comunicacoes_node.h"

ComunicacoesNode::ComunicacoesNode(ros::NodeHandle& nh, int robot_id)
    : nh_(nh)
    , robot_id_(robot_id)
{
    pub_planned_paths_ = nh_.advertise<std_msgs::Int32MultiArray>("/planned_paths", 10);
    pub_robot1_pose_   = nh_.advertise<std_msgs::UInt32>("/robot1/current_pose", 10);
    pub_robot2_pose_   = nh_.advertise<std_msgs::UInt32>("/robot2/current_pose", 10);
    pub_cp_send_       = nh_.advertise<std_msgs::UInt32>("/cp_send", 10);
    pub_path_send_     = nh_.advertise<std_msgs::Int32MultiArray>("/path_send", 10);

    if (robot_id_ == 1) {
        ROS_INFO("Comunicacoes: modo ROBOT_ID = 1");

        sub_r1_path_  = nh_.subscribe("/robot1_planned_paths", 10,
                                      &ComunicacoesNode::robot1PlannedPathCb, this);
        sub_r2_path_  = nh_.subscribe("/robot2_planned_paths", 10,
                                      &ComunicacoesNode::robot2PlannedPathCb, this);
        sub_this_pose_= nh_.subscribe("/this_current_pose", 10,
                                      &ComunicacoesNode::thisCurrentPoseRobot1Cb, this);
        sub_cp_rcv_   = nh_.subscribe("/cp_rcv", 10,
                                      &ComunicacoesNode::cpRcvCb, this);

    } else if (robot_id_ == 2) {
        ROS_INFO("Comunicacoes: modo ROBOT_ID = 2");

        sub_this_pose_= nh_.subscribe("/this_current_pose", 10,
                                      &ComunicacoesNode::thisCurrentPoseRobot2Cb, this);
        sub_path_rcv_ = nh_.subscribe("/path_rcv", 10,
                                      &ComunicacoesNode::pathRcvCb, this);

    } else {
        ROS_ERROR("ROBOT_ID must be 1 or 2");
        ros::shutdown();
    }
}

// =====================================================================
// Robot 1 callbacks
// =====================================================================

void ComunicacoesNode::robot1PlannedPathCb(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    ROS_INFO("[R1] Local path received");
    pub_planned_paths_.publish(*msg);
}

void ComunicacoesNode::robot2PlannedPathCb(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    ROS_INFO("[R1] Path for robot 2 received");
    pub_path_send_.publish(*msg);
}

void ComunicacoesNode::thisCurrentPoseRobot1Cb(const std_msgs::UInt32::ConstPtr& msg)
{
    ROS_INFO("[R1] Local position received: %u", msg->data);
    pub_robot1_pose_.publish(*msg);
}

void ComunicacoesNode::cpRcvCb(const std_msgs::UInt32::ConstPtr& msg)
{
    ROS_INFO("[R1] Position received from robot 2: %u", msg->data);
    pub_robot2_pose_.publish(*msg);
}

// =====================================================================
// Robot 2 callbacks
// =====================================================================

void ComunicacoesNode::thisCurrentPoseRobot2Cb(const std_msgs::UInt32::ConstPtr& msg)
{
    if (r2_destination_ >= 0 && static_cast<int32_t>(msg->data) == r2_destination_) {
        ROS_INFO("[R2] Arrived at destination %d, sending CP", r2_destination_);
        pub_cp_send_.publish(*msg);
        r2_destination_ = -1;
    }
}

void ComunicacoesNode::pathRcvCb(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    ROS_INFO("[R2] Path received");
    if (!msg->data.empty()) {
        r2_destination_ = msg->data.back();
        ROS_INFO("[R2] Destination set to node %d", r2_destination_);
    }
    pub_planned_paths_.publish(*msg);
}
