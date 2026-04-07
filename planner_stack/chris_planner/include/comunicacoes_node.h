#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>

class ComunicacoesNode {
public:
    ComunicacoesNode(ros::NodeHandle& nh, int robot_id);

private:
    // Robot 1 callbacks
    void robot1PlannedPathCb(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void thisCurrentPoseRobot1Cb(const std_msgs::UInt32::ConstPtr& msg);
    void cpRcvCb(const std_msgs::UInt32::ConstPtr& msg);
    void npRcvCb(const std_msgs::UInt32::ConstPtr& msg);
    void wtRcvCb(const std_msgs::Bool::ConstPtr& msg);
    void targetIdCmdCb(const std_msgs::UInt32::ConstPtr& msg);
    void stopWaitingCmdCb(const std_msgs::Bool::ConstPtr& msg);

    // Robot 2 callbacks
    void thisCurrentPoseRobot2Cb(const std_msgs::UInt32::ConstPtr& msg);

    ros::NodeHandle& nh_;
    int robot_id_;

    ros::Publisher pub_planned_paths_;
    ros::Publisher pub_robot1_pose_;
    ros::Publisher pub_robot2_pose_;
    ros::Publisher pub_cp_send_;
    ros::Publisher pub_np_send_;
    ros::Publisher pub_wt_send_;
    ros::Publisher pub_target_id_send_;
    ros::Publisher pub_stop_waiting_send_;

    ros::Subscriber sub_r1_path_;
    ros::Subscriber sub_this_pose_;
    ros::Subscriber sub_cp_rcv_;
    ros::Subscriber sub_np_rcv_;
    ros::Subscriber sub_wt_rcv_;
    ros::Subscriber sub_target_id_cmd_;
    ros::Subscriber sub_stop_waiting_cmd_;

};
