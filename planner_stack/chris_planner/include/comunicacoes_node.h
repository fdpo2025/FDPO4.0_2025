#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32MultiArray.h>

class ComunicacoesNode {
public:
    ComunicacoesNode(ros::NodeHandle& nh, int robot_id);

private:
    // Robot 1 callbacks
    void robot1PlannedPathCb(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void robot2PlannedPathCb(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void thisCurrentPoseRobot1Cb(const std_msgs::UInt32::ConstPtr& msg);
    void cpRcvCb(const std_msgs::UInt32::ConstPtr& msg);

    // Robot 2 callbacks
    void thisCurrentPoseRobot2Cb(const std_msgs::UInt32::ConstPtr& msg);
    void pathRcvCb(const std_msgs::Int32MultiArray::ConstPtr& msg);

    ros::NodeHandle& nh_;
    int robot_id_;

    ros::Publisher pub_planned_paths_;
    ros::Publisher pub_robot1_pose_;
    ros::Publisher pub_robot2_pose_;
    ros::Publisher pub_cp_send_;
    ros::Publisher pub_path_send_;

    ros::Subscriber sub_r1_path_;
    ros::Subscriber sub_r2_path_;
    ros::Subscriber sub_this_pose_;
    ros::Subscriber sub_cp_rcv_;
    ros::Subscriber sub_path_rcv_;

};
