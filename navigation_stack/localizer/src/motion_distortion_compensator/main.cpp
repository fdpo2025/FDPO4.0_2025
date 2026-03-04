#include <ros/ros.h>

#include <motion_distortion_compensator/mdistortion_compensator_node.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "mdistortion_compensator_node");

    ros::NodeHandle nh;
    Distortion_Compensator_Node node(nh);

    ros::spin();
    return 0;
}

//
// Created by afonso on 04/03/26.
//