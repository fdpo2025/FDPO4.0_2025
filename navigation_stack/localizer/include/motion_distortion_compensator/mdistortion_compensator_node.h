//
// Created by afonso on 04/03/26.
//

#ifndef FDPO_ROS_STACK_MDISTORTION_COMPENSATOR_NODE_H
#define FDPO_ROS_STACK_MDISTORTION_COMPENSATOR_NODE_H

#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

class Distortion_Compensator_Node {

public:
    explicit Distortion_Compensator_Node(ros::NodeHandle& nh);

private:
    ros::NodeHandle& nh;

    // Scan frequency (Hz) from driver; used to compute dt between consecutive points
    double scan_freq_hz_ {0.0};

    // Linear and angular velocities coming from odometry
    double v {0.0};
    double w {0.0};

    ros::Subscriber cloud_sub;
    sensor_msgs::PointCloud point_cloud_distorted;
    void getLaserScan(const sensor_msgs::PointCloud::ConstPtr& msg);

    ros::Subscriber scan_freq_sub;
    void updateScanFrequency(const std_msgs::Float32::ConstPtr& msg);

    ros::Subscriber odometry_sub;
    void updateVelocities(const nav_msgs::Odometry::ConstPtr& msg);

    ros::Publisher cloud_pub;
    sensor_msgs::PointCloud point_cloud_compensated;
};

// Subscribed topic:  "laser_scan_point_cloud"
// Published topic:    "laser_scan_point_cloud_compensated"

#endif //FDPO_ROS_STACK_MDISTORTION_COMPENSATOR_NODE_H