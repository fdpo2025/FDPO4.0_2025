//
// Created by afonso on 04/03/26.
//

#include <motion_distortion_compensator/mdistortion_compensator_node.h>

Distortion_Compensator_Node::Distortion_Compensator_Node(ros::NodeHandle& nh_)
    : nh(nh_) {

    odometry_sub = nh.subscribe<nav_msgs::Odometry>(
        "/odom", 10,
        &Distortion_Compensator_Node::updateVelocities, this);

    cloud_sub = nh.subscribe<sensor_msgs::PointCloud>(
        "laser_scan_point_cloud", 1,
        &Distortion_Compensator_Node::getLaserScan, this);

    scan_freq_sub = nh.subscribe<std_msgs::Float32>(
        "laser_scan_frequency", 1,
        &Distortion_Compensator_Node::updateScanFrequency, this);

    cloud_pub = nh.advertise<sensor_msgs::PointCloud>(
        "laser_scan_point_cloud_compensated", 1, false);
}

void Distortion_Compensator_Node::updateVelocities(
    const nav_msgs::Odometry::ConstPtr& msg) {

    v = msg->twist.twist.linear.x;
    w = msg->twist.twist.angular.z;
}

void Distortion_Compensator_Node::updateScanFrequency(
    const std_msgs::Float32::ConstPtr& msg) {
    const double f = static_cast<double>(msg->data);  // Hz
    if (f > 0.0) {
        scan_freq_hz_ = f;
    }
}

void Distortion_Compensator_Node::getLaserScan(
    const sensor_msgs::PointCloud::ConstPtr& msg) {

    point_cloud_distorted = *msg;
    point_cloud_compensated = *msg;

    const std::size_t n_points = point_cloud_distorted.points.size();
    if (n_points == 0) {
        cloud_pub.publish(point_cloud_compensated);
        return;
    }
    // dt between two consecutive points: T_scan/N (equally spaced samples in one rotation)
    const double dt_point = (scan_freq_hz_ > 0.0)
        ? (1.0 / scan_freq_hz_) / static_cast<double>(n_points)
        : 0.0;
    if (dt_point <= 0.0) {
        cloud_pub.publish(*msg);
        return;
    }

    const double w_tol = 1e-6;

    for (std::size_t i = 0; i < n_points; ++i) {
        const double ti = dt_point * static_cast<double>(i);

        // assuming w and v static during one scan
        double theta_i;
        double x_i;
        double y_i;

        if (std::fabs(w) < w_tol) {
            // w~0: movement almost straight
            theta_i = 0.0;
            x_i = v * ti;
            y_i = 0.0;
        } else {
            theta_i = w * ti;
            x_i = (v / w) * std::sin(theta_i);
            y_i = (v / w) * (1.0 - std::cos(theta_i));
        }

        const double x_Li_i = point_cloud_distorted.points[i].x;
        const double y_Li_i = point_cloud_distorted.points[i].y;

        const double cos_th = std::cos(theta_i);
        const double sin_th = std::sin(theta_i);

        // Transform point from frame_ti to frame_0: P_0 = R(theta_i)*P_ti + T_i
        const double x_Lo_i = cos_th * x_Li_i - sin_th * y_Li_i + x_i;
        const double y_Lo_i = sin_th * x_Li_i + cos_th * y_Li_i + y_i;

        point_cloud_compensated.points[i].x = static_cast<float>(x_Lo_i);
        point_cloud_compensated.points[i].y = static_cast<float>(y_Lo_i);
    }

    cloud_pub.publish(point_cloud_compensated);
}