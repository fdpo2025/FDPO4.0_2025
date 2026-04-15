#pragma once

#include <ros/ros.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

#include "localizer/Pose.h"
#include "localizer/Cluster.h"
#include "localizer/BeaconMatch.h" 
#include "general/pose.h"
#include "general/beacon.h"


class LocalizerNode {

    public:
        LocalizerNode(ros::NodeHandle& nh);

    private:
        ros::NodeHandle& nh;

        double dt;
        double v_e, w_e;

        ros::Time odom_stamp;
        ros::Time last_state_stamp_;

        Eigen::Matrix<double, 3, 1> X_state;
        Eigen::Matrix<double, 3, 3> P;
        Eigen::Matrix<double, 3, 3> grad_f_X;
        Eigen::Matrix<double, 3, 2> grad_f_U;
        Eigen::Matrix<double, 2, 2> Q;  

        ros::Subscriber odometry_sub;
        void ekf_predict(const nav_msgs::Odometry::ConstPtr& msg);

        std::unordered_map <std::string, Beacon> beacons;
        void loadBeaconsFromParams();
        void loadEKFParams();

        ros::Subscriber beacon_sub;
        void ekf_update(const localizer::BeaconMatch::ConstPtr& msg);

        double normalizeAngle(double theta);

        tf2_ros::TransformBroadcaster tf_broadcaster;
        tf2_ros::Buffer tf_buffer;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener;

        ros::Timer tf_timer_;
        void publishMapToOdomTF_(); 

        ros::Publisher pose_pub;
        void publishLogPose();

        int odom_skip_count_ = 0;
        int odom_skip_remaining_ = 0;
};


