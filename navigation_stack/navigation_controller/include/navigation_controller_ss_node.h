//
//  Navigation Controller - State Space Template
//  Template with basic ROS integration (subscriptions/publications)
//  State space control logic to be implemented
//

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>

// ============================================================================
// MAIN CLASS - NavigationControllerSS
// ============================================================================
// This class has only basic ROS infrastructure.
// ============================================================================

class NavigationControllerSS {

    public:
        NavigationControllerSS(ros::NodeHandle& nh_);

    private:
        ros::NodeHandle& nh;

        // ====================================================================
        // ROS INFRASTRUCTURE
        // ====================================================================
        ros::Subscriber odomSub;
        ros::Publisher velPub;
        ros::Timer controlTimer;

        // ====================================================================
        // CALLBACKS ROS
        // ====================================================================
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void controlLoop(const ros::TimerEvent&);

        // ====================================================================
        // DATA AVAILABLE FOR CONTROLLER
        // ====================================================================
        // Colleague can use these variables in control logic
        double curr_x, curr_y, curr_theta;  // Current robot pose
        double v_d, w_d;                    // Desired velocities (output)

        // ====================================================================
        // AREA FOR STATE SPACE LOGIC IMPLEMENTATION
        // ====================================================================
        // TODO: Add controller state variables
        // TODO: Implement control calculation function
        void computeStateSpaceControl();

};

