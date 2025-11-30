#include "navigation_controller_ss_node.h"
#include <cmath>

// ============================================================================
// CONSTRUCTOR
// ============================================================================
NavigationControllerSS::NavigationControllerSS(ros::NodeHandle& nh_) 
    : nh(nh_), curr_x(0.0), curr_y(0.0), curr_theta(0.0), v_d(0.0), w_d(0.0) {
    
    // ========================================================================
    // SUBSCRIBE TO ODOMETRY TOPIC
    // ========================================================================
    std::string odom_topic;
    nh.param("odom_topic", odom_topic, std::string("/odometry/filtered"));
    odomSub = nh.subscribe(odom_topic, 10, &NavigationControllerSS::odomCallback, this);
    ROS_INFO("NavigationControllerSS subscribing to: %s", odom_topic.c_str());

    // ========================================================================
    // PUBLISH TO VELOCITY TOPIC
    // ========================================================================
    velPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ROS_INFO("NavigationControllerSS publishing to: /cmd_vel");

    // ========================================================================
    // TIMER FOR CONTROL LOOP
    // ========================================================================
    int loop_rate_hz;
    nh.param("loop_rate_hz", loop_rate_hz, 30);
    controlTimer = nh.createTimer(ros::Duration(1.0 / loop_rate_hz), 
                                   &NavigationControllerSS::controlLoop, this);

    ROS_INFO("NavigationControllerSS initialized");
}

// ============================================================================
// ODOMETRY CALLBACK
// ============================================================================
void NavigationControllerSS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    curr_x = msg->pose.pose.position.x;
    curr_y = msg->pose.pose.position.y;
    curr_theta = tf2::getYaw(msg->pose.pose.orientation);
}

// ============================================================================
// CONTROL LOOP
// ============================================================================
void NavigationControllerSS::controlLoop(const ros::TimerEvent&) {
    
    // ========================================================================
    // CALL STATE SPACE CONTROLLER
    // ========================================================================
    // This function should calculate v_d and w_d based on current state
    computeStateSpaceControl();
    
    // ========================================================================
    // PUBLISH VELOCITY
    // ========================================================================
    geometry_msgs::Twist cmd;
    cmd.linear.x  = v_d;
    cmd.linear.y  = 0.0;
    cmd.linear.z  = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = w_d;
    velPub.publish(cmd);
}

// ============================================================================
// ============================================================================
// AREA FOR STATE SPACE LOGIC IMPLEMENTATION
// ============================================================================
// ============================================================================
void NavigationControllerSS::computeStateSpaceControl() {
    
    // ========================================================================
    // TODO: IMPLEMENT STATE SPACE CONTROL LOGIC
    // ========================================================================
    // 
    // AVAILABLE INPUTS:
    // - curr_x, curr_y, curr_theta: Current robot pose
    // 
    // EXPECTED OUTPUTS:
    // - v_d: Desired linear velocity (m/s)
    // - w_d: Desired angular velocity (rad/s)
    //
    // Colleague should implement here the state space control logic.
    // For example:
    // - Define system state (x, y, theta, or others)
    // - Calculate state error
    // - Apply control law (e.g., u = -K * (x - x_d))
    // - Convert to v_d and w_d
    //
    
    // Placeholder example (REPLACE):
    v_d = 0.0;
    w_d = 0.0;
}

