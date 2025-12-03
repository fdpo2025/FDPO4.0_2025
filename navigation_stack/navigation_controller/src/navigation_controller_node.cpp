#include "navigation_controller_node.h"
#include <cmath>

NavigationController::NavigationController(ros::NodeHandle& nh_) : nh(nh_), v_d(0.0), w_d(0.0), 
navigationFsm(navigation::states::idle), tfBuffer(), tfListener(tfBuffer), waypointReached(false) {
    
    mode = "idle";

    // load navigation parameters
    loadNavigationParams();
    
    //load RViz parameters
    nh.param("rviz_append", rvizGoalAppend, false);

    // ros init
    std::string odom_topic;
    nh.param("odom_topic", odom_topic, std::string("/odometry/filtered"));
    odomSub = nh.subscribe(odom_topic, 10, &NavigationController::updateCurrPose, this);
    ROS_INFO("NavigationController subscribing to odometry topic: %s", odom_topic.c_str());
    rvizGoalSub = nh.subscribe("/move_base_simple/goal", 10, &NavigationController::rvizGoalCallBack, this);
    velPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    controlTimer = nh.createTimer(ros::Duration(1.0 / std::max(1, param.loop_rate_hz)), &NavigationController::navigationFsmRunner, this);
    controlSrv = nh.advertiseService("control", &NavigationController::controlSrvCb, this);

    dynamic_reconfigure::Server<navigation_controller::NavigationConfig>::CallbackType cb;
    cb = boost::bind(&NavigationController::reconfigCb, this, _1, _2);
    dr_srv_.setCallback(cb);

    ROS_INFO("NavigationController instace created");
}

void NavigationController::reconfigCb(navigation_controller::NavigationConfig &cfg, uint32_t) {
    param.v_nom        = cfg.v_nom;
    param.w_nom        = cfg.w_nom;
    param.w_min        = cfg.w_min;
    param.kp_linear    = cfg.kp_linear;
    param.kp_angular   = cfg.kp_angular;
    param.arrive_radius= cfg.arrive_radius;
    param.yaw_tol      = cfg.yaw_tol;

    if (param.loop_rate_hz != cfg.loop_rate_hz) {
        param.loop_rate_hz = cfg.loop_rate_hz;
        controlTimer.stop();
        controlTimer = nh.createTimer(
            ros::Duration(1.0 / std::max(1, param.loop_rate_hz)),
            &NavigationController::navigationFsmRunner, this
        );
    }
}

void NavigationController::loadRouteFromParameters(){

    XmlRpc::XmlRpcValue waypoints;
    if(!nh.getParam("waypoints", waypoints)) return;

    route.clear();

    for(int i = 0; i < static_cast<int>(waypoints.size()); ++i){
        WayPoint waypoint_temp;

        waypoint_temp.pose.x = static_cast<double>(waypoints[i]["x"]);
        waypoint_temp.pose.y = static_cast<double>(waypoints[i]["y"]);
        waypoint_temp.pose.theta = static_cast<double>(waypoints[i]["yaw"]);
        waypoint_temp.align = static_cast<bool>(waypoints[i]["align"]);
        waypoint_temp.backwards = static_cast<bool>(waypoints[i]["backwards"]);
        ROS_INFO("Waypoint: x=%.2f y=%.2f yaw=%.2f", waypoint_temp.pose.x, waypoint_temp.pose.y, waypoint_temp.pose.theta);

        route.push_back(waypoint_temp);

    }

    updateDesiredPose();

}

void NavigationController::loadNavigationParams() {

    nh.param("v_nom", param.v_nom, 0.4);
    nh.param("w_nom", param.w_nom, 1.2);
    nh.param("w_min", param.w_min, 0.1);
    nh.param("v_min", param.v_min, 0.07);
    nh.param("a_max", param.a_max, 0.5);
    nh.param("d_max", param.d_max, 0.5);
    nh.param("kp_linear", param.kp_linear, 5.0);
    nh.param("kp_angular", param.kp_angular, 2.0/M_PI * param.w_nom);
    nh.param("arrive_radius",  param.arrive_radius, 0.05);
    nh.param("yaw_tol",param.yaw_tol, 0.08);
    nh.param("loop_rate_hz", param.loop_rate_hz, 30);

}

void NavigationController::updateDesiredPose() {

    if(route.empty()) return;

    // Work directly in map frame no transformations needed
    poseDesired = route.front().pose;
    ROS_INFO("New waypoint (map): x=%.2f y=%.2f yaw=%.2f", poseDesired.x, poseDesired.y, poseDesired.theta);

}

bool NavigationController::isBackwards() {

    return !route.empty() ? route.front().backwards : false;

}

double NavigationController::getAlignYawError() {

    double theta_d = std::atan2(poseDesired.y - poseCurr.y, poseDesired.x - poseCurr.x);
    
    // If backwards, robot must point in opposite direction to waypoint
    if (isBackwards()) {
        theta_d = normalizeAngle(theta_d + M_PI);
    }

    return normalizeAngle(theta_d - poseCurr.theta); 

}

bool NavigationController::checkAlignYaw() {

    double yaw_error = getAlignYawError();

    if(std::fabs(yaw_error) <= param.yaw_tol) return true;
    return false;

}

double NavigationController::getPositionError() {

    return std::hypot(poseDesired.x - poseCurr.x, poseDesired.y - poseCurr.y);

}

bool NavigationController::isPositionArrived() {
    
    double position_error = getPositionError();

    if(position_error <= param.arrive_radius) return true;
    return false;

}

double NavigationController::getDesiredYawError() {

    return normalizeAngle(poseDesired.theta - poseCurr.theta); 

}

bool NavigationController::isYawDesired() {

    double yaw_error = getDesiredYawError();

    if(std::fabs(yaw_error) <= param.yaw_tol) return true;
    return false;

}

void NavigationController::updateCurrPose(const nav_msgs::Odometry::ConstPtr& msg) {

    poseCurr.x = msg->pose.pose.position.x;
    poseCurr.y = msg->pose.pose.position.y;
    poseCurr.theta = tf2::getYaw(msg->pose.pose.orientation);

}

void NavigationController::rvizGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    // Ensure goal is in map frame
    geometry_msgs::PoseStamped poseInMap;
    if (msg->header.frame_id != "map") {
        try {
            poseInMap = tfBuffer.transform(*msg, "map", ros::Duration(1.0));
            ROS_INFO("Transformed RViz goal from %s to map frame", msg->header.frame_id.c_str());
        } catch (const tf2::TransformException& ex) {
            ROS_ERROR("Failed to transform goal from %s to map: %s", msg->header.frame_id.c_str(), ex.what());
            return;
        }
    } else {
        poseInMap = *msg;
    }

    if(!rvizGoalAppend) route.clear();

    WayPoint waypoint_temp;

    waypoint_temp.id = route.empty() ? 0 : (route.back().id + 1);    
    waypoint_temp.pose.x = poseInMap.pose.position.x;
    waypoint_temp.pose.y = poseInMap.pose.position.y;
    waypoint_temp.pose.theta = tf2::getYaw(poseInMap.pose.orientation);
    waypoint_temp.align = true;
    waypoint_temp.backwards = false;

    route.push_back(waypoint_temp);
    ROS_INFO("RViz goal added: x=%.2f y=%.2f yaw=%.2f (map frame)", 
             waypoint_temp.pose.x, waypoint_temp.pose.y, waypoint_temp.pose.theta);

    updateDesiredPose();

}

void NavigationController::publishVel() {

    // Only publish if there is real movement (not zeros)
    // This avoids constantly publishing when idle
    if (std::abs(v_d) < 1e-6 && std::abs(w_d) < 1e-6) {
        return;  // Don't publish zeros when stopped
    }

    geometry_msgs::Twist cmd;
    cmd.linear.x  = v_d;
    cmd.linear.y  = 0.0;
    cmd.linear.z  = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = w_d;
    velPub.publish(cmd);

}

double NavigationController::normalizeAngle(double theta) {

    while(theta > M_PI) theta -= 2.0 * M_PI;
    while (theta <= -M_PI) theta += 2.0*M_PI;

    return theta;
}

void NavigationController::hardStop() {

    w_d = v_d = 0.0;
    // Force publish zeros to stop robot immediately
    geometry_msgs::Twist cmd;
    cmd.linear.x  = 0.0;
    cmd.linear.y  = 0.0;
    cmd.linear.z  = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;
    velPub.publish(cmd);

}

void NavigationController::setTheta() {

    v_d = 0.0;

    double yaw_error = getDesiredYawError(); 
    if(std::fabs(yaw_error) <= param.yaw_tol) {

        w_d = 0.0;

        return;
    }

    w_d = param.kp_angular * yaw_error;

    if(w_d > param.w_nom) w_d = param.w_nom;
    else if(w_d < -param.w_nom) w_d = -param.w_nom;

}

void NavigationController::goToXY() {

    double position_error = getPositionError();
    double yaw_error = getAlignYawError();
    double dt = 1.0 / param.loop_rate_hz;

    // Stop if the robot reached the goal position
    if (position_error <= param.arrive_radius) {
        v_d = 0.0;
        w_d = 0.0;
        return;
    }

    // -----------------------
    //     ANGULAR CONTROL
    // -----------------------
    w_d = param.kp_angular * yaw_error;
    if (w_d > param.w_nom) w_d = param.w_nom;
    else if (w_d < -param.w_nom) w_d = -param.w_nom;

    // -----------------------
    //     LINEAR CONTROL
    // -----------------------
    double v_mag = 0.0;

    // Keep the original condition
    if (std::fabs(yaw_error) <= M_PI / 6.0) {
        v_mag = param.v_nom
                * std::cos(yaw_error)
                * std::min<double>(1.0, param.kp_linear * position_error);
    }

    // -----------------------
    //   MINIMUM SPEED ON TARGET  
    // -----------------------
    double v_target = v_mag;

    if (v_target > 0.0 && v_target < param.v_min)
        v_target = param.v_min;

    // -----------------------
    //   APPLY ACCEL/DECEL LIMIT
    // -----------------------
    if (v_target > v_d) {
        // Accelerate
        v_d += param.a_max * dt;
        if (v_d > v_target) v_d = v_target;
    } else {
        // Decelerate
        v_d -= param.d_max * dt;
        if (v_d < v_target) v_d = v_target;
    }

    // -----------------------
    //   BACKWARDS SUPPORT
    // -----------------------
    if (isBackwards())
        v_d = -v_d;
}



void NavigationController::navigationFsmRunner(const ros::TimerEvent&) {

    // Update's
    navigationFsm.update_tis();
    bool enable = !(mode == "stop" || mode == "pause") && !route.empty();

    // Compute Transitions
    if(navigationFsm.state == navigation::states::idle && enable) {

        waypointReached = false;  // Reset flag when starting new waypoint
        navigationFsm.new_state = navigation::states::driveToGoal;

    }

    else if(navigationFsm.state == navigation::states::driveToGoal && isPositionArrived() && route.front().align && enable) {

        waypointReached = true;  // Mark waypoint as reached
        navigationFsm.new_state = navigation::states::turnToFinalYaw;

    }

    else if(navigationFsm.state == navigation::states::driveToGoal && isPositionArrived() && !route.front().align && enable) {

        waypointReached = true;  // Mark waypoint as reached
        route.pop_front();
        updateDesiredPose();
        
        // If route is now empty, lock current position to prevent corrections
        if (route.empty()) {
            poseDesired = poseCurr;
        }
        
        navigationFsm.new_state = navigation::states::done;

    }

    else if (navigationFsm.state == navigation::states::turnToFinalYaw && enable && !waypointReached && !isPositionArrived()) {

        // Only go back to driveToGoal if waypoint was never reached
        navigationFsm.new_state = navigation::states::driveToGoal;

    }

    else if(navigationFsm.state == navigation::states::turnToFinalYaw && isYawDesired() && enable) {

        route.pop_front();
        updateDesiredPose();
        
        // If route is now empty, lock current position to prevent corrections
        if (route.empty()) {
            poseDesired = poseCurr;
        }
        
        waypointReached = false;  // Reset for next waypoint
        navigationFsm.new_state = navigation::states::done;

    }

    else if(navigationFsm.state == navigation::states::done && enable) {

        waypointReached = false;  // Reset flag when starting new waypoint
        navigationFsm.new_state = navigation::states::driveToGoal;

    }

    else if(navigationFsm.state == navigation::states::done && !enable) {

        // Lock current position to prevent EKF-induced corrections
        poseDesired = poseCurr;
        navigationFsm.new_state = navigation::states::idle;

    }

    // Set states
    navigationFsm.set_state();

    // Compute Actions
    if(navigationFsm.state == navigation::states::driveToGoal) goToXY();
    else if(navigationFsm.state == navigation::states::turnToFinalYaw && enable) setTheta();
    else {
        v_d = w_d = 0.0;
        // When idle or done, continuously update desired pose to current to prevent EKF corrections
        if (route.empty()) {
            poseDesired = poseCurr;
        }
    }

    // Affect outputs
    publishVel();

}

bool NavigationController::controlSrvCb(navigation_controller::NavigationControl::Request& req, navigation_controller::NavigationControl::Response& res) {

    mode = req.command;

    if(mode == "start") {

        loadRouteFromParameters();

        if (route.empty()) {
            res.success = false; res.message = "no waypoints in params";
            return true;
        }

        waypointReached = false;  // Reset flag on start
        res.success = true;  res.message = "started";
        ROS_INFO("Navigation START");
        return true;

    }

    else if(mode == "stop") {

        route.clear();
        navigationFsm.new_state = navigation::states::idle;
        poseDesired = poseCurr;
        waypointReached = false;  // Reset flag on stop
        navigationFsm.set_state();

        res.success = true; res.message = "stopped+cleared";
        ROS_INFO("Navigation STOP");
        return true;

    }

    else if(mode == "pause") {

        navigationFsm.new_state = navigation::states::idle;
        navigationFsm.set_state();

        res.success = true; res.message = "paused";
        ROS_INFO("Navigation PAUSE");
        return true;

    }

    else if(mode == "unpause") {

        res.success = true; res.message = "unpaused";
        ROS_INFO("Navigation UNPAUSE");
        return true;

    }
    
    return false;

}
