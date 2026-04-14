#include "navigation_controller_node.h"
#include <cmath>

NavigationController::NavigationController(ros::NodeHandle& nh_) : nh(nh_), v_d(0.0), w_d(0.0),
navigationFsm(navigation::states::idle), followLineFsm(navigation::followLineStates::Follow_Line),
k1(0.0), previousWaypoint({-1, {0, 0, 0}, false, false, -1.0, -1.0, false, false, false, -1}), tfBuffer(), tfListener(tfBuffer),
in_pick_box_forward(false), last_vel_before_approaching_(0.0), approaching_brake_ref_dist_(0.1),
    process_warehouse_goto_align_done_(false), process_warehouse_goto_start_dist_(0.0),
    process_warehouse_goto_completion_sent_(false), drop_magnet_wiggle_start_yaw_(0.0),
    drop_magnet_wiggle_target_rad_(0.0), drop_pick_box_release_published_for_node_id_(-1) {

    mode = "idle";

    // load navigation parameters
    loadNavigationParams();
    last_vel_before_approaching_ = param.vel_lin_nom;
    
    //load RViz parameters
    nh.param("rviz_append", rvizGoalAppend, false);

    // load from YAML or from /nav_plan
    nh.param("load_from_route", load_from_route, false);
    
    // ros init
    std::string odom_topic;
    nh.param("odom_topic", odom_topic, std::string("/odometry/filtered"));
    odomSub = nh.subscribe(odom_topic, 10, &NavigationController::updateCurrPose, this);
    ROS_INFO("NavigationController subscribing to odometry topic: %s", odom_topic.c_str());
    rvizGoalSub = nh.subscribe("/move_base_simple/goal", 10, &NavigationController::rvizGoalCallBack, this);
    navPlanSub = nh.subscribe("/nav_plan", 10, &NavigationController::navPlanCallback, this);
    ROS_INFO("NavigationController subscribing to /nav_plan (load_from_route=%s)", load_from_route ? "true" : "false");
    velPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    lineMarkerPub = nh.advertise<visualization_msgs::Marker>("navigation_lines", 1, true);  // latch=true para RViz ver imediatamente
    virtualLineMarkerPub = nh.advertise<visualization_msgs::Marker>("navigation_virtual_line", 1, true);
    navCompletionFeedbackPub = nh.advertise<plan_handler::CompletionFeedback>("/nav_completion_feedback", 10);
    pickBoxPub = nh.advertise<std_msgs::Bool>("/pick_box", 10);

    // =========================
    // PUBLICAR NÓ ATUAL
    // =========================
    currentNodePub = nh.advertise<std_msgs::UInt32>("/this_current_pose", 10, true);

    completion_feedback_sent = false;
    hasPendingNavPlan = false;
    last_published_node_id = -1;

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
    drop_pick_box_release_published_for_node_id_ = -1;
    
    // Inicializar previousWaypoint com posição atual do robô
    // Isto define o ponto inicial da primeira linha
    previousWaypoint.id = 0;
    previousWaypoint.pose.x = poseCurr.x;
    previousWaypoint.pose.y = poseCurr.y;
    previousWaypoint.pose.theta = poseCurr.theta;
    previousWaypoint.align = false;
    previousWaypoint.backwards = false;
    previousWaypoint.line_switch_ratio = -1.0;
    previousWaypoint.vel_lin_nom = -1.0;
    previousWaypoint.pick_box = false;  // route.yaml não tem pick_box, usar false
    previousWaypoint.is_warehouse = false;
    previousWaypoint.is_process_warehouse = false;
    previousWaypoint.node_id = -1;
    ROS_INFO("Initial position set as previousWaypoint: x=%.2f y=%.2f", poseCurr.x, poseCurr.y);

    for(int i = 0; i < static_cast<int>(waypoints.size()); ++i){
        WayPoint waypoint_temp;
        waypoint_temp.id = i;

        waypoint_temp.pose.x = static_cast<double>(waypoints[i]["x"]);
        waypoint_temp.pose.y = static_cast<double>(waypoints[i]["y"]);
        waypoint_temp.pose.theta = static_cast<double>(waypoints[i]["yaw"]);
        waypoint_temp.align = static_cast<bool>(waypoints[i]["align"]);
        waypoint_temp.backwards = static_cast<bool>(waypoints[i]["backwards"]);
        waypoint_temp.pick_box = false;  // route.yaml não tem pick_box, usar false
        waypoint_temp.is_warehouse = false;  // route.yaml não tem is_warehouse, usar false
        waypoint_temp.is_process_warehouse = false;
        waypoint_temp.node_id = -1;  // route.yaml não tem node_id
        
        // line_switch_ratio: se não definido, usar -1 (significa usar parâmetro global)
        if (waypoints[i].hasMember("line_switch_ratio")) {
            waypoint_temp.line_switch_ratio = static_cast<double>(waypoints[i]["line_switch_ratio"]);
        } else {
            waypoint_temp.line_switch_ratio = -1.0;  // Usar parâmetro global
        }
        
        // vel_lin_nom: se não definido, usar -1 (significa usar parâmetro global)
        if (waypoints[i].hasMember("vel_lin_nom")) {
            waypoint_temp.vel_lin_nom = static_cast<double>(waypoints[i]["vel_lin_nom"]);
        } else {
            waypoint_temp.vel_lin_nom = -1.0;  // Usar parâmetro global
        }
        
        double effective_vel = waypoint_temp.vel_lin_nom > 0 ? waypoint_temp.vel_lin_nom : param.vel_lin_nom;
        ROS_INFO("Waypoint %d: x=%.2f y=%.2f yaw=%.2f backwards=%d switch=%.0f%% vel=%.2f", 
                 waypoint_temp.id, waypoint_temp.pose.x, waypoint_temp.pose.y, waypoint_temp.pose.theta,
                 waypoint_temp.backwards, 
                 waypoint_temp.line_switch_ratio > 0 ? waypoint_temp.line_switch_ratio * 100 : param.line_switch_ratio * 100,
                 effective_vel);

        route.push_back(waypoint_temp);

    }

    updateDesiredPose();
    
    if (!route.empty()) {
        followLineFsm.new_state = navigation::followLineStates::Follow_Line;
        ROS_WARN("followline 1");
        followLineFsm.set_state();
        skipNearbyWaypoints();
    }
    
    completion_feedback_sent = false;
    publishLineMarkers();

}

void NavigationController::loadNavigationParams() {

    nh.param("v_nom", param.v_nom, 0.4);
    nh.param("w_nom", param.w_nom, 1.2);
    nh.param("w_min", param.w_min, 0.1);
    nh.param("v_min", param.v_min, 0.07);
    nh.param("v_max", param.v_max, 0.5);
    nh.param("a_max", param.a_max, 0.5);
    nh.param("d_max", param.d_max, 0.5);
    nh.param("kp_linear", param.kp_linear, 5.0);
    nh.param("kp_angular", param.kp_angular, 2.0/M_PI * param.w_nom);
    nh.param("arrive_radius",  param.arrive_radius, 0.05);
    nh.param("yaw_tol",param.yaw_tol, 0.08);
    nh.param("loop_rate_hz", param.loop_rate_hz, 30);

    // Follow line: /follow_line/navigation_controller (follow_line_parameters.yaml)
    ros::NodeHandle nh_fl("follow_line/navigation_controller");
    nh_fl.param("k_line", param.k_line, 1.0);
    nh_fl.param("gain_fwd", param.gain_fwd, 1.0);
    nh_fl.param("vel_lin_nom", param.vel_lin_nom, 0.3);
    nh_fl.param("dist_da", param.dist_da, 0.3);
    nh_fl.param("tol_findist", param.tol_findist, 0.05);
    nh_fl.param("max_etf", param.max_etf, 0.2);
    nh_fl.param("tol_init_line", param.tol_init_line, 0.1);
    nh_fl.param("line_switch_ratio", param.line_switch_ratio, 0.9);
    nh_fl.param("use_stanley_follow_line", param.use_stanley_follow_line, false);
    nh_fl.param("use_stanley_approaching", param.use_stanley_approaching, false);
    nh_fl.param("stanley_k", param.stanley_k, 1.0);
    nh_fl.param("stanley_soft_v", param.stanley_soft_v, 0.05);
    nh_fl.param("stanley_eps", param.stanley_eps, 0.0);
    nh_fl.param("approaching_enter_dist_m", param.approaching_enter_dist_m, 0.10);
    {
        const double bearing_default = std::max(param.yaw_tol * 1.5, 0.10);
        nh_fl.param("bearing_align_yaw_tol", param.bearing_align_yaw_tol, bearing_default);
    }
    ROS_INFO("NavigationController: bearing_align_yaw_tol=%.4f rad (process warehouse go-to, fase rodação em sitio)",
             param.bearing_align_yaw_tol);
    nh_fl.param("k_approaching", param.k_approaching, 10.0);
    nh_fl.param("gain_approaching_fwd", param.gain_approaching_fwd, 2.0);
    nh_fl.param("approaching_vel_normal", param.approaching_vel_normal, param.v_min);

    double approaching_vel_legacy = 0.085;
    nh_fl.param("approaching_vel", approaching_vel_legacy, approaching_vel_legacy);
    nh_fl.param("k_approaching_pickdrop", param.k_approaching_pickdrop, param.k_approaching);
    nh_fl.param("gain_approaching_fwd_pickdrop", param.gain_approaching_fwd_pickdrop, param.gain_approaching_fwd);
    nh_fl.param("approaching_vel_pickdrop", param.approaching_vel_pickdrop, approaching_vel_legacy);

    nh_fl.param("approaching_colinear_angle_rad", param.approaching_colinear_angle_rad, 0.087);
    nh_fl.param("k_approaching_process", param.k_approaching_process, param.k_approaching_pickdrop);
    nh_fl.param("gain_approaching_fwd_process", param.gain_approaching_fwd_process, param.gain_approaching_fwd_pickdrop);
    nh_fl.param("approaching_vel_process", param.approaching_vel_process, param.approaching_vel_pickdrop);
    nh_fl.param("pick_box_forward_vel", param.pick_box_forward_vel, 0.1);
    nh_fl.param("drop_magnet_wiggle_angle_deg", param.drop_magnet_wiggle_angle_deg, 25.0);
    nh_fl.param("drop_magnet_wiggle_angular_vel", param.drop_magnet_wiggle_angular_vel, 0.8);
    nh_fl.param("drop_pick_box_release_distance", param.drop_pick_box_release_distance, 0.03);

    ROS_INFO("NavigationController parameters loaded: v_nom=%.2f, w_nom=%.2f, k_line=%.2f, line_switch_ratio=%.2f (follow_line from /follow_line/navigation_controller)", 
             param.v_nom, param.w_nom, param.k_line, param.line_switch_ratio);

}

void NavigationController::transitionAfterDropWarehouse(const char* trace_tag) {
    route.pop_front();
    updateDesiredPose();
    followLineFsm.new_state = navigation::followLineStates::Follow_Line;
    if (trace_tag) ROS_WARN("%s", trace_tag);
    followLineFsm.set_state();
    completion_feedback_sent = false;
    if (route.empty()) {
        navigationFsm.new_state = navigation::states::idle;
    } else if (route.front().backwards && param.drop_magnet_wiggle_angle_deg > 1e-6) {
        drop_magnet_wiggle_start_yaw_ = poseCurr.theta;
        drop_magnet_wiggle_target_rad_ = param.drop_magnet_wiggle_angle_deg * M_PI / 180.0;
        navigationFsm.new_state = navigation::states::dropMagnetReleaseWiggle;
        ROS_INFO("NavigationController: Post-drop magnet wiggle: start_yaw=%.3f rad, target=%.2f deg, w=%.3f rad/s (before backwards segment)",
                 drop_magnet_wiggle_start_yaw_, param.drop_magnet_wiggle_angle_deg, param.drop_magnet_wiggle_angular_vel);
    } else {
        navigationFsm.new_state = navigation::states::done;
    }
}

void NavigationController::updateDesiredPose() {

    if(route.empty()) return;

    poseDesired = route.front().pose;
    ROS_INFO("New waypoint (map): x=%.2f y=%.2f yaw=%.2f", poseDesired.x, poseDesired.y, poseDesired.theta);

}

bool NavigationController::isBackwards() {

    return !route.empty() ? route.front().backwards : false;

}

double NavigationController::getAlignYawError() {

    double theta_d = std::atan2(poseDesired.y - poseCurr.y, poseDesired.x - poseCurr.x);
    
    if (isBackwards()) {
        theta_d = normalizeAngle(theta_d + M_PI);
    }

    return normalizeAngle(theta_d - poseCurr.theta); 

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

void NavigationController::dist2Line(double xi, double yi, double xf, double yf, double xr, double yr, double& distLine) {
    double ux, uy;
    
    double dx = xf - xi;
    double dy = yf - yi;
    double line_length = std::sqrt(dx * dx + dy * dy);
    
    if (line_length < 1e-6) {
        distLine = std::sqrt((xr - xi) * (xr - xi) + (yr - yi) * (yr - yi));
        k1 = 0.0;
        line_progress = 1.0;
        return;
    }
    
    ux = dx / line_length;
    uy = dy / line_length;
    
    k1 = (xr * uy - yr * ux - xi * uy + yi * ux) / (ux * ux + uy * uy);
    distLine = std::abs(k1);
    
    double t = ((xr - xi) * ux + (yr - yi) * uy);
    line_progress = t / line_length;
}

void NavigationController::dist2LineVirtual(double xi, double yi, double xf, double yf, double xr, double yr,
                                            double& k1_virtual, double& error_ang_virtual) {
    double dx = xf - xi;
    double dy = yf - yi;
    double line_length = std::sqrt(dx * dx + dy * dy);

    if (line_length < 1e-6) {
        k1_virtual = std::sqrt((xr - xi) * (xr - xi) + (yr - yi) * (yr - yi));
        error_ang_virtual = 0.0;
        return;
    }

    double ux = dx / line_length;
    double uy = dy / line_length;
    k1_virtual = (xr * uy - yr * ux - xi * uy + yi * ux);
    double line_angle = std::atan2(dy, dx);
    if (isBackwards()) {
        line_angle = normalizeAngle(line_angle + M_PI);
    }
    error_ang_virtual = normalizeAngle(line_angle - poseCurr.theta);
}

double NavigationController::getLineAngle(double pi_x, double pi_y, double pf_x, double pf_y) {
    double dx = pf_x - pi_x;
    double dy = pf_y - pi_y;
    return std::atan2(dy, dx);
}

double NavigationController::getLineError() {
    
    if (route.empty()) return 0.0;
    
    if (route.size() < 2) {
        return getPositionError();
    }
    
    auto it = route.begin();
    double pi_x = it->pose.x;
    double pi_y = it->pose.y;
    
    ++it;
    double pf_x = it->pose.x;
    double pf_y = it->pose.y;
    
    double distLine;
    dist2Line(pi_x, pi_y, pf_x, pf_y, poseCurr.x, poseCurr.y, distLine);
    return distLine;
}

double NavigationController::getAlignLineYawError() {
    
    if (route.empty()) return 0.0;
    
    if (route.size() < 2) {
        return getAlignYawError();
    }
    
    auto it = route.begin();
    double pi_x = it->pose.x;
    double pi_y = it->pose.y;
    
    ++it;
    double pf_x = it->pose.x;
    double pf_y = it->pose.y;
    
    double line_angle = getLineAngle(pi_x, pi_y, pf_x, pf_y);
    
    if (isBackwards()) {
        line_angle = normalizeAngle(line_angle + M_PI);
    }
    
    return normalizeAngle(line_angle - poseCurr.theta);
}

void NavigationController::publishLineMarkers() {
    
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "navigation_lines";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::Marker::LINE_LIST;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.05;
    line_marker.color.a = 1.0;
    line_marker.lifetime = ros::Duration(0);
    line_marker.points.clear();
    line_marker.colors.clear();
    
    if (route.empty() || route.size() < 2) {
        lineMarkerPub.publish(line_marker);
        return;
    }
    
    for (auto it = route.begin(); it != route.end(); ++it) {
        auto next_it = std::next(it);
        if (next_it == route.end()) break;
        
        geometry_msgs::Point p1;
        p1.x = it->pose.x;
        p1.y = it->pose.y;
        p1.z = 0.0;
        
        geometry_msgs::Point p2;
        p2.x = next_it->pose.x;
        p2.y = next_it->pose.y;
        p2.z = 0.0;
        
        std_msgs::ColorRGBA line_color;
        if (next_it->backwards) {
            line_color.r = 0.0;
            line_color.g = 0.5;
            line_color.b = 1.0;
            line_color.a = 1.0;
        } else {
            line_color.r = 0.0;
            line_color.g = 1.0;
            line_color.b = 0.0;
            line_color.a = 1.0;
        }
        
        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);
        line_marker.colors.push_back(line_color);
        line_marker.colors.push_back(line_color);
    }
    
    lineMarkerPub.publish(line_marker);
}

void NavigationController::publishVirtualLineMarker(double pi_x, double pi_y, double pf_x, double pf_y) {
    double dx = pf_x - pi_x;
    double dy = pf_y - pi_y;
    double line_length = std::sqrt(dx * dx + dy * dy);

    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.ns = "navigation_virtual_line";
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.08;
    m.color.r = 1.0;
    m.color.g = 0.5;
    m.color.b = 0.0;
    m.color.a = 1.0;
    m.lifetime = ros::Duration(0.2);

    const double ext = 50.0;
    geometry_msgs::Point p1, p2;
    if (line_length < 1e-6) {
        p1.x = pi_x; p1.y = pi_y; p1.z = 0.0;
        p2.x = pf_x; p2.y = pf_y; p2.z = 0.0;
    } else {
        double ux = dx / line_length;
        double uy = dy / line_length;
        p1.x = pi_x - ext * ux; p1.y = pi_y - ext * uy; p1.z = 0.0;
        p2.x = pf_x + ext * ux; p2.y = pf_y + ext * uy; p2.z = 0.0;
    }
    m.points.push_back(p1);
    m.points.push_back(p2);

    virtualLineMarkerPub.publish(m);
}

void NavigationController::skipNearbyWaypoints() {
    
    while (route.size() >= 2 && !route.front().align) {
        double pi_x = previousWaypoint.pose.x;
        double pi_y = previousWaypoint.pose.y;
        double pf_x = route.front().pose.x;
        double pf_y = route.front().pose.y;
        
        double line_length = std::sqrt((pf_x - pi_x) * (pf_x - pi_x) + (pf_y - pi_y) * (pf_y - pi_y));
        
        if (line_length < 0.01) {
            previousWaypoint = route.front();

            // =========================
            // PUBLICAR NÓ ATUAL
            // =========================
            publishCurrentNode(previousWaypoint.node_id);

            route.pop_front();
            ROS_INFO("Skipped waypoint (line too short): id=%d", previousWaypoint.id);
            continue;
        }
        
        double line_dx = (pf_x - pi_x) / line_length;
        double line_dy = (pf_y - pi_y) / line_length;
        
        double robot_dx = poseCurr.x - pi_x;
        double robot_dy = poseCurr.y - pi_y;
        
        double projection = robot_dx * line_dx + robot_dy * line_dy;
        double progress = projection / line_length;
        
        double switch_ratio = (route.front().line_switch_ratio > 0) ? 
                               route.front().line_switch_ratio : param.line_switch_ratio;
        
        if (progress >= switch_ratio) {
            previousWaypoint = route.front();

            // =========================
            // PUBLICAR NÓ ATUAL
            // =========================
            publishCurrentNode(previousWaypoint.node_id);

            route.pop_front();
            ROS_INFO("Skipped nearby waypoint: id=%d (progress=%.0f%% >= threshold=%.0f%%)", 
                     previousWaypoint.id, progress * 100, switch_ratio * 100);
        } else {
            break;
        }
    }
    
    updateDesiredPose();
}

void NavigationController::updateCurrPose(const nav_msgs::Odometry::ConstPtr& msg) {

    poseCurr.x = msg->pose.pose.position.x;
    poseCurr.y = msg->pose.pose.position.y;
    poseCurr.theta = tf2::getYaw(msg->pose.pose.orientation);

}

void NavigationController::rvizGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {

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

    if(!rvizGoalAppend) {
        route.clear();
        drop_pick_box_release_published_for_node_id_ = -1;
        previousWaypoint.id = 0;
        previousWaypoint.pose.x = poseCurr.x;
        previousWaypoint.pose.y = poseCurr.y;
        previousWaypoint.pose.theta = poseCurr.theta;
        previousWaypoint.align = false;
        previousWaypoint.backwards = false;
        previousWaypoint.line_switch_ratio = -1.0;
        previousWaypoint.vel_lin_nom = -1.0;
        previousWaypoint.is_warehouse = false;
        previousWaypoint.is_process_warehouse = false;
        previousWaypoint.node_id = -1;
        ROS_INFO("Initial position set as previousWaypoint: x=%.2f y=%.2f", poseCurr.x, poseCurr.y);
    }

    WayPoint waypoint_temp;

    waypoint_temp.id = route.empty() ? 1 : (route.back().id + 1);
    waypoint_temp.pose.x = poseInMap.pose.position.x;
    waypoint_temp.pose.y = poseInMap.pose.position.y;
    waypoint_temp.pose.theta = tf2::getYaw(poseInMap.pose.orientation);
    waypoint_temp.align = true;
    waypoint_temp.backwards = false;
    waypoint_temp.line_switch_ratio = -1.0;
    waypoint_temp.vel_lin_nom = -1.0;
    waypoint_temp.pick_box = false;
    waypoint_temp.is_warehouse = false;
    waypoint_temp.is_process_warehouse = false;
    waypoint_temp.node_id = -1; // goal RViz não tem node_id

    route.push_back(waypoint_temp);
    ROS_INFO("RViz goal added: x=%.2f y=%.2f yaw=%.2f (map frame)", 
             waypoint_temp.pose.x, waypoint_temp.pose.y, waypoint_temp.pose.theta);

    updateDesiredPose();
    
    followLineFsm.new_state = navigation::followLineStates::Follow_Line;
    ROS_WARN("followline 2");
    followLineFsm.set_state();
    
    publishLineMarkers();

}

void NavigationController::publishVel() {

    if (std::abs(v_d) < 1e-6 && std::abs(w_d) < 1e-6) {
        return;
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

    if(route.empty()) {
        w_d = 0.0;
        return;
    }

    double yaw_error = getDesiredYawError(); 
    if(std::fabs(yaw_error) <= param.yaw_tol) {
        w_d = 0.0;
        return;
    }

    w_d = param.kp_angular * yaw_error;

    if(w_d > param.w_nom) w_d = param.w_nom;
    else if(w_d < -param.w_nom) w_d = -param.w_nom;

}

void NavigationController::goToXYProcessWarehouse() {

    if (route.empty()) {
        v_d = 0.0;
        w_d = 0.0;
        return;
    }

    double position_error = getPositionError();
    double yaw_error = getAlignYawError();
    double dt = 1.0 / param.loop_rate_hz;

    if (position_error <= param.arrive_radius) {
        v_d = 0.0;
        w_d = 0.0;
        return;
    }

    if (!process_warehouse_goto_align_done_) {
        if (std::fabs(yaw_error) <= param.bearing_align_yaw_tol) {
            process_warehouse_goto_align_done_ = true;
        } else {
            v_d = 0.0;
            w_d = param.kp_angular * yaw_error;
            if (w_d > param.w_nom) w_d = param.w_nom;
            else if (w_d < -param.w_nom) w_d = -param.w_nom;
            if (param.w_min > 0.0 && std::abs(w_d) > 0.0 && std::abs(w_d) < param.w_min) {
                w_d = std::copysign(std::min(param.w_min, param.w_nom), w_d);
            }
            return;
        }
    }

    if (route.front().is_warehouse && !route.front().pick_box) {
        const double release_distance = param.drop_pick_box_release_distance;
        if (release_distance > 0.0 &&
            drop_pick_box_release_published_for_node_id_ != route.front().node_id &&
            position_error <= release_distance) {
            std_msgs::Bool pick_box_msg;
            pick_box_msg.data = false;
            pickBoxPub.publish(pick_box_msg);
            drop_pick_box_release_published_for_node_id_ = route.front().node_id;

            ROS_INFO("NavigationController: Early /pick_box=false for drop node_id=%d at dist=%.3f m (threshold=%.3f m)",
                     route.front().node_id, position_error, release_distance);
        }
    } else {
        drop_pick_box_release_published_for_node_id_ = -1;
    }

    w_d = 0.0;

    double v_target = std::min(param.v_nom, param.v_max);
    if (v_target < param.v_min)
        v_target = param.v_min;

    if (v_target > v_d) {
        v_d += param.a_max * dt;
        if (v_d > v_target) v_d = v_target;
    } else {
        v_d -= param.d_max * dt;
        if (v_d < v_target) v_d = v_target;
    }

    if (v_d > param.v_max)
        v_d = param.v_max;
    if (v_d < -param.v_max)
        v_d = -param.v_max;

    if (isBackwards())
        v_d = -v_d;
}

void NavigationController::goToXY() {

    if(route.empty()) {
        v_d = 0.0;
        w_d = 0.0;
        return;
    }

    double position_error = getPositionError();
    double yaw_error = getAlignYawError();
    double dt = 1.0 / param.loop_rate_hz;

    if (position_error <= param.arrive_radius) {
        v_d = 0.0;
        w_d = 0.0;
        return;
    }

    w_d = param.kp_angular * yaw_error;
    if (w_d > param.w_nom) w_d = param.w_nom;
    else if (w_d < -param.w_nom) w_d = -param.w_nom;

    double v_mag = 0.0;

    if (std::fabs(yaw_error) <= M_PI / 6.0) {
        v_mag = param.v_nom
                * std::cos(yaw_error)
                * std::min<double>(1.0, param.kp_linear * position_error);
    }

    double v_target = v_mag;

    if (v_target > 0.0 && v_target < param.v_min)
        v_target = param.v_min;

    if (v_target > param.v_max)
        v_target = param.v_max;

    if (v_target > v_d) {
        v_d += param.a_max * dt;
        if (v_d > v_target) v_d = v_target;
    } else {
        v_d -= param.d_max * dt;
        if (v_d < v_target) v_d = v_target;
    }

    if (v_d > param.v_max)
        v_d = param.v_max;
    if (v_d < -param.v_max)
        v_d = -param.v_max;

    if (isBackwards())
        v_d = -v_d;
}

void NavigationController::followLine() {
    
    if(route.empty()) {
        v_d = 0.0;
        w_d = 0.0;
        return;
    }

    Line line;
    
    currentWaypoint = route.front();
    line.pf = currentWaypoint;
    line.pi = previousWaypoint;
    
    double vel_lin_nom_eff = (currentWaypoint.vel_lin_nom > 0) ? 
                              currentWaypoint.vel_lin_nom : param.vel_lin_nom;
    
    double error_dist = std::sqrt((line.pf.pose.x - poseCurr.x) * (line.pf.pose.x - poseCurr.x) + 
                                  (line.pf.pose.y - poseCurr.y) * (line.pf.pose.y - poseCurr.y));

    double distLine;
    dist2Line(line.pi.pose.x, line.pi.pose.y, line.pf.pose.x, line.pf.pose.y, poseCurr.x, poseCurr.y, distLine);

    double k1_virtual, error_ang;
    dist2LineVirtual(line.pi.pose.x, line.pi.pose.y, line.pf.pose.x, line.pf.pose.y,
                     poseCurr.x, poseCurr.y, k1_virtual, error_ang);
    double k1_eff = k1_virtual;
    double error_ang_deg = std::abs(error_ang) * 180.0 / M_PI;

    publishVirtualLineMarker(line.pi.pose.x, line.pi.pose.y, line.pf.pose.x, line.pf.pose.y);

    followLineFsm.update_tis();
    
    bool pf_is_line_before_warehouse = (route.size() > 1 && route[1].is_warehouse);
    bool skip_approaching_straight = false;
    if (pf_is_line_before_warehouse) {
        const WayPoint& wh = route[1];
        double dx1 = line.pf.pose.x - line.pi.pose.x;
        double dy1 = line.pf.pose.y - line.pi.pose.y;
        double dx2 = wh.pose.x - line.pf.pose.x;
        double dy2 = wh.pose.y - line.pf.pose.y;
        const double len_eps = 1e-4;
        double len1_sq = dx1 * dx1 + dy1 * dy1;
        double len2_sq = dx2 * dx2 + dy2 * dy2;
        if (len1_sq > len_eps * len_eps && len2_sq > len_eps * len_eps) {
            double a1 = std::atan2(dy1, dx1);
            double a2 = std::atan2(dy2, dx2);
            double dtheta = std::abs(normalizeAngle(a2 - a1));
            if (dtheta < param.approaching_colinear_angle_rad) {
                skip_approaching_straight = true;
            }
        }
    }

    if (followLineFsm.state == navigation::followLineStates::Follow_Line) {
        if (line.pf.is_warehouse) {
            if (line.pf.is_process_warehouse) {
                followLineFsm.new_state = navigation::followLineStates::Approaching_process_PickDrop;
                ROS_WARN("approaching process pick/drop state");
            } else {
                followLineFsm.new_state = navigation::followLineStates::Approaching_PickDrop;
                ROS_WARN("approaching pick/drop state");
            }
        } else if (pf_is_line_before_warehouse && error_dist <= param.approaching_enter_dist_m
                   && !skip_approaching_straight) {
            followLineFsm.new_state = navigation::followLineStates::Approaching;
            approaching_brake_ref_dist_ = std::max(error_dist, 1e-3);
            ROS_WARN("approaching normal state (dist_pf=%.3f m <= %.3f m, ref_dist=%.3f m)",
                     error_dist, param.approaching_enter_dist_m, approaching_brake_ref_dist_);
        }
    }

    followLineFsm.set_state();
    
    const double completion_threshold = 0.7;
    if (line_progress > completion_threshold && !completion_feedback_sent) {
        plan_handler::CompletionFeedback feedback;
        feedback.x = line.pf.pose.x;
        feedback.y = line.pf.pose.y;
        navCompletionFeedbackPub.publish(feedback);
        completion_feedback_sent = true;
        ROS_INFO("NavigationController: Published completion feedback for waypoint (%.3f, %.3f) at %.1f%% progress", 
                 feedback.x, feedback.y, line_progress * 100.0);
    }
    if (line_progress < completion_threshold) {
        completion_feedback_sent = false;
    }

    auto stanleyDenomFromQuadraticW = [&](double v_nom_line, double w_probe) -> double {
        double Aq = -v_nom_line / (param.w_nom * param.w_nom);
        double v_quad = std::max(Aq * (w_probe - param.w_nom) * (w_probe + param.w_nom), 0.0);
        return std::max(v_quad, param.stanley_soft_v) + param.stanley_eps;
    };

    if (followLineFsm.state == navigation::followLineStates::Follow_Line) {

        if (param.use_stanley_follow_line) {
            double v_nom_lin = std::max(std::abs(vel_lin_nom_eff), param.stanley_soft_v) + param.stanley_eps;
            double w_coarse = error_ang + std::atan2(param.stanley_k * k1_eff, v_nom_lin);
            double v_den = stanleyDenomFromQuadraticW(vel_lin_nom_eff, w_coarse);
            w_d = error_ang + std::atan2(param.stanley_k * k1_eff, v_den);
        } else {
            w_d = param.k_line * k1_eff + param.gain_fwd * error_ang;
        }

        double A = -vel_lin_nom_eff/(param.w_nom*param.w_nom);
        v_d = std::max(A * (w_d - param.w_nom) * (w_d + param.w_nom), 0.0);

        last_vel_before_approaching_ = std::abs(v_d);

        if (w_d > param.w_nom) w_d = param.w_nom;
        else if (w_d < -param.w_nom) w_d = -param.w_nom;

    }
    else if (followLineFsm.state == navigation::followLineStates::Approaching) {
        double v_scale = error_dist / std::max(approaching_brake_ref_dist_, 1e-4);
        if (v_scale > 1.0) v_scale = 1.0;
        double v_prop = last_vel_before_approaching_ * v_scale;
        if (v_prop > last_vel_before_approaching_) v_prop = last_vel_before_approaching_;
        double v_ref_approaching = std::max(v_prop, param.approaching_vel_normal);

        if (param.use_stanley_approaching) {
            double v_nom_lin = std::max(std::abs(v_ref_approaching), param.stanley_soft_v) + param.stanley_eps;
            double w_coarse = error_ang + std::atan2(param.k_approaching * k1_eff, v_nom_lin);
            double v_den = stanleyDenomFromQuadraticW(v_ref_approaching, w_coarse);
            w_d = error_ang + std::atan2(param.k_approaching * k1_eff, v_den);
        } else {
            w_d = param.k_approaching * k1_eff + param.gain_approaching_fwd * error_ang;
        }

        v_d = v_ref_approaching;

        if (w_d > param.w_nom)       w_d = param.w_nom;
        else if (w_d < -param.w_nom) w_d = -param.w_nom;

    }
    else if (followLineFsm.state == navigation::followLineStates::Approaching_PickDrop) {
        if (param.use_stanley_approaching) {
            double v_nom_lin = std::max(param.approaching_vel_pickdrop, param.stanley_soft_v) + param.stanley_eps;
            double w_coarse = error_ang + std::atan2(param.k_approaching_pickdrop * k1_eff, v_nom_lin);
            double w_ratio_c = (param.w_nom > 1e-6) ? std::min(std::abs(w_coarse) / param.w_nom, 1.0) : 1.0;
            double v_quad = std::max(param.approaching_vel_pickdrop * (1.0 - w_ratio_c * w_ratio_c), 0.0);
            double v_den = std::max(v_quad, param.stanley_soft_v) + param.stanley_eps;
            w_d = error_ang + std::atan2(param.k_approaching_pickdrop * k1_eff, v_den);
        } else {
            w_d = param.k_approaching_pickdrop * k1_eff + param.gain_approaching_fwd_pickdrop * error_ang;
        }

        double w_ratio = (param.w_nom > 1e-6) ? std::min(std::abs(w_d) / param.w_nom, 1.0) : 1.0;
        v_d = std::max(param.approaching_vel_pickdrop * (1.0 - w_ratio * w_ratio), 0.0);

        if (w_d > param.w_nom)       w_d = param.w_nom;
        else if (w_d < -param.w_nom) w_d = -param.w_nom;

    }
    else if (followLineFsm.state == navigation::followLineStates::Approaching_process_PickDrop) {
        if (param.use_stanley_approaching) {
            double v_nom_lin = std::max(param.approaching_vel_process, param.stanley_soft_v) + param.stanley_eps;
            double w_coarse = error_ang + std::atan2(param.k_approaching_process * k1_eff, v_nom_lin);
            double w_ratio_c = (param.w_nom > 1e-6) ? std::min(std::abs(w_coarse) / param.w_nom, 1.0) : 1.0;
            double v_quad = std::max(param.approaching_vel_process * (1.0 - w_ratio_c * w_ratio_c), 0.0);
            double v_den = std::max(v_quad, param.stanley_soft_v) + param.stanley_eps;
            w_d = error_ang + std::atan2(param.k_approaching_process * k1_eff, v_den);
        } else {
            w_d = param.k_approaching_process * k1_eff + param.gain_approaching_fwd_process * error_ang;
        }

        double w_ratio = (param.w_nom > 1e-6) ? std::min(std::abs(w_d) / param.w_nom, 1.0) : 1.0;
        v_d = std::max(param.approaching_vel_process * (1.0 - w_ratio * w_ratio), 0.0);

        if (w_d > param.w_nom)       w_d = param.w_nom;
        else if (w_d < -param.w_nom) w_d = -param.w_nom;

    }

    if (error_ang_deg > 93.0) {
        v_d = 0.0;
        ROS_WARN_THROTTLE(1.0, "NavigationController: Angular error %.1f deg > 93 deg, setting linear velocity to 0", error_ang_deg);
    }
    
    if (isBackwards() && v_d > 0.0) {
        v_d = -v_d;
    }

    const char* state_str = "Follow_Line";
    if (followLineFsm.state == navigation::followLineStates::Approaching) state_str = "Approaching";
    else if (followLineFsm.state == navigation::followLineStates::Approaching_PickDrop) state_str = "Approaching_PickDrop";
    else if (followLineFsm.state == navigation::followLineStates::Approaching_process_PickDrop) state_str = "Approaching_process_PickDrop";
    ROS_INFO("[FOLLOW_LINE] Line: (%.2f,%.2f)->(%.2f,%.2f) | progress=%.0f%% | dist=%.3f | state=%s | dist_da=%.3f", 
             line.pi.pose.x, line.pi.pose.y, line.pf.pose.x, line.pf.pose.y, 
             line_progress * 100, error_dist, state_str, param.dist_da);

}

void NavigationController::navigationFsmRunner(const ros::TimerEvent&) {

    navigationFsm.update_tis();
    const int nav_state_at_tick_start = navigationFsm.state;
    bool enable = !(mode == "stop" || mode == "pause") && !route.empty();

    if(navigationFsm.state == navigation::states::idle && enable) {

        navigationFsm.new_state = navigation::states::driveToGoal;

    }

    else if(navigationFsm.state == navigation::states::driveToGoal && isPositionArrived() && route.front().align && enable) {
        
        previousWaypoint = route.front();

        // =========================
        // PUBLICAR NÓ ATUAL
        // =========================
        publishCurrentNode(previousWaypoint.node_id);
        
        if (route.front().pick_box) {
            navigationFsm.new_state = navigation::states::pickBoxForward;
            pick_box_forward_start_time = ros::Time::now();
            in_pick_box_forward = true;
            ROS_INFO("NavigationController: Arrived at pick warehouse (id=%d, x=%.2f, y=%.2f, backwards=%d), entering pickBoxForward state", 
                     previousWaypoint.id, previousWaypoint.pose.x, previousWaypoint.pose.y, previousWaypoint.backwards ? 1 : 0);
        } else {
            navigationFsm.new_state = navigation::states::turnToFinalYaw;
        }

    }

    else if(navigationFsm.state == navigation::states::driveToGoal && !route.front().align && enable) {

        bool warehouse_pickdrop_goto_xy = route.front().is_warehouse && !route.front().align;
        if (!warehouse_pickdrop_goto_xy) {

            double switch_ratio = (route.front().line_switch_ratio > 0) ? 
                                   route.front().line_switch_ratio : param.line_switch_ratio;
            
            if (line_progress >= switch_ratio) {
                previousWaypoint = route.front();

                // =========================
                // PUBLICAR NÓ ATUAL
                // =========================
                publishCurrentNode(previousWaypoint.node_id);
                
                bool is_pick_warehouse = route.front().pick_box;
                
                ROS_INFO("Line switch at %.0f%% (threshold=%.0f%%): waypoint (id=%d, x=%.2f, y=%.2f, pick_box=%d, backwards=%d, node_id=%d). Remaining: %zu", 
                         line_progress * 100, switch_ratio * 100,
                         previousWaypoint.id, previousWaypoint.pose.x, previousWaypoint.pose.y, is_pick_warehouse ? 1 : 0, 
                         previousWaypoint.backwards ? 1 : 0, previousWaypoint.node_id, route.size() - 1);
            
                if (is_pick_warehouse) {
                    navigationFsm.new_state = navigation::states::pickBoxForward;
                    pick_box_forward_start_time = ros::Time::now();
                    in_pick_box_forward = true;
                    ROS_INFO("NavigationController: Completed line to pick warehouse, entering pickBoxForward state");
                } else {
                    transitionAfterDropWarehouse("followline 3");
                }
            }
        }
    }

    else if (navigationFsm.state == navigation::states::processWarehouseGoToXY && enable && isPositionArrived()) {

        // Se o feedback a ~70% não correu (ou não fez match no plan_handler), garantir remoção na stack
        if (!process_warehouse_goto_completion_sent_ && !route.empty()) {
            plan_handler::CompletionFeedback fb;
            fb.x = route.front().pose.x;
            fb.y = route.front().pose.y;
            navCompletionFeedbackPub.publish(fb);
            process_warehouse_goto_completion_sent_ = true;
            ROS_INFO("NavigationController: warehouse go-to-XY arrival completion feedback (%.3f, %.3f)", fb.x, fb.y);
        }

        previousWaypoint = route.front();
        publishCurrentNode(previousWaypoint.node_id);

        bool is_pick_warehouse = route.front().pick_box;
        ROS_INFO("NavigationController: Arrived at warehouse (go-to-XY) id=%d node_id=%d pick_box=%d",
                 previousWaypoint.id, previousWaypoint.node_id, is_pick_warehouse ? 1 : 0);

        if (is_pick_warehouse) {
            navigationFsm.new_state = navigation::states::pickBoxForward;
            pick_box_forward_start_time = ros::Time::now();
            in_pick_box_forward = true;
        } else {
            transitionAfterDropWarehouse("followline process goto pop");
        }
    }

    else if (navigationFsm.state == navigation::states::turnToFinalYaw && enable && !isPositionArrived()) {

        navigationFsm.new_state = navigation::states::driveToGoal;

    }

    else if(navigationFsm.state == navigation::states::turnToFinalYaw && isYawDesired() && enable) {

        bool is_pick_warehouse = false;
        if(!route.empty()) {
            previousWaypoint = route.front();

            // =========================
            // PUBLICAR NÓ ATUAL
            // =========================
            publishCurrentNode(previousWaypoint.node_id);

            is_pick_warehouse = route.front().pick_box;
            ROS_INFO("Stored previousWaypoint (id=%d, x=%.2f, y=%.2f, pick_box=%d, node_id=%d) before removing (turnToFinalYaw). Remaining waypoints: %zu", 
                     previousWaypoint.id, previousWaypoint.pose.x, previousWaypoint.pose.y, is_pick_warehouse ? 1 : 0,
                     previousWaypoint.node_id, route.size() - 1);
        }
        
        if (is_pick_warehouse) {
            navigationFsm.new_state = navigation::states::pickBoxForward;
            pick_box_forward_start_time = ros::Time::now();
            in_pick_box_forward = true;
            ROS_INFO("NavigationController: Completed turnToFinalYaw at pick warehouse, entering pickBoxForward state");
        } else {
            transitionAfterDropWarehouse("followline 4");
        }

    }

    else if(navigationFsm.state == navigation::states::done && enable) {

        navigationFsm.new_state = navigation::states::driveToGoal;

    }

    else if(navigationFsm.state == navigation::states::done && !enable) {

        navigationFsm.new_state = navigation::states::idle;

    }

    else if(navigationFsm.state == navigation::states::pickBoxForward && enable) {
        double elapsed_time = (ros::Time::now() - pick_box_forward_start_time).toSec();
        ROS_WARN("pickboxforward state: elapsed time = %.2f seconds", elapsed_time);
        if (elapsed_time >= 0.7) {
            in_pick_box_forward = false;
            if (!route.empty()) {
                route.pop_front();
                updateDesiredPose();
                followLineFsm.new_state = navigation::followLineStates::Follow_Line;
                ROS_WARN("followline 5");
                followLineFsm.set_state();
                completion_feedback_sent = false;
            }
            if(route.empty()) {
                navigationFsm.new_state = navigation::states::idle;
            } else {
                navigationFsm.new_state = navigation::states::done;
            }
            ROS_INFO("NavigationController: Completed pickBoxForward (2s), continuing to next waypoint");
        }
    }

    else if (navigationFsm.state == navigation::states::dropMagnetReleaseWiggle && enable) {
        double traveled = normalizeAngle(poseCurr.theta - drop_magnet_wiggle_start_yaw_);
        if (traveled < 0.0) traveled += 2.0 * M_PI;
        if (traveled >= drop_magnet_wiggle_target_rad_) {
            navigationFsm.new_state = navigation::states::done;
            ROS_INFO("NavigationController: dropMagnetReleaseWiggle finished (traveled=%.3f rad, target=%.3f rad)",
                     traveled, drop_magnet_wiggle_target_rad_);
        }
    }

    if (navigationFsm.new_state == navigation::states::driveToGoal && enable && !route.empty()
        && route.front().is_warehouse && !route.front().align) {
        navigationFsm.new_state = navigation::states::processWarehouseGoToXY;
    }

    navigationFsm.set_state();

    if (navigationFsm.state == navigation::states::processWarehouseGoToXY
        && nav_state_at_tick_start != navigation::states::processWarehouseGoToXY) {
        process_warehouse_goto_align_done_ = false;
        process_warehouse_goto_completion_sent_ = false;
        process_warehouse_goto_start_dist_ = std::max(getPositionError(), 1e-3);
    }

    if (navigationFsm.state == navigation::states::driveToGoal && enable) followLine();
    else if (navigationFsm.state == navigation::states::processWarehouseGoToXY && enable) goToXYProcessWarehouse();
    else if(navigationFsm.state == navigation::states::turnToFinalYaw && enable) setTheta();
    else if(navigationFsm.state == navigation::states::pickBoxForward && enable) {
        const double v_pf = param.pick_box_forward_vel;
        v_d = previousWaypoint.backwards ? -v_pf : v_pf;
        w_d = 0.0;
        ROS_INFO_THROTTLE(0.5, "NavigationController: pickBoxForward state - moving at %.2f m/s (backwards=%d)", 
                         v_d, previousWaypoint.backwards ? 1 : 0);
    }
    else if (navigationFsm.state == navigation::states::dropMagnetReleaseWiggle && enable) {
        v_d = 0.0;
        double w_cmd = param.drop_magnet_wiggle_angular_vel;
        if (std::abs(w_cmd) > param.w_nom) w_cmd = std::copysign(param.w_nom, w_cmd);
        w_d = std::abs(w_cmd);
        double traveled = normalizeAngle(poseCurr.theta - drop_magnet_wiggle_start_yaw_);
        if (traveled < 0.0) traveled += 2.0 * M_PI;
        ROS_INFO_THROTTLE(0.2, "NavigationController: dropMagnetReleaseWiggle w_d=%.3f rad/s traveled=%.1f/%.1f deg",
                          w_d, traveled * 180.0 / M_PI, drop_magnet_wiggle_target_rad_ * 180.0 / M_PI);
    }
    else {
        v_d = 0.0;
        w_d = 0.0;
    }

    // Mesmo critério que followLine (line_progress > 0.7): feedback para plan_handler /pick_box
    if (navigationFsm.state == navigation::states::processWarehouseGoToXY && enable && !route.empty()
        && route.front().is_warehouse) {
        const double completion_threshold = 0.7;
        const double pe = getPositionError();
        const double start = process_warehouse_goto_start_dist_;
        const double progress = (start > 1e-6) ? (1.0 - pe / start) : 1.0;
        if (progress > completion_threshold && !process_warehouse_goto_completion_sent_) {
            plan_handler::CompletionFeedback feedback;
            feedback.x = poseDesired.x;
            feedback.y = poseDesired.y;
            navCompletionFeedbackPub.publish(feedback);
            process_warehouse_goto_completion_sent_ = true;
            ROS_INFO(
                "NavigationController: warehouse go-to-XY completion feedback at %.1f%% (dist=%.3f m, ref=%.3f m)",
                progress * 100.0, pe, start);
        }
        if (progress < completion_threshold) {
            process_warehouse_goto_completion_sent_ = false;
        }
    }

    // =========================
    // TROCAR PARA A PRÓXIMA ROTA
    // =========================
    if (route.empty() && hasPendingNavPlan) {
        loadPendingNavPlanIfAvailable();

        if (!route.empty() && mode != "stop" && mode != "pause") {
            navigationFsm.new_state = navigation::states::driveToGoal;
            navigationFsm.set_state();
            ROS_INFO("NavigationController: Started pending route");
        }
    }

    if (route.empty() && (v_d == 0.0 && w_d == 0.0)) {
        geometry_msgs::Twist cmd;
        cmd.linear.x  = 0.0;
        cmd.linear.y  = 0.0;
        cmd.linear.z  = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;
        velPub.publish(cmd);
        ROS_INFO_THROTTLE(1.0, "NavigationController: Route empty, publishing stop command");
    } else {
        publishVel();
    }

}

bool NavigationController::controlSrvCb(navigation_controller::NavigationControl::Request& req, navigation_controller::NavigationControl::Response& res) {

    mode = req.command;

    if(mode == "start") {

        if (load_from_route) {
            loadRouteFromParameters();
            
            if (route.empty()) {
                res.success = false; res.message = "no waypoints in params";
                return true;
            }
        } else {
            if (route.empty()) {
                res.success = false; res.message = "no route loaded, waiting for /nav_plan";
                ROS_WARN("NavigationController: No route loaded. Waiting for /nav_plan message.");
                return true;
            }
        }

        res.success = true;  res.message = "started";
        ROS_INFO("Navigation START");
        return true;

    }

    else if(mode == "stop") {

        route.clear();
        drop_pick_box_release_published_for_node_id_ = -1;
        previousWaypoint.id = -1;
        previousWaypoint.node_id = -1;
        last_published_node_id = -1;
        hasPendingNavPlan = false;

        navigationFsm.new_state = navigation::states::idle;
        poseDesired = poseCurr;
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

void NavigationController::navPlanCallback(const plan_handler::NavPlan::ConstPtr& msg) {
    ROS_INFO("NavigationController: Received NavPlan with %zu points", msg->points.size());

    // =========================
    // TROCAR PARA A PRÓXIMA ROTA
    // se ainda há rota ativa, guardar pendente
    // =========================
    if (!route.empty() || navigationFsm.state != navigation::states::idle) {
        appendRouteFromNavPlan(msg);
        publishLineMarkers();
        ROS_INFO("NavigationController: Current route active, appended new NavPlan to existing route");
        return;
    }

    loadRouteFromNavPlan(msg);
}

void NavigationController::loadRouteFromNavPlan(const plan_handler::NavPlan::ConstPtr& msg) {
    if (msg->points.empty()) {
        ROS_WARN("NavigationController: Received empty NavPlan, ignoring");
        return;
    }

    route.clear();
    drop_pick_box_release_published_for_node_id_ = -1;
    
    previousWaypoint.id = 0;
    previousWaypoint.pose.x = poseCurr.x;
    previousWaypoint.pose.y = poseCurr.y;
    previousWaypoint.pose.theta = poseCurr.theta;
    previousWaypoint.align = false;
    previousWaypoint.backwards = false;
    previousWaypoint.line_switch_ratio = -1.0;
    previousWaypoint.vel_lin_nom = -1.0;
    previousWaypoint.pick_box = false;
    previousWaypoint.is_warehouse = false;
    previousWaypoint.is_process_warehouse = false;
    previousWaypoint.node_id = -1;
    ROS_INFO("Initial position set as previousWaypoint: x=%.2f y=%.2f", poseCurr.x, poseCurr.y);

    appendRouteFromNavPlan(msg);

    updateDesiredPose();
    
    if (!route.empty()) {
        followLineFsm.new_state = navigation::followLineStates::Follow_Line;
        ROS_WARN("followline 6");
        followLineFsm.set_state();
        skipNearbyWaypoints();
    }
    
    publishLineMarkers();
    completion_feedback_sent = false;
    
    ROS_INFO("NavigationController: Route loaded from NavPlan with %zu waypoints", route.size());
}

void NavigationController::appendRouteFromNavPlan(const plan_handler::NavPlan::ConstPtr& msg) {
    if (msg->points.empty()) {
        ROS_WARN("NavigationController: Received empty NavPlan for append, ignoring");
        return;
    }

    const int start_id = route.empty() ? 0 : route.back().id + 1;
    const Pose append_anchor_pose = route.empty() ? previousWaypoint.pose : route.back().pose;

    for (size_t i = 0; i < msg->points.size(); ++i) {
        const plan_handler::ControllerPoint& cp = msg->points[i];
        WayPoint waypoint_temp;
        waypoint_temp.id = start_id + static_cast<int>(i);

        // =========================
        // RECEBER NODE_ID
        // =========================
        waypoint_temp.node_id = cp.node_id;
        
        waypoint_temp.pose.x = cp.x;
        waypoint_temp.pose.y = cp.y;

        if (i < msg->points.size() - 1) {
            double dx = msg->points[i + 1].x - cp.x;
            double dy = msg->points[i + 1].y - cp.y;
            waypoint_temp.pose.theta = std::atan2(dy, dx);
        } else {
            if (i > 0) {
                double dx = cp.x - msg->points[i - 1].x;
                double dy = cp.y - msg->points[i - 1].y;
                waypoint_temp.pose.theta = std::atan2(dy, dx);
            } else {
                double dx = cp.x - append_anchor_pose.x;
                double dy = cp.y - append_anchor_pose.y;
                waypoint_temp.pose.theta = std::atan2(dy, dx);
            }
        }
        
        waypoint_temp.align = false;
        waypoint_temp.backwards = cp.backwards;
        waypoint_temp.pick_box = cp.pick_box;
        waypoint_temp.is_warehouse = cp.is_warehouse;
        waypoint_temp.is_process_warehouse = cp.is_process_warehouse;
        
        waypoint_temp.line_switch_ratio = (cp.line_switch_ratio > 0) ? cp.line_switch_ratio : -1.0;
        waypoint_temp.vel_lin_nom = (cp.vel_lin_nom > 0) ? cp.vel_lin_nom : -1.0;
        
        double effective_vel = waypoint_temp.vel_lin_nom > 0 ? waypoint_temp.vel_lin_nom : param.vel_lin_nom;
        ROS_INFO("Waypoint %d: node_id=%d x=%.2f y=%.2f yaw=%.2f backwards=%d switch=%.0f%% vel=%.2f", 
                 waypoint_temp.id, waypoint_temp.node_id, waypoint_temp.pose.x, waypoint_temp.pose.y, waypoint_temp.pose.theta,
                 waypoint_temp.backwards, 
                 waypoint_temp.line_switch_ratio > 0 ? waypoint_temp.line_switch_ratio * 100 : param.line_switch_ratio * 100,
                 effective_vel);

        route.push_back(waypoint_temp);
    }
}

// =========================
// PUBLICAR NÓ ATUAL
// =========================
void NavigationController::publishCurrentNode(int node_id) {
    if (node_id < 0) return;
    if (node_id == last_published_node_id) return;

    std_msgs::UInt32 msg;
    msg.data = static_cast<uint32_t>(node_id);
    currentNodePub.publish(msg);

    last_published_node_id = node_id;

    ROS_INFO("NavigationController: Published current node_id=%d to /this_current_pose", node_id);
}

// =========================
// TROCAR PARA A PRÓXIMA ROTA
// =========================
void NavigationController::loadPendingNavPlanIfAvailable() {
    if (!hasPendingNavPlan) return;

    ROS_INFO("NavigationController: Loading pending NavPlan with %zu points", pendingNavPlan.points.size());

    plan_handler::NavPlan::Ptr msg(new plan_handler::NavPlan(pendingNavPlan));
    hasPendingNavPlan = false;
    loadRouteFromNavPlan(msg);
}
