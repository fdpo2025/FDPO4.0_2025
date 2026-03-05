#include "pure_pursuit.h"
#include <cmath>

NavigationController::NavigationController(ros::NodeHandle& nh_) : nh(nh_), v_d(0.0), w_d(0.0), 
navigationFsm(navigation::states::idle),
k1(0.0), previousWaypoint({-1, {0, 0, 0}, false, false, -1.0, -1.0, false}), tfBuffer(), tfListener(tfBuffer),
in_pick_box_forward(false) {
    
    mode = "idle";

    // load navigation parameters
    loadNavigationParams();
    
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
    navCompletionFeedbackPub = nh.advertise<plan_handler::CompletionFeedback>("/nav_completion_feedback", 10);
    completion_feedback_sent = false;
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

void NavigationController::loadRouteFromNavPlan(const plan_handler::NavPlan::ConstPtr& msg) {
  if (msg->points.empty()) return;

  route_full_.clear();
  route_seg_.clear();
  segment_active_ = false;

  // previousWaypoint = pose atual
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

  for (size_t i = 0; i < msg->points.size(); ++i) {
    const auto& cp = msg->points[i];
    WayPoint wp;
    wp.id = (int)i;
    wp.pose.x = cp.x;
    wp.pose.y = cp.y;

    // theta como tu já fazes
    if (i < msg->points.size() - 1) {
      double dx = msg->points[i + 1].x - cp.x;
      double dy = msg->points[i + 1].y - cp.y;
      wp.pose.theta = std::atan2(dy, dx);
    } else if (i > 0) {
      double dx = cp.x - msg->points[i - 1].x;
      double dy = cp.y - msg->points[i - 1].y;
      wp.pose.theta = std::atan2(dy, dx);
    } else {
      wp.pose.theta = poseCurr.theta;
    }

    wp.align = false;
    wp.backwards = cp.backwards;
    wp.pick_box = cp.pick_box;
    wp.is_warehouse = cp.is_warehouse;
    wp.line_switch_ratio = (cp.line_switch_ratio > 0) ? cp.line_switch_ratio : -1.0;
    wp.vel_lin_nom = (cp.vel_lin_nom > 0) ? cp.vel_lin_nom : -1.0;

    route_full_.push_back(wp);
  }

  buildNextSegment();          // <<<<<< cria route_seg_
  updateDesiredPoseSegment();  // desired = route_seg_.front()
  buildSmoothedPathFromSegment();
  publishLineMarkersSegment();

  completion_feedback_sent = false;
}

void NavigationController::loadNavigationParams() {

    nh.param("v_nom", param.v_nom, 0.4);
    nh.param("w_nom", param.w_nom, 1.2);
    nh.param("w_min", param.w_min, 0.1);
    nh.param("v_min", param.v_min, 0.07);
    nh.param("v_max", param.v_max, 0.5);  // Limite máximo de velocidade (m/s)
    nh.param("a_max", param.a_max, 0.5);
    nh.param("d_max", param.d_max, 0.5);
    nh.param("kp_linear", param.kp_linear, 5.0);
    nh.param("kp_angular", param.kp_angular, 2.0/M_PI * param.w_nom);
    nh.param("k_line", param.k_line, 1.0);  // Ganho para correção de linha (default: 2.0)
    nh.param("arrive_radius",  param.arrive_radius, 0.05);
    nh.param("yaw_tol",param.yaw_tol, 0.08);
    nh.param("loop_rate_hz", param.loop_rate_hz, 30);
       
    nh.param("pp_Ld", param.pp_Ld_, 0.9);
    nh.param("pp_vref", param.pp_vref_, 0.3);
    nh.param("smooth_radius", param.smooth_radius_, 0.25);
    nh.param("smooth_corner_steps", param.smooth_corner_steps_, 6);
    nh.param("pp_L0",     param.pp_L0_,     0.35);
    nh.param("pp_kv",     param.pp_kv_,     1.0);
    nh.param("pp_Ld_min", param.pp_Ld_min_, 0.25);
    nh.param("pp_Ld_max", param.pp_Ld_max_, 1.20);
    nh.param("pp_kc", param.pp_kc, 0.2);
            
}

void NavigationController::updateDesiredPoseSegment() {
  if (route_seg_.empty()) return;
  poseDesired = route_seg_.back().pose;   // <- objetivo do segmento = warehouse
}

bool NavigationController::isBackwards() {
  return !route_seg_.empty() ? route_seg_.front().backwards : false;
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


void NavigationController::publishLineMarkersSegment() {
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

  if (route_seg_.size() < 2) {
    lineMarkerPub.publish(line_marker);
    return;
  }

  for (auto it = route_seg_.begin(); it != route_seg_.end(); ++it) {
    auto next_it = std::next(it);
    if (next_it == route_seg_.end()) break;

    geometry_msgs::Point p1, p2;
    p1.x = it->pose.x;      p1.y = it->pose.y;      p1.z = 0.0;
    p2.x = next_it->pose.x; p2.y = next_it->pose.y; p2.z = 0.0;

    std_msgs::ColorRGBA c;
    if (next_it->backwards) { c.r=0.0; c.g=0.5; c.b=1.0; c.a=1.0; }
    else                    { c.r=0.0; c.g=1.0; c.b=0.0; c.a=1.0; }

    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);
    line_marker.colors.push_back(c);
    line_marker.colors.push_back(c);
  }

  lineMarkerPub.publish(line_marker);
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

    if(!rvizGoalAppend) {
        route_seg_.clear();
        // Inicializar previousWaypoint com posição atual do robô
        previousWaypoint.id = 0;
        previousWaypoint.pose.x = poseCurr.x;
        previousWaypoint.pose.y = poseCurr.y;
        previousWaypoint.pose.theta = poseCurr.theta;
        previousWaypoint.align = false;
        previousWaypoint.backwards = false;
        previousWaypoint.line_switch_ratio = -1.0;
        previousWaypoint.vel_lin_nom = -1.0;
        previousWaypoint.is_warehouse = false;
        ROS_INFO("Initial position set as previousWaypoint: x=%.2f y=%.2f", poseCurr.x, poseCurr.y);
    }

    WayPoint waypoint_temp;

    waypoint_temp.id = route_seg_.empty() ? 1 : (route_seg_.back().id + 1);  // IDs começam em 1 (0 é previousWaypoint)
    waypoint_temp.pose.x = poseInMap.pose.position.x;
    waypoint_temp.pose.y = poseInMap.pose.position.y;
    waypoint_temp.pose.theta = tf2::getYaw(poseInMap.pose.orientation);
    waypoint_temp.align = true;
    waypoint_temp.backwards = false;
    waypoint_temp.line_switch_ratio = -1.0;  // Usar parâmetro global
    waypoint_temp.vel_lin_nom = -1.0;        // Usar parâmetro global

    route_seg_.push_back(waypoint_temp);
    ROS_INFO("RViz goal added: x=%.2f y=%.2f yaw=%.2f (map frame)", 
             waypoint_temp.pose.x, waypoint_temp.pose.y, waypoint_temp.pose.theta);

    updateDesiredPoseSegment();
           
    // Publicar linhas de visualização
    publishLineMarkersSegment();

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

    // Se não há waypoints, parar imediatamente
    if(route_seg_.empty()) {
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

void NavigationController::navigationFsmRunner(const ros::TimerEvent&) {

    // Update's
    navigationFsm.update_tis();
    bool enable = !(mode == "stop" || mode == "pause") && !route_seg_.empty();

    // Compute Transitions
    if(navigationFsm.state == navigation::states::idle && enable) {

        navigationFsm.new_state = navigation::states::driveToGoal;

    }

    // Quando align=true: usar isPositionArrived() para parar no ponto exato e rodar
    else if(navigationFsm.state == navigation::states::driveToGoal && isPositionArrived() && enable) {
                        
        
        if (route_seg_.back().pick_box) {
            navigationFsm.new_state = navigation::states::pickBoxForward;
            pick_box_forward_start_time = ros::Time::now();
            in_pick_box_forward = true;           
        } else {
            navigationFsm.new_state = navigation::states::turnToFinalYaw;
        }

    }
    
    else if (navigationFsm.state == navigation::states::driveToGoal && enable) {

        if (isPositionArrived()) {

            WayPoint seg_goal = route_seg_.back();
            previousWaypoint = seg_goal;

            // se objetivo do segmento é pick warehouse -> pickBoxForward
            if (seg_goal.pick_box) {
            navigationFsm.new_state = navigation::states::pickBoxForward;
            pick_box_forward_start_time = ros::Time::now();
            in_pick_box_forward = true;
            } else {
            // fecha segmento e abre próximo
            route_seg_.clear();
            buildNextSegment();
            updateDesiredPoseSegment();
            buildSmoothedPathFromSegment();
            publishLineMarkersSegment(); // (tens de criar esta versão)

            completion_feedback_sent = false;

            if (route_seg_.empty() && route_full_.empty()) navigationFsm.new_state = navigation::states::idle;
            else navigationFsm.new_state = navigation::states::done;
            }
        }
    }

    else if (navigationFsm.state == navigation::states::turnToFinalYaw && enable && !isPositionArrived()) {

        navigationFsm.new_state = navigation::states::driveToGoal;

    }

    else if(navigationFsm.state == navigation::states::turnToFinalYaw && isYawDesired() && enable) {

        // Guardar waypoint anterior antes de remover
        bool is_pick_warehouse = false;
        if(!route_seg_.empty()) {
            
            is_pick_warehouse = route.back().pick_box;
           
        }
        
        // Se é warehouse de pick, NÃO remover ainda - será removido quando sair do estado pickBoxForward
        if (is_pick_warehouse) {
            navigationFsm.new_state = navigation::states::pickBoxForward;
            pick_box_forward_start_time = ros::Time::now();
            in_pick_box_forward = true;
            ROS_INFO("NavigationController: Completed turnToFinalYaw at pick warehouse, entering pickBoxForward state");
        } else {
                            
            
            // Se não há mais waypoints, ir direto para idle
            if(route_full_.empty()) {
                navigationFsm.new_state = navigation::states::idle;
            } else {
                buildNextSegment();
                updateDesiredPoseSegment();
                navigationFsm.new_state = navigation::states::done;
            }
        }

    }

    else if(navigationFsm.state == navigation::states::done && enable) {

        navigationFsm.new_state = navigation::states::driveToGoal;

    }

    else if(navigationFsm.state == navigation::states::done && !enable) {

        navigationFsm.new_state = navigation::states::idle;

    }

    // Estado pickBoxForward: andar para frente 2s após chegar a warehouse de pick
    else if(navigationFsm.state == navigation::states::pickBoxForward && enable) {
        double elapsed_time = (ros::Time::now() - pick_box_forward_start_time).toSec();
        
        if (elapsed_time >= 2.0) {

            in_pick_box_forward = false;

            // fecha segmento atual
            route_seg_.clear();

            // abre próximo
            buildNextSegment();
            updateDesiredPoseSegment();
            buildSmoothedPathFromSegment();
            publishLineMarkersSegment();

            completion_feedback_sent = false;

            navigationFsm.new_state = (route_seg_.empty() && route_full_.empty())
                                        ? navigation::states::idle
                                        : navigation::states::done;
        }
    }

    // Set states
    navigationFsm.set_state();

    // Compute Actions
    if(navigationFsm.state == navigation::states::driveToGoal && enable) purePursuitFollowPath();
    else if(navigationFsm.state == navigation::states::turnToFinalYaw && enable) setTheta();
    else if(navigationFsm.state == navigation::states::pickBoxForward && enable) {
        // Andar para frente com velocidade linear 0.1 m/s durante 2s
        // Se estava indo backwards, andar para trás
        v_d = 0.1;
        w_d = 0.0;
        ROS_INFO_THROTTLE(0.5, "NavigationController: pickBoxForward state - moving at %.2f m/s (backwards=%d)", 
                         v_d, previousWaypoint.backwards ? 1 : 0);
    }
    else {
        // Parar se não há waypoints ou estado inválido
        v_d = 0.0;
        w_d = 0.0;
    }

    // Affect outputs
    // Se route está vazia, publicar zeros explicitamente para parar o robô
    if (route_seg_.empty() && route_full_.empty() && (v_d == 0.0 && w_d == 0.0)) {
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

    if (mode == "start") {

        if (load_from_route) {
            ROS_WARN("pure_pursuit_node: load_from_route=true mas este node nao carrega YAML route. Envia /nav_plan.");
            res.success = false;
            res.message = "pure_pursuit_node does not load route.yaml; send /nav_plan";
            return true;
        }

        if (route_seg_.empty() && route_full_.empty()) {
            res.success = false;
            res.message = "no route loaded, waiting for /nav_plan";
            ROS_WARN("pure_pursuit_node: No route loaded. Waiting for /nav_plan.");
            return true;
        }

        res.success = true;
        res.message = "started";
        ROS_INFO("Navigation START");
        return true;
    }

    else if(mode == "stop") {

        route.clear();
        previousWaypoint.id = -1;  // Reset previousWaypoint quando para
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
    loadRouteFromNavPlan(msg);
}


int NavigationController::nearestPointIndex(const std::vector<Point>& p, double x, double y, int last_idx) {
  if (p.empty()) return 0;
  int n = (int)p.size();
  int start = std::max(0, std::min(last_idx, n-1));

  // procura localmente pra frente (rápido). se quiser robustez, pode ampliar janela.
  int best = start;
  double best_d2 = std::numeric_limits<double>::infinity();

  int end = std::min(n, start + 200); // janela
  for (int i = start; i < end; ++i) {
    double dx = p[i].x - x, dy = p[i].y - y;
    double d2 = dx*dx + dy*dy;
    if (d2 < best_d2) { best_d2 = d2; best = i; }
  }

  // fallback: se piorou muito, varre tudo (opcional)
  return best;
}

int NavigationController::lookaheadTargetIndex(const std::vector<Point>& p, double x, double y, int near_idx, double Ld) {
  int n = (int)p.size();
  int target = near_idx;
  int end = std::min(n, near_idx + 2000);
  for (int j = near_idx; j < end; ++j) {
    if (std::hypot(p[j].x - x, p[j].y - y) >= Ld) { target = j; break; }
  }
  return target;
}

void NavigationController::buildSmoothedPathFromSegment() {
  path_pts_.clear();
  path_ready_ = false;

  if (route_seg_.empty()) return;

  std::vector<Point> raw;
  raw.reserve(route_seg_.size() + 1);

  raw.push_back({previousWaypoint.pose.x, previousWaypoint.pose.y});
  for (const auto& wp : route_seg_) raw.push_back({wp.pose.x, wp.pose.y});

  std::vector<Point> sm = raw;
  if (raw.size() >= 3) sm = smoothPath(raw, param.smooth_radius_, (int)param.smooth_corner_steps_);

  path_pts_ = std::move(sm);
  last_near_idx_ = 0;
  target_idx_ = 0;
  path_ready_ = (path_pts_.size() >= 2);

  ROS_INFO("PP: segment path built. raw=%zu smoothed=%zu", raw.size(), path_pts_.size());
}

void NavigationController::purePursuitFollowPath() {
  // Agora o PP usa o segmento ativo
  if (!path_ready_ || path_pts_.size() < 2 || route_seg_.empty()) {
    v_d = 0.0; w_d = 0.0;
    return;
  }

  // Estado atual
  PPState st;
  st.x   = poseCurr.x;
  st.y   = poseCurr.y;
  st.yaw = poseCurr.theta;
  st.v   = std::abs(v_d);

  // Waypoint final do segmento (normalmente a warehouse)
  const WayPoint seg_init = route_seg_.front();
  const WayPoint& seg_goal = route_seg_.back();
  
  // Lookahead (fixo)
  //const double Ld = pp_Ld_;

  // velocidade "do controlador" (usa o que tens agora + o que queres impor)
  const double v_abs = std::abs(v_d);  // ou std::abs(st.v), mas v_d é o comando atual

  double Ld = param.pp_L0_ + param.pp_kv_ * v_abs;
  //Ld = std::clamp(Ld, param.pp_Ld_min_, param.pp_Ld_max_);

  // nearest e target no path suavizado
  int near_idx = nearestPointIndex(path_pts_, st.x, st.y, last_near_idx_);
  int tgt_idx  = lookaheadTargetIndex(path_pts_, st.x, st.y, near_idx, Ld);

  last_near_idx_ = near_idx;
  target_idx_    = tgt_idx;

  const auto& tgt = path_pts_[tgt_idx];

  // Ângulo para o target
  const double angle_to_target = std::atan2(tgt.y - st.y, tgt.x - st.x);

  // -------------------------------------------------------
  // BACKWARDS: calcular alpha com yaw invertido (se segmento em ré)
  // -------------------------------------------------------
  const bool backwards = seg_init.backwards; // ou isBackwards() se já estiver adaptado
  const double yaw_eff = backwards ? normalizeAngle(st.yaw + M_PI) : st.yaw;
  const double alpha   = normalizeAngle(angle_to_target - yaw_eff);

  // Curvatura e comandos
  const double kappa = 2.0 * std::sin(alpha) / std::max(1e-3, param.pp_kc);

  // Velocidade nominal do segmento:
  // Podes escolher: usar a velocidade do goal (warehouse) ou a mínima do segmento.
  double v_ref = param.pp_vref_;

  // opção A (simples): usar vel do waypoint final se existir
  //if (seg_goal.vel_lin_nom > 0) v_ref = seg_goal.vel_lin_nom;

  // opcional: reduzir velocidade ao aproximar do fim do segmento
  // (fica suave e evita overshoot)
  const double dist_to_goal = std::hypot(seg_goal.pose.x - st.x, seg_goal.pose.y - st.y);
  if (dist_to_goal < 0.20) {               // 20 cm antes do fim
    v_ref = std::min(v_ref, 0.10);         // limita a 0.10 m/s
  }
  if (dist_to_goal < 0.10) {               // 10 cm
    v_ref = std::min(v_ref, 0.05);         // limita a 0.05 m/s
  }

  double w_cmd = v_ref * kappa;

  // Saturação de w
  if (w_cmd > param.w_nom)       w_cmd = param.w_nom;
  else if (w_cmd < -param.w_nom) w_cmd = -param.w_nom;

  // v_target com limites + acel/dec
  const double dt = 1.0 / param.loop_rate_hz;

  double v_target = std::clamp(v_ref, 0.0, param.v_max);
  if (v_target > 0.0 && v_target < param.v_min) v_target = param.v_min;

  const double v_curr_abs = std::abs(v_d);

  double v_next = v_curr_abs;
  if (v_target > v_curr_abs) {
    v_next += param.a_max * dt;
    if (v_next > v_target) v_next = v_target;
  } else {
    v_next -= param.d_max * dt;
    if (v_next < v_target) v_next = v_target;
  }

  // Se erro angular muito grande, zera linear (mantém tua lógica)
  const double alpha_deg = std::abs(alpha) * 180.0 / M_PI;
  if (alpha_deg > 93.0) v_next = 0.0;

  // Aplicar sinal de velocidade conforme backwards
  v_d = v_next;
  w_d = w_cmd;

  if(backwards){
    v_d = -v_next;
    w_d = 0;

    if (isNearSegInit(0.05)) {  
       route_seg_.pop_front();
    } 
  }


  ROS_INFO_THROTTLE(0.5,
    "[PP_SEG] near=%d tgt=%d Ld=%.2f dist_goal=%.2f v=%.2f w=%.2f backwards=%d",
    near_idx, tgt_idx, Ld, dist_to_goal, v_d, w_d, backwards ? 1 : 0
  );
}

void NavigationController::buildNextSegment() {
  route_seg_.clear();

  if (route_full_.empty()) {
    segment_active_ = false;
    return;
  }

  while (!route_full_.empty()) {

    // adiciona o ponto atual
    route_seg_.push_back(route_full_.front());
    route_full_.pop_front();

    // se acabou, fecha
    if (route_full_.empty()) break;

    // REGRA: se o PRÓXIMO ponto for backwards=true, fecha segmento AGORA
    if (route_full_.front().backwards) {
      break;
    }
  }

  segment_active_ = !route_seg_.empty();

  ROS_INFO("Segment built (break if next is backwards): %zu points (remaining full: %zu). Next_backwards=%d",
           route_seg_.size(), route_full_.size(),
           (!route_full_.empty() ? (route_full_.front().backwards ? 1 : 0) : 0));
}

bool NavigationController::isNearSegInit(double tol) const {
  if (route_seg_.empty()) return true;  // ou false, como preferires

  const WayPoint seg_init = route_seg_.front();

  const double dx = seg_init.pose.x - poseCurr.x;
  const double dy = seg_init.pose.y - poseCurr.y;
  const double dist = std::hypot(dx, dy);

  return dist <= tol;
}

std::pair<double, double> NavigationController::normalize(double vx, double vy) const {
    double length = std::hypot(vx, vy);
    if (length == 0.0) {
        return {0.0, 0.0};
    }
    return {vx / length, vy / length};
}

std::vector<Point> NavigationController::smoothPath(const std::vector<Point>& path_in,
                                                        double radius,
                                                        int corner_steps) const {
    if (path_in.size() < 2) {
        return path_in;
    }

    ROS_WARN("ESTOU A USAR ESTE smoothPath!!!");

    std::vector<Point> new_path;
    new_path.reserve(path_in.size() * corner_steps);
    new_path.push_back(path_in[0]);

    for (size_t i = 1; i + 1 < path_in.size(); ++i) {
        Point p_prev = path_in[i - 1];
        Point p_curr = path_in[i];
        Point p_next = path_in[i + 1];

        double v1x = p_curr.x - p_prev.x;
        double v1y = p_curr.y - p_prev.y;
        double v2x = p_next.x - p_curr.x;
        double v2y = p_next.y - p_curr.y;

        std::pair<double,double> n1 = normalize(v1x, v1y);
        std::pair<double,double> n2 = normalize(v2x, v2y);
        double d1x = n1.first;
        double d1y = n1.second;
        double d2x = n2.first;
        double d2y = n2.second;

        double dist1 = std::hypot(v1x, v1y);
        double dist2 = std::hypot(v2x, v2y);
        //double r = std::min({radius, dist1 * 2, dist2 * 2});
        double r = radius;

        Point before{
            p_curr.x - d1x * r,
            p_curr.y - d1y * r
        };
        Point after{
            p_curr.x + d2x * r,
            p_curr.y + d2y * r
        };

        new_path.push_back(before);

        for (int k = 1; k < corner_steps; ++k) {
            double t = static_cast<double>(k) / corner_steps;
            double omt = 1.0 - t;

            double bx = (omt * omt) * before.x
                        + 2.0 * omt * t * p_curr.x
                        + (t * t) * after.x;
            double by = (omt * omt) * before.y
                        + 2.0 * omt * t * p_curr.y
                        + (t * t) * after.y;
            new_path.push_back({bx, by});
        }

        new_path.push_back(after);
    }

    new_path.push_back(path_in.back());
    return new_path;
}