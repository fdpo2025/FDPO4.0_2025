#include "navigation_controller_node.h"
#include <cmath>

NavigationController::NavigationController(ros::NodeHandle& nh_) : nh(nh_), v_d(0.0), w_d(0.0), 
navigationFsm(navigation::states::idle), followLineFsm(navigation::followLineStates::GoTo_Init), 
k1(0.0), previousWaypoint({-1, {0, 0, 0}, false, false}), tfBuffer(), tfListener(tfBuffer) {
    
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
    lineMarkerPub = nh.advertise<visualization_msgs::Marker>("navigation_lines", 1, true);  // latch=true para RViz ver imediatamente
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
    previousWaypoint.id = -1;  // Reset previousWaypoint quando rota é carregada

    for(int i = 0; i < static_cast<int>(waypoints.size()); ++i){
        WayPoint waypoint_temp;
        waypoint_temp.id = i;

        waypoint_temp.pose.x = static_cast<double>(waypoints[i]["x"]);
        waypoint_temp.pose.y = static_cast<double>(waypoints[i]["y"]);
        waypoint_temp.pose.theta = static_cast<double>(waypoints[i]["yaw"]);
        waypoint_temp.align = static_cast<bool>(waypoints[i]["align"]);
        waypoint_temp.backwards = static_cast<bool>(waypoints[i]["backwards"]);
        ROS_INFO("Waypoint: x=%.2f y=%.2f yaw=%.2f", waypoint_temp.pose.x, waypoint_temp.pose.y, waypoint_temp.pose.theta);

        route.push_back(waypoint_temp);

    }

    updateDesiredPose();
    
    // Reinicializar followLine FSM quando rota é carregada
    if (route.size() >= 2) {
        followLineFsm.new_state = navigation::followLineStates::GoTo_Init;
        followLineFsm.set_state();
    }
    
    // Publicar linhas de visualização
    publishLineMarkers();

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
    
    // FollowLine parameters (from Pascal code)
    nh.param("gain_fwd", param.gain_fwd, 1.0);      // GAIN_FWD
    nh.param("vel_lin_nom", param.vel_lin_nom, 0.3);  // VEL_LIN_NOM
    nh.param("dist_da", param.dist_da, 0.3);       // DIST_DA
    nh.param("tol_findist", param.tol_findist, 0.05); // TOL_FINDIST
    nh.param("max_etf", param.max_etf, 0.2);       // MAX_ETF (rad)
    nh.param("tol_init_line", param.tol_init_line, 0.1); // Tolerância de distância à linha para GoTo_Init -> Follow_Line (m)
    
    ROS_INFO("NavigationController parameters loaded: v_nom=%.2f, w_nom=%.2f, k_line=%.2f, gain_fwd=%.2f, vel_lin_nom=%.2f", 
             param.v_nom, param.w_nom, param.k_line, param.gain_fwd, param.vel_lin_nom);

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

// ============================================================================
// FOLLOW LINE FUNCTIONS
// ============================================================================

// Find distance from estimated position to nearest point in a line
// Implementação exata do código Pascal
void NavigationController::dist2Line(double xi, double yi, double xf, double yf, double xr, double yr, double& distLine) {
    double ux, uy;
    
    // Vetor unitário da linha
    double dx = xf - xi;
    double dy = yf - yi;
    double line_length = std::sqrt(dx * dx + dy * dy);
    
    if (line_length < 1e-6) {
        distLine = std::sqrt((xr - xi) * (xr - xi) + (yr - yi) * (yr - yi));
        k1 = 0.0;
        return;
    }
    
    ux = dx / line_length;
    uy = dy / line_length;
    
    // Calcular k1 (distância perpendicular com sinal)
    k1 = (xr * uy - yr * ux - xi * uy + yi * ux) / (ux * ux + uy * uy);
    
    // distLine é o valor absoluto de k1
    distLine = std::abs(k1);
}

double NavigationController::getLineAngle(double pi_x, double pi_y, double pf_x, double pf_y) {
    // Calcula o ângulo da linha usando atan2(pf - pi)
    double dx = pf_x - pi_x;
    double dy = pf_y - pi_y;
    return std::atan2(dy, dx);
}

double NavigationController::getLineError() {
    // Retorna a distância perpendicular do robô à linha atual
    // A linha é definida pelo waypoint atual (pi) e o próximo waypoint (pf)
    
    if (route.empty()) return 0.0;
    
    // Se há apenas um waypoint, não há linha - retorna erro de posição
    if (route.size() < 2) {
        return getPositionError();
    }
    
    // pi = waypoint atual (route.front())
    // pf = próximo waypoint
    auto it = route.begin();
    double pi_x = it->pose.x;
    double pi_y = it->pose.y;
    
    ++it;
    double pf_x = it->pose.x;
    double pf_y = it->pose.y;
    
    // Calcular distância perpendicular à linha
    double distLine;
    dist2Line(pi_x, pi_y, pf_x, pf_y, poseCurr.x, poseCurr.y, distLine);
    return distLine;
}

double NavigationController::getAlignLineYawError() {
    // Retorna o erro de orientação em relação à linha atual
    // A linha é definida pelo waypoint atual (pi) e o próximo waypoint (pf)
    
    if (route.empty()) return 0.0;
    
    // Se há apenas um waypoint, usa o erro de alinhamento normal
    if (route.size() < 2) {
        return getAlignYawError();
    }
    
    // pi = waypoint atual (route.front())
    // pf = próximo waypoint
    auto it = route.begin();
    double pi_x = it->pose.x;
    double pi_y = it->pose.y;
    
    ++it;
    double pf_x = it->pose.x;
    double pf_y = it->pose.y;
    
    // Calcular ângulo da linha
    double line_angle = getLineAngle(pi_x, pi_y, pf_x, pf_y);
    
    // Se backwards, inverter direção
    if (isBackwards()) {
        line_angle = normalizeAngle(line_angle + M_PI);
    }
    
    // Erro de orientação = diferença entre ângulo da linha e orientação atual
    return normalizeAngle(line_angle - poseCurr.theta);
}

void NavigationController::publishLineMarkers() {
    // Publica markers para visualizar as linhas entre waypoints no RViz
    
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "navigation_lines";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::Marker::LINE_LIST;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    
    // Propriedades visuais
    line_marker.scale.x = 0.05;  // Espessura da linha (5 cm)
    line_marker.color.r = 0.0;
    line_marker.color.g = 1.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;  // Verde opaco
    
    line_marker.lifetime = ros::Duration(0);  // Sem expiração
    
    // Limpar pontos anteriores
    line_marker.points.clear();
    
    // Se não há waypoints ou há apenas um, publicar marker vazio
    if (route.empty() || route.size() < 2) {
        lineMarkerPub.publish(line_marker);
        return;
    }
    
    // Criar linhas entre waypoints consecutivos
    for (auto it = route.begin(); it != route.end(); ++it) {
        auto next_it = std::next(it);
        if (next_it == route.end()) break;
        
        // Ponto inicial (pi)
        geometry_msgs::Point p1;
        p1.x = it->pose.x;
        p1.y = it->pose.y;
        p1.z = 0.0;
        
        // Ponto final (pf)
        geometry_msgs::Point p2;
        p2.x = next_it->pose.x;
        p2.y = next_it->pose.y;
        p2.z = 0.0;
        
        // Adicionar ambos os pontos para criar a linha
        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);
    }
    
    // Publicar marker
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
    
    // Reinicializar followLine FSM quando nova rota é adicionada
    if (route.size() >= 2) {
        followLineFsm.new_state = navigation::followLineStates::GoTo_Init;
        followLineFsm.set_state();
    }
    
    // Publicar linhas de visualização
    publishLineMarkers();

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

void NavigationController::goToXY() {

    // Se não há waypoints, parar imediatamente
    if(route.empty()) {
        v_d = 0.0;
        w_d = 0.0;
        return;
    }

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
    //   APPLY MAXIMUM SPEED LIMIT
    // -----------------------
    if (v_target > param.v_max)
        v_target = param.v_max;

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
    //   ENSURE MAXIMUM SPEED LIMIT (after accel/decel)
    // -----------------------
    if (v_d > param.v_max)
        v_d = param.v_max;
    if (v_d < -param.v_max)
        v_d = -param.v_max;

    // -----------------------
    //   BACKWARDS SUPPORT
    // -----------------------
    if (isBackwards())
        v_d = -v_d;
}

// Developed function to follow a line
// Implementação exata do código Pascal
void NavigationController::followLine() {
    
    // no waypoints: stop
    if(route.empty()) {
        v_d = 0.0;
        w_d = 0.0;
        followLineFsm.new_state = navigation::followLineStates::Stop_line;
        followLineFsm.set_state();
        return;
    }
    
    // Se só resta 1 waypoint, ir diretamente para o waypoint final (não usar FSM do followLine)
    if(route.size() == 1) {
        double xf = route.front().pose.x;
        double yf = route.front().pose.y;
        double tf = route.front().pose.theta;
        
        double dist_to_final = std::sqrt((xf - poseCurr.x) * (xf - poseCurr.x) + 
                                         (yf - poseCurr.y) * (yf - poseCurr.y));
        double yaw_error_final = normalizeAngle(tf - poseCurr.theta);
        
        // Se já chegou ao último waypoint - remover e parar
        if (dist_to_final <= param.arrive_radius && std::fabs(yaw_error_final) <= param.yaw_tol) {
            ROS_INFO("followLine: Reached final waypoint (x=%.2f, y=%.2f), removing from route", xf, yf);
            route.pop_front();
            v_d = 0.0;
            w_d = 0.0;
            followLineFsm.new_state = navigation::followLineStates::Stop_line;
            followLineFsm.set_state();
            navigationFsm.new_state = navigation::states::idle;
            return;
        }
        
        // Se não chegou, ir diretamente para o waypoint final usando goToXY
        // (ignorar a FSM do followLine quando só resta 1 waypoint)
        Pose saved_pose = poseDesired;
        poseDesired.x = xf;
        poseDesired.y = yf;
        poseDesired.theta = tf;
        goToXY();
        poseDesired = saved_pose;
        return;
    }
    
    // Obter pontos inicial e final da linha
    double xi, yi, xf, yf, tf;
    
    if(route.size() >= 2) {
        // Caso normal: usar waypoint atual e próximo
        auto it = route.begin();
        xi = it->pose.x;
        yi = it->pose.y;
        ++it;
        xf = it->pose.x;
        yf = it->pose.y;
        tf = it->pose.theta;
    } else if(route.size() == 1) {
        // Quando só resta 1 waypoint: usar waypoint anterior como ponto inicial
        // Se previousWaypoint não foi inicializado (id < 0), usar posição atual
        if(previousWaypoint.id < 0) {
            // Usar posição atual como ponto inicial
            xi = poseCurr.x;
            yi = poseCurr.y;
            ROS_WARN("followLine: Only 1 waypoint left and previousWaypoint not set, using current position as line start");
        } else {
            // Usar waypoint anterior guardado
            xi = previousWaypoint.pose.x;
            yi = previousWaypoint.pose.y;
            ROS_INFO("followLine: Using previousWaypoint (id=%d, x=%.2f, y=%.2f) -> final waypoint (x=%.2f, y=%.2f)", 
                     previousWaypoint.id, xi, yi, route.front().pose.x, route.front().pose.y);
        }
        xf = route.front().pose.x;
        yf = route.front().pose.y;
        tf = route.front().pose.theta;
    } else {
        // Sem waypoints: parar
        v_d = 0.0;
        w_d = 0.0;
        return;
    }
    
    // Calcular erros
    double tr = std::atan2(yf - yi, xf - xi);  // ângulo da linha
    double error_ang = normalizeAngle(tr - poseCurr.theta);
    double error_dist = std::sqrt((xf - poseCurr.x) * (xf - poseCurr.x) + 
                                  (yf - poseCurr.y) * (yf - poseCurr.y));
    
    // Calcular distância à linha (também calcula k1)
    double distLine;
    dist2Line(xi, yi, xf, yf, poseCurr.x, poseCurr.y, distLine);
    
    // Update FSM
    followLineFsm.update_tis();
    
    // State machine - Transitions
    if (followLineFsm.state == navigation::followLineStates::GoTo_Init) {
        if (distLine < param.tol_init_line && std::abs(error_ang) < param.max_etf) {
            followLineFsm.new_state = navigation::followLineStates::Follow_Line;
            navigationFsm.new_state = navigation::states::idle;  // state := 0
        }
    }
    else if (followLineFsm.state == navigation::followLineStates::Follow_Line) {
        if (error_dist < param.dist_da) {
            followLineFsm.new_state = navigation::followLineStates::Approaching;
        }
    }
    else if (followLineFsm.state == navigation::followLineStates::Approaching) {
        if (error_dist < param.tol_findist) {
            followLineFsm.new_state = navigation::followLineStates::Final_Rot;
        }
    }
    else if (followLineFsm.state == navigation::followLineStates::Final_Rot) {
        if (navigationFsm.state == navigation::states::idle) {  // state = Stop
            followLineFsm.new_state = navigation::followLineStates::Stop_line;
        }
    }
    
    // Apply transitions
    followLineFsm.set_state();
    
    // State machine - Outputs
    if (followLineFsm.state == navigation::followLineStates::GoTo_Init) {
        // gotoXY(xi, yi, tr) - usar waypoint inicial com orientação da linha
        Pose saved_pose = poseDesired;
        poseDesired.x = xi;
        poseDesired.y = yi;
        poseDesired.theta = tr;
        goToXY();
        poseDesired = saved_pose;  // Restaurar
    }
    else if (followLineFsm.state == navigation::followLineStates::Follow_Line) {
        v_d = param.vel_lin_nom;
        w_d = param.gain_fwd * k1 + param.gain_fwd * error_ang;
        
        // Limitar velocidade angular
        if (w_d > param.w_nom) w_d = param.w_nom;
        else if (w_d < -param.w_nom) w_d = -param.w_nom;
    }
    else if (followLineFsm.state == navigation::followLineStates::Approaching) {
        // LinDeAccel - desaceleração linear
        double dt = 1.0 / param.loop_rate_hz;
        double v_target = param.vel_lin_nom * (error_dist / param.dist_da);
        if (v_target < param.v_min) v_target = param.v_min;
        
        if (v_target > v_d) {
            v_d += param.a_max * dt;
            if (v_d > v_target) v_d = v_target;
        } else {
            v_d -= param.d_max * dt;
            if (v_d < v_target) v_d = v_target;
        }
        
        w_d = param.gain_fwd * k1 + param.gain_fwd * error_ang;
        
        // Limitar velocidade angular
        if (w_d > param.w_nom) w_d = param.w_nom;
        else if (w_d < -param.w_nom) w_d = -param.w_nom;
    }
    else if (followLineFsm.state == navigation::followLineStates::Final_Rot) {
        // gotoXY(xf, yf, tf) - usar waypoint final
        Pose saved_pose = poseDesired;
        poseDesired.x = xf;
        poseDesired.y = yf;
        poseDesired.theta = tf;
        goToXY();
        
        // Se só resta 1 waypoint, verificar se chegou ao waypoint final
        if (route.size() == 1) {
            // Verificar se chegou ao waypoint final (xf, yf)
            double dist_to_final = std::sqrt((xf - poseCurr.x) * (xf - poseCurr.x) + 
                                             (yf - poseCurr.y) * (yf - poseCurr.y));
            double yaw_error_final = normalizeAngle(tf - poseCurr.theta);
            
            if (dist_to_final <= param.arrive_radius && std::fabs(yaw_error_final) <= param.yaw_tol) {
                // Chegou ao último waypoint - remover e parar
                ROS_INFO("followLine: Reached final waypoint (x=%.2f, y=%.2f), removing from route", xf, yf);
                route.pop_front();
                v_d = 0.0;
                w_d = 0.0;
                followLineFsm.new_state = navigation::followLineStates::Stop_line;
                followLineFsm.set_state();
                navigationFsm.new_state = navigation::states::idle;
                poseDesired = saved_pose;  // Restaurar
                return;
            }
        }
        
        poseDesired = saved_pose;  // Restaurar
    }
    else if (followLineFsm.state == navigation::followLineStates::Stop_line) {
        v_d = 0.0;
        w_d = 0.0;
    }
    
    // Backwards support
    if (isBackwards() && v_d > 0.0) {
        v_d = -v_d;
    }
}


void NavigationController::navigationFsmRunner(const ros::TimerEvent&) {

    // Update's
    navigationFsm.update_tis();
    bool enable = !(mode == "stop" || mode == "pause") && !route.empty();

    // Compute Transitions
    if(navigationFsm.state == navigation::states::idle && enable) {

        navigationFsm.new_state = navigation::states::driveToGoal;

    }

    else if(navigationFsm.state == navigation::states::driveToGoal && isPositionArrived() && route.front().align && enable) {

        navigationFsm.new_state = navigation::states::turnToFinalYaw;

    }

    else if(navigationFsm.state == navigation::states::driveToGoal && isPositionArrived() && !route.front().align && enable) {

        // Guardar waypoint anterior antes de remover
        if(!route.empty()) {
            previousWaypoint = route.front();
            ROS_INFO("Stored previousWaypoint (id=%d, x=%.2f, y=%.2f) before removing. Remaining waypoints: %zu", 
                     previousWaypoint.id, previousWaypoint.pose.x, previousWaypoint.pose.y, route.size() - 1);
        }
        
        route.pop_front();
        updateDesiredPose();
        
        // Reinicializar followLine FSM para nova linha
        followLineFsm.new_state = navigation::followLineStates::GoTo_Init;
        followLineFsm.set_state();
        
        // Se não há mais waypoints, ir direto para idle
        if(route.empty()) {
            navigationFsm.new_state = navigation::states::idle;
        } else {
            navigationFsm.new_state = navigation::states::done;
        }

    }

    else if (navigationFsm.state == navigation::states::turnToFinalYaw && enable && !isPositionArrived()) {

        navigationFsm.new_state = navigation::states::driveToGoal;

    }

    else if(navigationFsm.state == navigation::states::turnToFinalYaw && isYawDesired() && enable) {

        // Guardar waypoint anterior antes de remover
        if(!route.empty()) {
            previousWaypoint = route.front();
            ROS_INFO("Stored previousWaypoint (id=%d, x=%.2f, y=%.2f) before removing (turnToFinalYaw). Remaining waypoints: %zu", 
                     previousWaypoint.id, previousWaypoint.pose.x, previousWaypoint.pose.y, route.size() - 1);
        }
        
        route.pop_front();
        updateDesiredPose();
        
        // Reinicializar followLine FSM para nova linha
        followLineFsm.new_state = navigation::followLineStates::GoTo_Init;
        followLineFsm.set_state();
        
        // Se não há mais waypoints, ir direto para idle
        if(route.empty()) {
            navigationFsm.new_state = navigation::states::idle;
        } else {
            navigationFsm.new_state = navigation::states::done;
        }

    }

    else if(navigationFsm.state == navigation::states::done && enable) {

        navigationFsm.new_state = navigation::states::driveToGoal;

    }

    else if(navigationFsm.state == navigation::states::done && !enable) {

        navigationFsm.new_state = navigation::states::idle;

    }

    // Set states
    navigationFsm.set_state();

    // Compute Actions
    if(navigationFsm.state == navigation::states::driveToGoal && enable) followLine();
    else if(navigationFsm.state == navigation::states::turnToFinalYaw && enable) setTheta();
    else {
        // Parar se não há waypoints ou estado inválido
        v_d = 0.0;
        w_d = 0.0;
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

        res.success = true;  res.message = "started";
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
