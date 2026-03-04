#include "navigation_controller_node.h"
#include <cmath>

NavigationController::NavigationController(ros::NodeHandle& nh_) : nh(nh_), v_d(0.0), w_d(0.0), 
navigationFsm(navigation::states::idle), followLineFsm(navigation::followLineStates::Follow_Line), 
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

void NavigationController::loadRouteFromParameters(){

    XmlRpc::XmlRpcValue waypoints;
    if(!nh.getParam("waypoints", waypoints)) return;

    route.clear();
    
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
    
    // Reinicializar followLine FSM quando rota é carregada
    // Agora previousWaypoint (posição inicial) + route.front() = 1 linha válida
    if (!route.empty()) {
        followLineFsm.new_state = navigation::followLineStates::Follow_Line;
        followLineFsm.set_state();
        
        // Verificar se já estamos perto do primeiro waypoint (skip inicial)
        // Se line_progress >= switch_ratio, avançar para a próxima linha
        skipNearbyWaypoints();
    }
    
    // Reset completion feedback flag quando nova rota é carregada
    completion_feedback_sent = false;
    
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
    nh.param("line_switch_ratio", param.line_switch_ratio, 0.9); // Ratio da linha para mudar para próxima (0.9 = 90%)
    nh.param("approaching_line_progress", param.approaching_line_progress, 0.60); // Progresso da linha para entrar em Approaching (0.60 = 60%)
    nh.param("approaching_vel", param.approaching_vel, 0.05); // Velocidade constante no estado Approaching (m/s)
    
    ROS_INFO("NavigationController parameters loaded: v_nom=%.2f, w_nom=%.2f, k_line=%.2f, line_switch_ratio=%.2f", 
             param.v_nom, param.w_nom, param.k_line, param.line_switch_ratio);

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
// Também calcula line_progress (0 = em pi, 1 = em pf)
void NavigationController::dist2Line(double xi, double yi, double xf, double yf, double xr, double yr, double& distLine) {
    double ux, uy;
    
    // Vetor unitário da linha
    double dx = xf - xi;
    double dy = yf - yi;
    double line_length = std::sqrt(dx * dx + dy * dy);
    
    if (line_length < 1e-6) {
        distLine = std::sqrt((xr - xi) * (xr - xi) + (yr - yi) * (yr - yi));
        k1 = 0.0;
        line_progress = 1.0;  // Linha tem comprimento 0, considerar como chegado
        return;
    }
    
    ux = dx / line_length;
    uy = dy / line_length;
    
    // Calcular k1 (distância perpendicular com sinal)
    k1 = (xr * uy - yr * ux - xi * uy + yi * ux) / (ux * ux + uy * uy);
    
    // distLine é o valor absoluto de k1
    distLine = std::abs(k1);
    
    // Calcular line_progress: projeção do robô ao longo da linha (0 a 1)
    // t = dot(robot - pi, linha_unitaria) / comprimento_linha
    double t = ((xr - xi) * ux + (yr - yi) * uy);
    line_progress = t / line_length;  // 0 = em pi, 1 = em pf, >1 = passou pf
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
    // Verde = forward, Azul = backwards
    
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
    // Cor default não é usada quando colors está preenchido
    line_marker.color.a = 1.0;
    
    line_marker.lifetime = ros::Duration(0);  // Sem expiração
    
    // Limpar pontos e cores anteriores
    line_marker.points.clear();
    line_marker.colors.clear();
    
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
        
        // Cor da linha baseada no backwards do pf (next_it)
        // Verde = forward, Azul = backwards
        std_msgs::ColorRGBA line_color;
        if (next_it->backwards) {
            // Azul para backwards
            line_color.r = 0.0;
            line_color.g = 0.5;
            line_color.b = 1.0;
            line_color.a = 1.0;
        } else {
            // Verde para forward
            line_color.r = 0.0;
            line_color.g = 1.0;
            line_color.b = 0.0;
            line_color.a = 1.0;
        }
        
        // Adicionar pontos e cores (2 cores por linha, uma para cada vértice)
        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);
        line_marker.colors.push_back(line_color);
        line_marker.colors.push_back(line_color);
    }
    
    // Publicar marker
    lineMarkerPub.publish(line_marker);
}

void NavigationController::skipNearbyWaypoints() {
    // Se já estamos perto de waypoints (align=false), skip para a próxima linha
    // Isto evita criar linhas muito curtas quando o robô já está perto do waypoint inicial
    
    while (route.size() >= 2 && !route.front().align) {
        // Calcular line_progress para a linha atual
        double pi_x = previousWaypoint.pose.x;
        double pi_y = previousWaypoint.pose.y;
        double pf_x = route.front().pose.x;
        double pf_y = route.front().pose.y;
        
        double line_length = std::sqrt((pf_x - pi_x) * (pf_x - pi_x) + (pf_y - pi_y) * (pf_y - pi_y));
        
        if (line_length < 0.01) {
            // Linha muito curta, skip direto
            previousWaypoint = route.front();
            route.pop_front();
            ROS_INFO("Skipped waypoint (line too short): id=%d", previousWaypoint.id);
            continue;
        }
        
        // Vetor da linha (normalizado)
        double line_dx = (pf_x - pi_x) / line_length;
        double line_dy = (pf_y - pi_y) / line_length;
        
        // Vetor do ponto inicial até posição atual
        double robot_dx = poseCurr.x - pi_x;
        double robot_dy = poseCurr.y - pi_y;
        
        // Projeção da posição do robô na linha
        double projection = robot_dx * line_dx + robot_dy * line_dy;
        double progress = projection / line_length;
        
        // Obter switch_ratio para este waypoint
        double switch_ratio = (route.front().line_switch_ratio > 0) ? 
                               route.front().line_switch_ratio : param.line_switch_ratio;
        
        if (progress >= switch_ratio) {
            // Já estamos além do switch_ratio, avançar para próxima linha
            previousWaypoint = route.front();
            route.pop_front();
            ROS_INFO("Skipped nearby waypoint: id=%d (progress=%.0f%% >= threshold=%.0f%%)", 
                     previousWaypoint.id, progress * 100, switch_ratio * 100);
        } else {
            // Ainda não chegámos ao threshold, parar de skipar
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
        route.clear();
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

    waypoint_temp.id = route.empty() ? 1 : (route.back().id + 1);  // IDs começam em 1 (0 é previousWaypoint)
    waypoint_temp.pose.x = poseInMap.pose.position.x;
    waypoint_temp.pose.y = poseInMap.pose.position.y;
    waypoint_temp.pose.theta = tf2::getYaw(poseInMap.pose.orientation);
    waypoint_temp.align = true;
    waypoint_temp.backwards = false;
    waypoint_temp.line_switch_ratio = -1.0;  // Usar parâmetro global
    waypoint_temp.vel_lin_nom = -1.0;        // Usar parâmetro global

    route.push_back(waypoint_temp);
    ROS_INFO("RViz goal added: x=%.2f y=%.2f yaw=%.2f (map frame)", 
             waypoint_temp.pose.x, waypoint_temp.pose.y, waypoint_temp.pose.theta);

    updateDesiredPose();
    
    // Reinicializar followLine FSM quando nova rota é adicionada
    followLineFsm.new_state = navigation::followLineStates::Follow_Line;
    followLineFsm.set_state();
    
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
void NavigationController::followLine() {
    
    // no waypoints: stop
    if(route.empty()) {
        v_d = 0.0;
        w_d = 0.0;
        return;
    }

    // Line extraction from route
    // Linha: de previousWaypoint (pi) para route.front() (pf) - o objetivo atual
    Line line;
    
    // Ponto final é sempre o waypoint atual (route.front())
    currentWaypoint = route.front();
    line.pf = currentWaypoint;
    line.pi = previousWaypoint;
    
    // Velocidade linear nominal efetiva (do waypoint se definida, senão global)
    double vel_lin_nom_eff = (currentWaypoint.vel_lin_nom > 0) ? 
                              currentWaypoint.vel_lin_nom : param.vel_lin_nom;
    
    // error calc.
    double tr = std::atan2(line.pf.pose.y - line.pi.pose.y, line.pf.pose.x - line.pi.pose.x);  // Line Angle
    // Se backwards, seguir a linha no sentido oposto
    if (isBackwards()) {
        tr = normalizeAngle(tr + M_PI);
    }

    double error_ang = normalizeAngle(tr - poseCurr.theta);
    double error_dist = std::sqrt((line.pf.pose.x - poseCurr.x) * (line.pf.pose.x - poseCurr.x) + 
                                  (line.pf.pose.y - poseCurr.y) * (line.pf.pose.y - poseCurr.y));
    
    // Converter erro angular para valor absoluto em graus para verificação
    double error_ang_deg = std::abs(error_ang) * 180.0 / M_PI;

    // Line dist
    double distLine;
    dist2Line(line.pi.pose.x, line.pi.pose.y, line.pf.pose.x, line.pf.pose.y, poseCurr.x, poseCurr.y, distLine);
    
    double k1_eff = k1;
    if (isBackwards()) {
        k1_eff = k1;
    }

    // // Reduzir velocidade nominal para metade se erro de linha > 3cm
    // const double line_error_threshold = 0.03; // 3cm
    // if (std::abs(k1) > line_error_threshold) {
    //     vel_lin_nom_eff = vel_lin_nom_eff * 0.5;
    //     ROS_INFO_THROTTLE(1.0, "NavigationController: Line error %.3fm > %.3fm, reducing velocity to %.3f m/s", 
    //                      std::abs(k1), line_error_threshold, vel_lin_nom_eff);
    // }

    // Update tis
    followLineFsm.update_tis();
    
    // State machine - Transitions
    // Apenas 2 estados: Follow_Line e Approaching
    // Vai para Approaching quando tiver percorrido approaching_line_progress da linha E
    // (o ponto final for uma warehouse OU se estiver saindo de uma warehouse (backwards=true))
    if (followLineFsm.state == navigation::followLineStates::Follow_Line) {
        if (error_dist < param.dist_da && (line.pf.is_warehouse || isBackwards())) {
            followLineFsm.new_state = navigation::followLineStates::Approaching;
        }
    }
    
    // Apply transitions
    followLineFsm.set_state();
    
    // Publicar feedback de conclusão quando >70% da linha for completada (apenas uma vez por linha)
    if (line_progress > 0.7 && !completion_feedback_sent) {
        plan_handler::CompletionFeedback feedback;
        feedback.x = line.pf.pose.x;
        feedback.y = line.pf.pose.y;
        navCompletionFeedbackPub.publish(feedback);
        completion_feedback_sent = true;
        ROS_INFO("NavigationController: Published completion feedback for waypoint (%.3f, %.3f) at %.1f%% progress", 
                 feedback.x, feedback.y, line_progress * 100.0);
    }
    
    // Reset flag quando mudar de linha (quando line_progress < 0.7 novamente ou quando avançar para próximo waypoint)
    if (line_progress < 0.7) {
        completion_feedback_sent = false;
    }

    // State machine - Outputs
    if (followLineFsm.state == navigation::followLineStates::Follow_Line) {
        

        w_d = param.k_line * k1_eff + param.gain_fwd * error_ang;

        double A = -vel_lin_nom_eff/(param.w_nom*param.w_nom);
        v_d = std::max(A * (w_d - param.w_nom) * (w_d + param.w_nom), 0.0);
        
        // Limitar velocidade angular
        if (w_d > param.w_nom) w_d = param.w_nom;
        else if (w_d < -param.w_nom) w_d = -param.w_nom;
        
    }
    else if (followLineFsm.state == navigation::followLineStates::Approaching) {

        // -----------------------
        //   ANGULAR CONTROL
        // -----------------------
        w_d = param.k_line * k1_eff + param.gain_fwd * error_ang;

        // Limitar velocidade angular
        if (w_d > param.w_nom)       w_d = param.w_nom;
        else if (w_d < -param.w_nom) w_d = -param.w_nom;

        // -----------------------
        //   LINEAR CONTROL 
        // -----------------------
        // Velocidade constante no estado Approaching
        v_d = param.approaching_vel;
    }

    // Zerar velocidade linear se erro angular > 93°
    if (error_ang_deg > 93.0) {
        v_d = 0.0;
        ROS_INFO_THROTTLE(1.0, "NavigationController: Angular error %.1f° > 93°, setting linear velocity to 0", error_ang_deg);
    }
    
    // Backwards support
    if (isBackwards() && v_d > 0.0) {
        v_d = -v_d;
    }

    // DEBUG
    ROS_INFO("[FOLLOW_LINE] Line: (%.2f,%.2f)->(%.2f,%.2f) | progress=%.0f%% | dist=%.3f | state=%s", 
             line.pi.pose.x, line.pi.pose.y, line.pf.pose.x, line.pf.pose.y, 
             line_progress * 100, error_dist,
             (followLineFsm.state == navigation::followLineStates::Follow_Line) ? "Follow_Line" : "Approaching");

}


void NavigationController::navigationFsmRunner(const ros::TimerEvent&) {

    // Update's
    navigationFsm.update_tis();
    bool enable = !(mode == "stop" || mode == "pause") && !route.empty();

    // Compute Transitions
    if(navigationFsm.state == navigation::states::idle && enable) {

        navigationFsm.new_state = navigation::states::driveToGoal;

    }

    // Quando align=true: usar isPositionArrived() para parar no ponto exato e rodar
    else if(navigationFsm.state == navigation::states::driveToGoal && isPositionArrived() && route.front().align && enable) {
        
        // Guardar waypoint anterior antes de remover
        previousWaypoint = route.front();
        
        // Se é warehouse de pick, entrar no estado pickBoxForward
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

    // Quando align=false: usar line_progress para mudar de linha mais cedo (suaviza transições)
    // Usar line_switch_ratio do waypoint se definido (>0), senão usar parâmetro global
    else if(navigationFsm.state == navigation::states::driveToGoal && !route.front().align && enable) {
        
        double switch_ratio = (route.front().line_switch_ratio > 0) ? 
                               route.front().line_switch_ratio : param.line_switch_ratio;
        
        if (line_progress >= switch_ratio) {
            // Guardar waypoint anterior antes de remover
            previousWaypoint = route.front();
            
            // Verificar se é warehouse de pick antes de remover
            bool is_pick_warehouse = route.front().pick_box;
            
            ROS_INFO("Line switch at %.0f%% (threshold=%.0f%%): waypoint (id=%d, x=%.2f, y=%.2f, pick_box=%d, backwards=%d). Remaining: %zu", 
                     line_progress * 100, switch_ratio * 100,
                     previousWaypoint.id, previousWaypoint.pose.x, previousWaypoint.pose.y, is_pick_warehouse ? 1 : 0, 
                     previousWaypoint.backwards ? 1 : 0, route.size() - 1);
        
            // Se é warehouse de pick, NÃO remover ainda - será removido quando sair do estado pickBoxForward
            if (is_pick_warehouse) {
                navigationFsm.new_state = navigation::states::pickBoxForward;
                pick_box_forward_start_time = ros::Time::now();
                in_pick_box_forward = true;
                ROS_INFO("NavigationController: Completed line to pick warehouse, entering pickBoxForward state");
            } else {
                // Não é warehouse de pick, remover normalmente
                route.pop_front();
                updateDesiredPose();
                
                // Reinicializar followLine FSM para nova linha
                followLineFsm.new_state = navigation::followLineStates::Follow_Line;
                followLineFsm.set_state();
                
                // Reset completion feedback flag para nova linha
                completion_feedback_sent = false;
                
                // Se não há mais waypoints, ir direto para idle
                if(route.empty()) {
                    navigationFsm.new_state = navigation::states::idle;
                } else {
                    navigationFsm.new_state = navigation::states::done;
                }
            }
        }
    }

    else if (navigationFsm.state == navigation::states::turnToFinalYaw && enable && !isPositionArrived()) {

        navigationFsm.new_state = navigation::states::driveToGoal;

    }

    else if(navigationFsm.state == navigation::states::turnToFinalYaw && isYawDesired() && enable) {

        // Guardar waypoint anterior antes de remover
        bool is_pick_warehouse = false;
        if(!route.empty()) {
            previousWaypoint = route.front();
            is_pick_warehouse = route.front().pick_box;
            ROS_INFO("Stored previousWaypoint (id=%d, x=%.2f, y=%.2f, pick_box=%d) before removing (turnToFinalYaw). Remaining waypoints: %zu", 
                     previousWaypoint.id, previousWaypoint.pose.x, previousWaypoint.pose.y, is_pick_warehouse ? 1 : 0, route.size() - 1);
        }
        
        // Se é warehouse de pick, NÃO remover ainda - será removido quando sair do estado pickBoxForward
        if (is_pick_warehouse) {
            navigationFsm.new_state = navigation::states::pickBoxForward;
            pick_box_forward_start_time = ros::Time::now();
            in_pick_box_forward = true;
            ROS_INFO("NavigationController: Completed turnToFinalYaw at pick warehouse, entering pickBoxForward state");
        } else {
            // Não é warehouse de pick, remover normalmente
            route.pop_front();
            // Reset completion feedback flag para nova linha
            completion_feedback_sent = false;
            updateDesiredPose();
            
            // Reinicializar followLine FSM para nova linha
            followLineFsm.new_state = navigation::followLineStates::Follow_Line;
            followLineFsm.set_state();
            
            // Se não há mais waypoints, ir direto para idle
            if(route.empty()) {
                navigationFsm.new_state = navigation::states::idle;
            } else {
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
        
        if (elapsed_time >= 0.7) {
            // Passou 2 segundos, remover waypoint e continuar para próximo
            in_pick_box_forward = false;
            if (!route.empty()) {
                route.pop_front();
                updateDesiredPose();
                // Reinicializar followLine FSM para nova linha
                followLineFsm.new_state = navigation::followLineStates::Follow_Line;
                followLineFsm.set_state();
                // Reset completion feedback flag para nova linha
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

    // Set states
    navigationFsm.set_state();

    // Compute Actions
    if(navigationFsm.state == navigation::states::driveToGoal && enable) followLine();
    else if(navigationFsm.state == navigation::states::turnToFinalYaw && enable) setTheta();
    else if(navigationFsm.state == navigation::states::pickBoxForward && enable) {
        // Andar para frente com velocidade linear 0.1 m/s durante 2s
        // Se estava indo backwards, andar para trás
        v_d = previousWaypoint.backwards ? -0.1 : 0.1;
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

        // Só carregar route do YAML se load_from_route for true
        if (load_from_route) {
            loadRouteFromParameters();
            
            if (route.empty()) {
                res.success = false; res.message = "no waypoints in params";
                return true;
            }
        } else {
            // Se não carregar do route, esperar por /nav_plan
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

void NavigationController::loadRouteFromNavPlan(const plan_handler::NavPlan::ConstPtr& msg) {
    if (msg->points.empty()) {
        ROS_WARN("NavigationController: Received empty NavPlan, ignoring");
        return;
    }

    route.clear();
    
    // Inicializar previousWaypoint com posição atual do robô
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
    ROS_INFO("Initial position set as previousWaypoint: x=%.2f y=%.2f", poseCurr.x, poseCurr.y);

    // Converter NavPlan points para WayPoints
    for (size_t i = 0; i < msg->points.size(); ++i) {
        const plan_handler::ControllerPoint& cp = msg->points[i];
        WayPoint waypoint_temp;
        waypoint_temp.id = static_cast<int>(i);
        
        waypoint_temp.pose.x = cp.x;
        waypoint_temp.pose.y = cp.y;
        // Calcular theta baseado na direção para o próximo ponto (ou usar 0 se for o último)
        if (i < msg->points.size() - 1) {
            double dx = msg->points[i + 1].x - cp.x;
            double dy = msg->points[i + 1].y - cp.y;
            waypoint_temp.pose.theta = std::atan2(dy, dx);
        } else {
            // Último ponto: manter direção do penúltimo ou usar theta atual
            if (i > 0) {
                double dx = cp.x - msg->points[i - 1].x;
                double dy = cp.y - msg->points[i - 1].y;
                waypoint_temp.pose.theta = std::atan2(dy, dx);
            } else {
                waypoint_temp.pose.theta = poseCurr.theta;
            }
        }
        
        waypoint_temp.align = false;  // NavPlan não tem campo align, usar false
        waypoint_temp.backwards = cp.backwards;
        waypoint_temp.pick_box = cp.pick_box;  // Copiar pick_box do NavPlan
        waypoint_temp.is_warehouse = cp.is_warehouse;  // Copiar is_warehouse do NavPlan
        
        // Usar valores do NavPlan se definidos, senão usar -1 (global)
        waypoint_temp.line_switch_ratio = (cp.line_switch_ratio > 0) ? cp.line_switch_ratio : -1.0;
        waypoint_temp.vel_lin_nom = (cp.vel_lin_nom > 0) ? cp.vel_lin_nom : -1.0;
        
        double effective_vel = waypoint_temp.vel_lin_nom > 0 ? waypoint_temp.vel_lin_nom : param.vel_lin_nom;
        ROS_INFO("Waypoint %d: x=%.2f y=%.2f yaw=%.2f backwards=%d switch=%.0f%% vel=%.2f", 
                 waypoint_temp.id, waypoint_temp.pose.x, waypoint_temp.pose.y, waypoint_temp.pose.theta,
                 waypoint_temp.backwards, 
                 waypoint_temp.line_switch_ratio > 0 ? waypoint_temp.line_switch_ratio * 100 : param.line_switch_ratio * 100,
                 effective_vel);

        route.push_back(waypoint_temp);
    }

    updateDesiredPose();
    
    // Reinicializar followLine FSM quando rota é carregada
    if (!route.empty()) {
        followLineFsm.new_state = navigation::followLineStates::Follow_Line;
        followLineFsm.set_state();
        
        // Verificar se já estamos perto do primeiro waypoint (skip inicial)
        skipNearbyWaypoints();
    }
    
    // Publicar linhas de visualização
    publishLineMarkers();
    
    // Reset completion feedback flag quando nova rota é carregada
    completion_feedback_sent = false;
    
    ROS_INFO("NavigationController: Route loaded from NavPlan with %zu waypoints", route.size());
}
