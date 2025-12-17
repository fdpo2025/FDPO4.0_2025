#include "navigation_controller_ss_node.h"
#include <cmath>

// ============================================================================
// CONSTRUCTOR
// ============================================================================
NavigationControllerSS::NavigationControllerSS(ros::NodeHandle& nh_) 
    : nh(nh_), curr_x(0.0), curr_y(0.0), curr_theta(0.0), v_d(0.0), w_d(0.0),
      seg_idx(0), last_th(0.0) {
    
    // ========================================================================
    // SUBSCRIBE TO ODOMETRY TOPIC
    // ========================================================================
    std::string odom_topic;
    nh.param("odom_topic", odom_topic, std::string("/odometry/filtered"));
    odomSub = nh.subscribe(odom_topic, 10, &NavigationControllerSS::odomCallback, this);
    ROS_INFO("NavigationControllerSS subscribing to: %s", odom_topic.c_str());

    // ========================================================================
    // SUBSCRIBE TO RVIZ GOAL TOPIC
    // ========================================================================
    rvizGoalSub = nh.subscribe("/move_base_simple/goal", 10, 
                                &NavigationControllerSS::rvizGoalCallback, this);
    ROS_INFO("NavigationControllerSS subscribing to: /move_base_simple/goal");

    // ========================================================================
    // PUBLISH TO VELOCITY TOPIC
    // ========================================================================
    velPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ROS_INFO("NavigationControllerSS publishing to: /cmd_vel");

    // ========================================================================
    // LOAD CONTROLLER PARAMETERS
    // ========================================================================
    loadControllerParams();

    // ========================================================================
    // TIMER FOR CONTROL LOOP
    // ========================================================================
    int loop_rate_hz;
    nh.param("loop_rate_hz", loop_rate_hz, 30);
    controlTimer = nh.createTimer(ros::Duration(1.0 / loop_rate_hz), 
                                   &NavigationControllerSS::controlLoop, this);

    // ========================================================================
    // LOAD PATH FROM PARAMETERS
    // ========================================================================
    loadPathFromParameters();

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
// RVIZ GOAL CALLBACK
// ============================================================================
void NavigationControllerSS::rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::vector<Point> waypoints;
    
    // Se já existe um caminho, adiciona o novo goal ao final
    if (!path.empty()) {
        waypoints = path;
    }
    
    // Adiciona o novo goal
    Point goal;
    goal.x = msg->pose.position.x;
    goal.y = msg->pose.position.y;
    waypoints.push_back(goal);
    
    ROS_INFO("RViz goal received: x=%.2f y=%.2f", goal.x, goal.y);
    
    updatePathFromWaypoints(waypoints);
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
// FUNÇÕES AUXILIARES
// ============================================================================
std::pair<double, double> NavigationControllerSS::normalize(double vx, double vy) const {
    double length = std::hypot(vx, vy);
    if (length == 0.0) {
        return {0.0, 0.0};
    }
    return {vx / length, vy / length};
}

std::vector<Point> NavigationControllerSS::smoothPath(const std::vector<Point>& path_in,
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

// ============================================================================
// LOAD CONTROLLER PARAMETERS
// ============================================================================
void NavigationControllerSS::loadControllerParams() {
    // Valores padrão
    params.kx = 1.0;
    params.ky = 50.0;
    params.kth = 5.0;
    params.v_max = 0.4;
    params.w_max = 3.0;
    params.v_ref = 0.2;
    params.end_dist_tol = 0.05;
    params.smooth_radius = 0.01;
    params.smooth_corner_steps = 8;
    
    // Carregar do ROS parameter server
    nh.param("kx", params.kx, params.kx);
    nh.param("ky", params.ky, params.ky);
    nh.param("kth", params.kth, params.kth);
    nh.param("v_max", params.v_max, params.v_max);
    nh.param("w_max", params.w_max, params.w_max);
    nh.param("v_ref", params.v_ref, params.v_ref);
    nh.param("end_dist_tol", params.end_dist_tol, params.end_dist_tol);
    nh.param("smooth_radius", params.smooth_radius, params.smooth_radius);
    nh.param("smooth_corner_steps", params.smooth_corner_steps, params.smooth_corner_steps);
    
    ROS_INFO("NavigationControllerSS parameters loaded: kx=%.2f, ky=%.2f, kth=%.2f, v_max=%.2f, w_max=%.2f, v_ref=%.2f, end_dist_tol=%.3f",
             params.kx, params.ky, params.kth, params.v_max, params.w_max, params.v_ref, params.end_dist_tol);
}

// ============================================================================
// LOAD PATH FROM PARAMETERS
// ============================================================================
void NavigationControllerSS::loadPathFromParameters() {
    XmlRpc::XmlRpcValue waypoints;
    if (!nh.getParam("waypoints", waypoints)) {
        ROS_WARN("NavigationControllerSS: No waypoints parameter found. Waiting for RViz goal or waypoints parameter.");
        return;
    }

    std::vector<Point> waypoint_list;
    for (int i = 0; i < static_cast<int>(waypoints.size()); ++i) {
        Point wp;
        wp.x = static_cast<double>(waypoints[i]["x"]);
        wp.y = static_cast<double>(waypoints[i]["y"]);
        waypoint_list.push_back(wp);
        ROS_INFO("NavigationControllerSS: Waypoint %d: x=%.2f y=%.2f", i, wp.x, wp.y);
    }

    updatePathFromWaypoints(waypoint_list);
}

// ============================================================================
// UPDATE PATH FROM WAYPOINTS
// ============================================================================
void NavigationControllerSS::updatePathFromWaypoints(const std::vector<Point>& waypoints) {
    if (waypoints.empty()) {
        ROS_WARN("NavigationControllerSS: Empty waypoints list");
        return;
    }

    // Atualiza o caminho base
    path = waypoints;
    
    // Gera o caminho suavizado
    smooth = smoothPath(path, params.smooth_radius, params.smooth_corner_steps);

    ROS_INFO("smooth size = %zu, path size = %zu", smooth.size(), path.size());
    ROS_WARN("EFFECTIVE: smooth_radius=%.3f corner_steps=%d",
         params.smooth_radius, params.smooth_corner_steps);
    ROS_WARN("OLAAAAAAAAAAAA");

    
    // Reinicia o índice do segmento
    seg_idx = 0;
    last_th = 0.0;
    
    ROS_INFO("NavigationControllerSS: Path updated with %zu waypoints, smoothed to %zu points", 
             path.size(), smooth.size());
}

// ============================================================================
// computeRef: calcula referência no caminho (mesma lógica do C++ de simulação)
// ============================================================================
RefState NavigationControllerSS::computeRef(double x, double y, double theta,
                                            const std::vector<Point>& path_in,
                                            int seg_idx_in)
{
    // Acede aos parâmetros via membro da classe
    double v_ref = params.v_ref;
    
    int N = static_cast<int>(path_in.size());

    if (seg_idx_in >= N - 1) {
        Point last = path_in.back();
        // Usa o último theta conhecido ou calcula a partir da posição atual
        double theta_final = (N >= 2) ? std::atan2(last.y - path_in[N-2].y, 
                                                     last.x - path_in[N-2].x) : last_th;
        return { last.x, last.y, theta_final, 0.0, 0.0, seg_idx_in };
    }

    Point A = path_in[seg_idx_in];
    Point B = path_in[seg_idx_in + 1];

    double ABx = B.x - A.x;
    double ABy = B.y - A.y;
    double APx = x - A.x;
    double APy = y - A.y;

    double AB2 = ABx*ABx + ABy*ABy;
    if (AB2 < 1e-6) {
        // Segmento degenerado: tenta o seguinte
        return computeRef(x, y, theta, path_in, seg_idx_in + 1);
    }

    // projeção escalar
    double s = (APx * ABx + APy * ABy) / AB2;

    // passou do fim? avança segmento
    if (s > 1.0) {
        return computeRef(x, y, theta, path_in, seg_idx_in + 1);
    }

    // clamp [0,1]
    if (s < 0.0) s = 0.0;
    if (s > 1.0) s = 1.0;

    double xr = A.x + s * ABx;
    double yr = A.y + s * ABy;
    double theta_r = std::atan2(ABy, ABx);

    return { xr, yr, theta_r, v_ref, 0.0, seg_idx_in };
}


// ============================================================================
// ============================================================================
// AREA FOR STATE SPACE LOGIC IMPLEMENTATION
// ============================================================================
// ============================================================================
void NavigationControllerSS::computeStateSpaceControl() {
    
    // ========================================================================
    // VERIFICAR SE HÁ CAMINHO DISPONÍVEL
    // ========================================================================
    if (smooth.empty()) {
        // Sem caminho → pára
        v_d = 0.0;
        w_d = 0.0;
        return;
    }

    // ========================================================================
    // VERIFICAR SE JÁ ESTAMOS NO FIM DO CAMINHO
    // ========================================================================
    const Point& goal = smooth.back();
    double dx_goal = goal.x - curr_x;
    double dy_goal = goal.y - curr_y;
    double dist_goal = std::hypot(dx_goal, dy_goal);

    if (dist_goal < params.end_dist_tol && seg_idx >= static_cast<int>(smooth.size()) - 1) {
        // Dentro da tolerância → pára
        v_d = 0.0;
        w_d = 0.0;
        return;
    }

    // ========================================================================
    // CALCULAR REFERÊNCIA NO CAMINHO SUAVIZADO
    // ========================================================================
    RefState ref = computeRef(curr_x, curr_y, curr_theta, smooth, seg_idx);

    seg_idx = ref.seg_idx;
    last_th = ref.theta_r;

    double xr      = ref.xr;
    double yr      = ref.yr;
    double theta_r = ref.theta_r;
    double v_r     = ref.v_r;
    double w_r     = ref.w_r;

    double theta = curr_theta;

    // ========================================================================
    // CALCULAR ERRO NO REFERENCIAL DO ROBÔ
    // ========================================================================
    double dx = xr - curr_x;
    double dy = yr - curr_y;

    double ex =  std::cos(theta) * dx + std::sin(theta) * dy;
    double ey = -std::sin(theta) * dx + std::cos(theta) * dy;

    double e_theta = theta_r - theta;
    // Normalizar para [-pi, pi)
    e_theta = std::atan2(std::sin(e_theta), std::cos(e_theta));

    // ========================================================================
    // APLICAR LEI DE CONTROLO DE ESPAÇO DE ESTADOS
    // ========================================================================
    double v = v_r * std::cos(e_theta) + params.kx * ex;
    double w = w_r + params.ky * v_r * ey + params.kth * std::sin(e_theta);

    // ========================================================================
    // SATURAÇÃO DE VELOCIDADES
    // ========================================================================
    if (v >  params.v_max) v =  params.v_max;
    if (v < -params.v_max) v = -params.v_max;
    if (w >  params.w_max) w =  params.w_max;
    if (w < -params.w_max) w = -params.w_max;

    // ========================================================================
    // GUARDAR COMO VELOCIDADES DESEJADAS
    // ========================================================================
    v_d = v;
    w_d = w;
}

