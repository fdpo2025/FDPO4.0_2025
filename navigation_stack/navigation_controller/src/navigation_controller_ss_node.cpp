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

// ----------------------------------
// Funções auxiliares
// ----------------------------------
std::pair<double, double> normalize(double vx, double vy) {
    double length = std::hypot(vx, vy);
    if (length == 0.0) {
        return {0.0, 0.0};
    }
    return {vx / length, vy / length};
}

std::vector<Point> smooth_path(const std::vector<Point>& path,
                               double radius = 0.3,
                               int corner_steps = 8)
{
    if (path.size() < 3) {
        return path;
    }

    std::vector<Point> new_path;
    new_path.reserve(path.size() * corner_steps); // reserva aproximada
    new_path.push_back(path[0]);

    for (size_t i = 1; i + 1 < path.size(); ++i) {
        Point p_prev = path[i - 1];
        Point p_curr = path[i];
        Point p_next = path[i + 1];

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
        double r = std::min({radius, dist1 * 0.5, dist2 * 0.5});

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

    new_path.push_back(path.back());
    return new_path;
}

// ============================================================================
// computeRef: calcula referência no caminho (mesma lógica do C++ de simulação)
// ============================================================================
RefState NavigationControllerSS::computeRef(double x, double y, double theta,
                                            const std::vector<Point>& path_in,
                                            int seg_idx_in)
{
    int N = static_cast<int>(path_in.size());

    if (seg_idx_in >= N - 1) {
        Point last = path_in.back();
        return { last.x, last.y, last_th, 0.0, 0.0, seg_idx_in };
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

    return { xr, yr, theta_r, V_REF, 0.0, seg_idx_in };
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

    // 1) Verificar se já estamos no fim do caminho
    const Point& goal = smooth.back();
    double dx_goal = goal.x - curr_x;
    double dy_goal = goal.y - curr_y;
    double dist_goal = std::hypot(dx_goal, dy_goal);

    if (dist_goal < END_DIST_TOL && seg_idx >= static_cast<int>(smooth.size()) - 1) {
        // Dentro da tolerância → pára
        v_d = 0.0;
        w_d = 0.0;
        return;
    }

    // 2) Calcula referência no caminho suavizado
    RefState ref = computeRef(curr_x, curr_y, curr_theta, smooth, seg_idx);

    seg_idx = ref.seg_idx;
    last_th = ref.theta_r;

    double xr      = ref.xr;
    double yr      = ref.yr;
    double theta_r = ref.theta_r;
    double v_r     = ref.v_r;
    double w_r     = ref.w_r;

    double theta = curr_theta;

    // 3) Erro no referencial do robô
    double dx = xr - curr_x;
    double dy = yr - curr_y;

    double ex =  std::cos(theta) * dx + std::sin(theta) * dy;
    double ey = -std::sin(theta) * dx + std::cos(theta) * dy;

    double e_theta = theta_r - theta;
    // Normalizar para [-pi, pi)
    e_theta = std::atan2(std::sin(e_theta), std::cos(e_theta));

    // 4) Lei de controlo (igual ao código original)
    double v = v_r * std::cos(e_theta) + KX * ex;
    double w = w_r + KY * v_r * ey + KTH * std::sin(e_theta);

    // 5) Saturação
    if (v >  V_MAX) v =  V_MAX;
    if (v < -V_MAX) v = -V_MAX;
    if (w >  W_MAX) w =  W_MAX;
    if (w < -W_MAX) w = -W_MAX;

    // 6) Guardar como velocidades desejadas
    v_d = v;
    w_d = w;
}

