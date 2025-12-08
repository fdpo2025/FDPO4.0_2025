//
//  Navigation Controller - State Space Template
//  Template with basic ROS integration (subscriptions/publications)
//  State space control logic to be implemented
//

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>
#include <XmlRpcValue.h>

#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>

// ============================================================================
// MAIN CLASS - NavigationControllerSS
// ============================================================================
// This class has only basic ROS infrastructure.
// Colleague should implement state space control logic.
// ============================================================================

// ============================================================================
// ESTRUTURA DE PARÂMETROS DO CONTROLADOR
// ============================================================================
struct SSControllerParams {
    double kx;
    double ky;
    double kth;
    double v_max;
    double w_max;
    double v_ref;
    double end_dist_tol;
    double smooth_radius;
    int smooth_corner_steps;
};

// ============================================================================
// ESTRUTURAS DE DADOS
// ============================================================================
struct Point {
    double x;
    double y;
};

struct Pose {
    double x;
    double y;
    double theta;
};

struct RefState {
    double xr;
    double yr;
    double theta_r;
    double v_r;
    double w_r;
    int seg_idx;
};

class NavigationControllerSS {

    public:
        NavigationControllerSS(ros::NodeHandle& nh_);

    private:
        ros::NodeHandle& nh;

        // ====================================================================
        // ROS INFRASTRUCTURE
        // ====================================================================
        ros::Subscriber odomSub;
        ros::Subscriber rvizGoalSub;
        ros::Publisher velPub;
        ros::Timer controlTimer;

        // ====================================================================
        // CALLBACKS ROS
        // ====================================================================
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void controlLoop(const ros::TimerEvent&);

        // ====================================================================
        // DATA AVAILABLE FOR CONTROLLER
        // ====================================================================
        // Colleague can use these variables in control logic
        double curr_x, curr_y, curr_theta;  // Current robot pose
        double v_d, w_d;                    // Desired velocities (output)
        
        // ====================================================================
        // PARÂMETROS DO CONTROLADOR
        // ====================================================================
        SSControllerParams params;
        void loadControllerParams();

        // ====================================================================
        // AREA FOR STATE SPACE LOGIC IMPLEMENTATION
        // ====================================================================
        // TODO: Add controller state variables
        // TODO: Implement control calculation function
        std::vector<Point> path;      // caminho base (poucos pontos)
        std::vector<Point> smooth;    // caminho suavizado
        int seg_idx;                  // índice do segmento atual
        double last_th;               // último theta de referência
        
        // Funções auxiliares
        std::pair<double,double> normalize(double vx, double vy) const;
        std::vector<Point> smoothPath(const std::vector<Point>& path_in,
                                      double radius,
                                      int corner_steps) const;
        
        // Carregamento de waypoints
        void loadPathFromParameters();
        void updatePathFromWaypoints(const std::vector<Point>& waypoints);
        
        // Controlo
        RefState computeRef(double x, double y, double theta,
                            const std::vector<Point>& path,
                            int seg_idx);
        void computeStateSpaceControl();

};

