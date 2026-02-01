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
#include <navigation_controller/NavigationControl.h>
#include "fsm.h"
#include <std_msgs/Bool.h>

#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>
#include <string>

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
    double yaw_tol;         // Tolerância de yaw para alinhamento final
    double a_max;           // Aceleração máxima (m/s²)
    double d_max;           // Desaceleração máxima (m/s²)
    double smooth_radius;
    int smooth_corner_steps;
    double slow_down_dist;  // começa a abrandar a 60 cm do goal
    double v_final;   
    double reverse_dist_tol;   // tolerância para "cheguei ao penúltimo"
    double reverse_speed;
    double time_to_reverse;
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
        ros::ServiceServer controlSrv;
        ros::Publisher pick_box_pub_;

        // ====================================================================
        // CALLBACKS ROS
        // ====================================================================
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        // ====================================================================
        // DATA AVAILABLE FOR CONTROLLER
        // ====================================================================
        // Colleague can use these variables in control logic
        double curr_x, curr_y, curr_theta;  // Current robot pose
        double v_d, w_d;                    // Desired velocities (output)
        int loop_rate_hz;                   // Control loop rate (Hz)
        Point reverse_target;
        bool reverse_target_valid = false;
        ros::Time reverse_start_time;
        bool reverse_waiting = false;
        std::vector<Point> wp_first3, wp_rest;

        
        // ====================================================================
        // FSM AND CONTROL MODE
        // ====================================================================
        Fsm navigationFsm;
        std::string mode;  // "idle" | "start" | "pause" | "stop"
        
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
        bool aux=false;
        
        // Funções auxiliares
        std::pair<double,double> normalize(double vx, double vy) const;
        std::vector<Point> smoothPath(const std::vector<Point>& path_in,
                                      double radius,
                                      int corner_steps) const;
        
        // Carregamento de waypoints
        void loadPathFromParameters();
        void updatePathFromWaypoints(const std::vector<Point>& waypoints);
        void loadNextRouteFromQueue();

        int route_queue_idx_ = 0;
        bool route_wrap_ = true;  // lê do param "route_wrap"
        bool have_route_list_ = false;
        
        // Controlo
        RefState computeRef(double x, double y, double theta,
                            const std::vector<Point>& path,
                            int seg_idx);
        void computeStateSpaceControl();
        
        // ====================================================================
        // FSM LOGIC
        // ====================================================================
        void navigationFsmRunner(const ros::TimerEvent&);
        void driveToGoal();
        void turnToFinalYaw();
        
        // ====================================================================
        // HELPER FUNCTIONS FOR FSM
        // ====================================================================
        double normalizeAngle(double theta);
        bool isPositionArrived();
        bool isYawDesired();
        double getPositionError();
        double getDesiredYawError();
        void setReverseTargetToPenultimate();
        double getReverseError(); 
        bool isReverseArrived();
        void reverseToPenultimate();
        void splitWaypoints(
            const std::vector<Point>& waypoints,
            std::vector<Point>& first3,
            std::vector<Point>& rest);
        void sendPickBoxCommand(bool pick_box);

        // ====================================================================
        // SERVICE CALLBACK
        // ====================================================================
        bool controlSrvCb(navigation_controller::NavigationControl::Request& req,
                         navigation_controller::NavigationControl::Response& res);

};

// ============================================================================
// NAMESPACE FOR FSM STATES
// ============================================================================
namespace navigation_ss {
    namespace states {
        enum {
            idle = 0,
            driveToGoal,
            turnToFinalYaw,
            reverseToPrev,
            done
        };
    }
}

