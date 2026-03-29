//
//  Created by afonso on 07/09/2025
//

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h> 
#include <cmath>
#include <algorithm>
#include <deque> 
#include <XmlRpcValue.h>
#include <geometry_msgs/PoseStamped.h> 
#include <navigation_controller/NavigationControl.h> 
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <navigation_controller/NavigationConfig.h>  
#include <boost/bind.hpp>
#include <visualization_msgs/Marker.h>                            
#include <plan_handler/NavPlan.h>
#include <plan_handler/CompletionFeedback.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>



#include "fsm.h"


struct Pose {

    double x, y, theta;

};

struct WayPoint {

    int id;
    Pose pose;
    bool align;
    bool backwards;
    double line_switch_ratio;  // % da linha para mudar para próxima (0.9 = 90%), -1 = usar global
    double vel_lin_nom;        // Velocidade linear nominal para esta linha, -1 = usar global
    bool pick_box;             // Se é warehouse de pick (true) ou drop (false)
    bool is_warehouse;          // Se o ponto final é uma warehouse
    int node_id;
};

struct Line{

    WayPoint pi;
    WayPoint pf;
    
};

class NavigationController {

    public:
        NavigationController(ros::NodeHandle& nh_);
        void reconfigCb(navigation_controller::NavigationConfig &cfg, uint32_t level);

    private:
        ros::NodeHandle& nh;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;

        std::string mode; // "start" | "pause" | "unpause" | "stop""

        Fsm navigationFsm;
        Fsm followLineFsm;  // followLine fsm
        // both with respect to the map frame
        Pose poseCurr, poseDesired;
        double v_d, w_d;
        double k1;  // dist2Line result (perpendicular distance with sign)
        double line_progress;  // dist2Line result (0 = at pi, 1 = at pf, >1 = past pf)
        WayPoint currentWaypoint, previousWaypoint;  // currentWaypoint: pi; previousWaypoint: pf
        const double ALIGN_DIST_THRESHOLD = 0.02;
        const double ALIGN_YAW_THRESHOLD = 0.025;


        struct Parameters {

            double v_nom, w_nom, w_min;
            double v_min;  
            double v_max;  
            double a_max;  
            double d_max;  
            double kp_linear, kp_angular;
            double k_line;  
            double arrive_radius, yaw_tol;
            int loop_rate_hz;
            bool invert_odom_theta;  
            
            // FollowLine parameters 
            double gain_fwd;      // GAIN_FWD
            double vel_lin_nom;   // VEL_LIN_NOM
            double dist_da;       // DIST_DA
            double tol_findist;   // TOL_FINDIST
            double max_etf;       // MAX_ETF
            double tol_init_line; // (m) tolerance to GoTo_Init -> Follow_Line
            double line_switch_ratio;  // Ratio of line to switch to next (0.9 = 90%)
            double approaching_line_progress;  // Line progress threshold to enter Approaching state (0.60 = 60%)
            double approaching_vel;            // Constant velocity in Approaching state (m/s)
            double k_approaching;              // Gain for angular control in Approaching state
            double gain_approaching_fwd;         // Forward gain for control in Approaching state
            double approaching_colinear_angle_rad;  // Se |Δθ| entre linha atual e pf→warehouse < isto, não entra em Approaching

        };

        dynamic_reconfigure::Server<navigation_controller::NavigationConfig> dr_srv_;
        Parameters param;
        void loadNavigationParams();
        void pickBoxAction();

        double normalizeAngle(double theta);

        // Check if is suppose to move backwards
        bool isBackwards();
        // Align to reach the desired position
        double getAlignYawError();
        bool checkAlignYaw();
        // Go to desired position
        double getPositionError();
        bool isPositionArrived();
        // Align to the desired theta
        double getDesiredYawError();
        bool isYawDesired();

        // Follow line functions
        void dist2Line(double xi, double yi, double xf, double yf, double xr, double yr, double& distLine);
        void dist2LineVirtual(double xi, double yi, double xf, double yf, double xr, double yr,
                              double& k1_virtual, double& error_ang_virtual);
        double getLineAngle(double pi_x, double pi_y, double pf_x, double pf_y);
        double getLineError();
        double getAlignLineYawError();

        void hardStop();
        void setTheta();
        void goToXY();
        void followLine();

        std::deque<WayPoint> route;
        void updateDesiredPose();
        void loadRouteFromParameters();
        
        ros::Subscriber odomSub;
        void updateCurrPose(const nav_msgs::Odometry::ConstPtr& msg);

        ros::Publisher velPub;
        void publishVel();
        
        ros::Publisher lineMarkerPub;
        ros::Publisher virtualLineMarkerPub;
        void publishLineMarkers();
        void publishVirtualLineMarker(double pi_x, double pi_y, double pf_x, double pf_y);
        
        ros::Publisher navCompletionFeedbackPub;
        bool completion_feedback_sent;  // Para enviar feedback apenas uma vez por linha

        double last_vel_before_approaching_;  // Última v_d em Follow_Line; limite máximo em Approaching
        double approaching_start_progress_;   // line_progress ao entrar em Approaching (para rampa \)

        // Estado para pick box forward
        ros::Time pick_box_forward_start_time;
        bool in_pick_box_forward;  // Se está no estado de andar para frente após pick
        void skipNearbyWaypoints();  // Skip waypoints se já estamos perto deles

        ros::Timer controlTimer;
        void navigationFsmRunner(const ros::TimerEvent&);

        bool rvizGoalAppend;
        ros::Subscriber rvizGoalSub;
        void rvizGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);

        ros::Subscriber navPlanSub;
        void navPlanCallback(const plan_handler::NavPlan::ConstPtr& msg);
        void loadRouteFromNavPlan(const plan_handler::NavPlan::ConstPtr& msg);
        
        bool load_from_route;  // Se deve carregar waypoints do route.yaml

        // Services        
        ros::ServiceServer controlSrv; 
        bool controlSrvCb(navigation_controller::NavigationControl::Request& req, navigation_controller::NavigationControl::Response& res);

        ros::Publisher currentNodePub;
        void publishCurrentNode(int node_id);
        int last_published_node_id = -1;

        plan_handler::NavPlan pendingNavPlan;
        bool hasPendingNavPlan = false;
        void loadPendingNavPlanIfAvailable();

};

namespace navigation {

    namespace states {

        enum {

            idle = 0,
            driveToGoal,
            turnToFinalYaw,
            pickBoxForward,  // Estado para andar para frente após chegar a warehouse de pick
            done

        }; 
    }
    
    namespace followLineStates {

        enum {
            Follow_Line = 0,
            Approaching,        // Linhas normais: v proporcional a error_dist
            Approaching_PickDrop  // Pick/drop (warehouse): v cúbica em w_d
        };
    }

}
