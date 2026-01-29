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


#include "fsm.h"


struct Pose {

    double x, y, theta;

};

struct WayPoint {

    int id;
    Pose pose;
    bool align;
    bool backwards;

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
        Fsm followLineFsm;  // FSM para followLine
        // both with respect to the map frame
        Pose poseCurr, poseDesired;
        double v_d, w_d;
        double k1;  // Variável calculada em dist2Line, usada em followLine
        WayPoint previousWaypoint;  // Guarda o waypoint anterior para criar linha quando só resta 1
        
        struct Parameters {

            double v_nom, w_nom, w_min;
            double v_min;  // Velocidade mínima constante na fase de desaceleração
            double v_max;  // Velocidade máxima (m/s) - limite absoluto
            double a_max;  // Aceleração máxima (m/s²)
            double d_max;  // Desaceleração máxima (m/s²)
            double kp_linear, kp_angular;
            double k_line;  // Ganho para correção de linha (followLine)
            double arrive_radius, yaw_tol;
            int loop_rate_hz;
            bool invert_odom_theta;  // Se true, inverte o theta do odom (corrige frame invertido)
            
            // FollowLine parameters (from Pascal code)
            double gain_fwd;      // GAIN_FWD
            double vel_lin_nom;   // VEL_LIN_NOM
            double dist_da;       // DIST_DA
            double tol_findist;   // TOL_FINDIST
            double max_etf;       // MAX_ETF

        };

        dynamic_reconfigure::Server<navigation_controller::NavigationConfig> dr_srv_;
        Parameters param;
        void loadNavigationParams();

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
        void publishLineMarkers();

        ros::Timer controlTimer;
        void navigationFsmRunner(const ros::TimerEvent&);

        bool rvizGoalAppend;
        ros::Subscriber rvizGoalSub;
        void rvizGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);

        // Services        
        ros::ServiceServer controlSrv; 
        bool controlSrvCb(navigation_controller::NavigationControl::Request& req, navigation_controller::NavigationControl::Response& res);

};

namespace navigation {

    namespace states {

        enum {

            idle = 0,
            driveToGoal,
            turnToFinalYaw,
            done

        }; 
    }
    
    namespace followLineStates {
        
        enum {
            GoTo_Init = 0,
            Follow_Line,
            Approaching,
            Final_Rot,
            Stop_line
        };
    }

}
