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
#include <mutex>
#include <set>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <navigation_controller/NavigationConfig.h>  
#include <boost/bind.hpp>
#include <visualization_msgs/Marker.h>                            
#include <plan_handler/NavPlan.h>
#include <plan_handler/CompletionFeedback.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt32MultiArray.h>



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
    bool is_process_warehouse;  // Warehouse de process (segmento final antes de pick/drop process)
    int node_id;
    /** Índice 0.. no NavPlan que gerou o ponto; -1 se não aplicável (ex. route.yaml / RViz). */
    int plan_index;
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
            bool use_stanley_follow_line;   // Follow_Line: ψ + atan2(k·e, max(|v|,soft_v)+eps)
            bool use_stanley_approaching;  // Approaching / PickDrop / process: mesma lei com k por estado
            double stanley_k;               // Ganho lateral em Follow_Line (atan2)
            double stanley_soft_v;          // Chão de |v| antes de somar eps (m/s)
            double stanley_eps;             // ε em atan2(k·e, max(|v_ref|,soft_v)+eps)
            double approaching_enter_dist_m;   // Entrar em Approaching quando dist(robô, pf) <= isto (m)
            /** Go-to process warehouse: só avança (v>0) quando |erro bearing| <= isto (rad). Maior → menos rotação em sitio, mais cedo avança. */
            double bearing_align_yaw_tol;
            double approaching_vel_normal;     // Piso de v_d no Approaching (antes de warehouse)
            double k_approaching;              // Ganhos estado Approaching (normal)
            double gain_approaching_fwd;
            double k_approaching_pickdrop;     // Ganhos estado Approaching_PickDrop
            double gain_approaching_fwd_pickdrop;
            double approaching_vel_pickdrop;   // Vel. ref. na lei quadrática em |w_d| (PickDrop)
            double pick_box_forward_vel;       // Velocidade no estado pickBoxForward
            double approaching_colinear_angle_rad;  // Se |Δθ| entre linha atual e pf→warehouse < isto, não entra em Approaching
            double k_approaching_process;
            double gain_approaching_fwd_process;
            double approaching_vel_process;
            /** Early drop: publish /pick_box=false when dist(go-to-XY target) <= this. <=0 disables. */
            double drop_pick_box_release_distance;

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
        /** Process warehouse go-to: 1) alinha v=0 até bearing_align_yaw_tol; 2) v constante (v_nom), w=0. */
        void goToXYProcessWarehouse();
        void followLine();

        std::deque<WayPoint> route;
        void updateDesiredPose();
        void loadRouteFromParameters();
        
        ros::Subscriber odomSub;
        void updateCurrPose(const nav_msgs::Odometry::ConstPtr& msg);

        ros::Publisher velPub;
        void publishVel();

        /** Optional early release trigger for drop warehouses (go-to-XY). */
        ros::Publisher pickBoxPub;
        int drop_pick_box_release_published_for_node_id_{-1};
        
        ros::Publisher lineMarkerPub;
        ros::Publisher virtualLineMarkerPub;
        void publishLineMarkers();
        void publishVirtualLineMarker(double pi_x, double pi_y, double pf_x, double pf_y);
        
        ros::Publisher navCompletionFeedbackPub;
        bool completion_feedback_sent;  // Para enviar feedback apenas uma vez por linha

        double last_vel_before_approaching_;  // Última |v_d| em Follow_Line; teto em Approaching
        double approaching_brake_ref_dist_;   // dist(robô,pf) ao entrar em Approaching (normaliza v proporcional)
        bool process_warehouse_goto_align_done_;  // Fase 2 do go-to process: já passou alinhamento inicial (v=0)

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

        ros::Subscriber waitStateSub_;
        void waitStateCallback(const std_msgs::Bool::ConstPtr& msg);
        bool network_wait_hold_{false};
        /** Mid-route pause: não fazer pop da route até wait_state libertar (pick _W). */
        bool route_execution_pause_hold_{false};
        ros::Subscriber nav_pause_after_wp_sub_;
        void navPauseAfterWpIndexCb(const std_msgs::UInt32MultiArray::ConstPtr& msg);
        ros::Publisher nav_route_pause_request_pub_;
        void onRouteWaypointConsumedForPauseCheck(const WayPoint& consumed);
        /** Pop consumed waypoint after non-pick warehouse and preserve consumed/pause callbacks. */
        void transitionAfterDropWarehouse(const char* trace_tag);
        /** Monotonic id from NavPlan.header.seq when route loaded from /nav_plan (plan_handler). */
        uint32_t loaded_nav_plan_seq_{0};
        ros::Publisher nav_plan_waypoint_consumed_pub_;
        std::mutex nav_pause_mtx_;
        std::vector<uint32_t> staged_nav_pause_indices_;
        std::multiset<uint32_t> pause_remaining_after_wp_;
        void loadRouteFromNavPlan(const plan_handler::NavPlan::ConstPtr& msg);
        
        bool load_from_route;  // Se deve carregar waypoints do route.yaml

        // Services        
        ros::ServiceServer controlSrv; 
        bool controlSrvCb(navigation_controller::NavigationControl::Request& req, navigation_controller::NavigationControl::Response& res);

        ros::Publisher currentNodePub;
        void publishCurrentNode(int node_id);
        int last_published_node_id = -1;

        /** Next graph node (route.front) for Pico /np_send when not using comunicacoes_node. */
        ros::Publisher npSendGraphPub_;
        void publishGraphNextNode();
        int last_published_np_node_id_{-999};

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
            /** Warehouse de processo (!align): go-to XY com fase inicial só rotação (bearing_align_yaw_tol). */
            processWarehouseGoToXY,
            pickBoxForward,  // Estado para andar para frente após chegar a warehouse de pick
            done

        }; 
    }
    
    namespace followLineStates {

        enum {
            Follow_Line = 0,
            Approaching,        // Linhas normais: v proporcional a error_dist
            Approaching_PickDrop,  // Pick/drop (warehouse não-process): v quadrática em |w_d|/w_nom
            Approaching_process_PickDrop  // Pick/drop warehouse process: mesma lei, parâmetros dedicados
        };
    }

}
