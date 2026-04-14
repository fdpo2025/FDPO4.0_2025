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
#include <std_msgs/Bool.h>
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
    bool is_process_warehouse;  // Warehouse de process (segmento final antes de pick/drop process)
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
            double approaching_colinear_angle_rad;  // Se |Δθ| entre linha atual e pf→warehouse < isto, não entra em Approaching
            double k_approaching_process;
            double gain_approaching_fwd_process;
            double approaching_vel_process;
            /** pickBoxForward: |v| após pick warehouse (YAML /follow_line/navigation_controller). */
            double pick_box_forward_vel = 0.1;
            /** Após drop: rotação em sitio (soltar íman) antes do segmento backwards; duração (s). */
            double drop_magnet_wiggle_angle_deg = 25.0;
            /** ω em dropMagnetReleaseWiggle (rad/s), eixo Z; sinal = sentido. */
            double drop_magnet_wiggle_angular_vel = 0.8;
            /** Publica /pick_box=false nesta percentagem da linha antes de um drop output. <=0 desliga. */
            double drop_pick_box_release_ratio_other = 0.9;
            /** Publica /pick_box=false nesta percentagem da linha antes de um drop process. <=0 desliga. */
            double drop_pick_box_release_ratio_process = 0.9;

        };

        dynamic_reconfigure::Server<navigation_controller::NavigationConfig> dr_srv_;
        Parameters param;
        void loadNavigationParams();
        void pickBoxAction();
        /** Após consumir waypoint de drop: pop + próximo segmento; se egress backwards, wiggle antes de done. */
        void transitionAfterDropWarehouse(const char* trace_tag);

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
        void maybePublishEarlyDropPickBoxRelease();
        /** Warehouse pick/drop (!align): go-to — 1) alinha v=0 até bearing_align_yaw_tol; 2) v (v_nom), w=0. */
        void goToXYProcessWarehouse();
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
        ros::Publisher pickBoxPub;
        bool completion_feedback_sent;  // Para enviar feedback apenas uma vez por linha
        int drop_pick_box_release_published_for_node_id_;

        double last_vel_before_approaching_;  // Última |v_d| em Follow_Line; teto em Approaching
        double approaching_brake_ref_dist_;   // dist(robô,pf) ao entrar em Approaching (normaliza v proporcional)
        bool process_warehouse_goto_align_done_;  // Fase 2 do warehouse go-to: já passou alinhamento inicial (v=0)
        /** Distância ao alvo ao entrar em warehouse go-to-XY (progresso 0–1 vs follow_line 70%). */
        double process_warehouse_goto_start_dist_;
        bool process_warehouse_goto_completion_sent_;

        // Estado para pick box forward
        ros::Time pick_box_forward_start_time;
        bool in_pick_box_forward;  // Se está no estado de andar para frente após pick
        double drop_magnet_wiggle_start_yaw_;
        double drop_magnet_wiggle_target_rad_;
        void skipNearbyWaypoints();  // Skip waypoints se já estamos perto deles

        ros::Timer controlTimer;
        void navigationFsmRunner(const ros::TimerEvent&);

        bool rvizGoalAppend;
        ros::Subscriber rvizGoalSub;
        void rvizGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);

        ros::Subscriber navPlanSub;
        void navPlanCallback(const plan_handler::NavPlan::ConstPtr& msg);
        void loadRouteFromNavPlan(const plan_handler::NavPlan::ConstPtr& msg);
        void appendRouteFromNavPlan(const plan_handler::NavPlan::ConstPtr& msg);
        
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
            /** Warehouse pick/drop (!align): go-to-XY (process e não-process). */
            processWarehouseGoToXY,
            pickBoxForward,  // Estado para andar para frente após chegar a warehouse de pick
            /** Após drop: ω constante até acumular drop_magnet_wiggle_angle_deg antes do backwards. */
            dropMagnetReleaseWiggle,
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
