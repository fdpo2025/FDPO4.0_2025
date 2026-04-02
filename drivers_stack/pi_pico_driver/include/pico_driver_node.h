#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <plan_handler/CompletionFeedback.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <string.h>
#include <iostream>
#include <string>
#include <sstream>
#include <cstdio>
#include <vector>

struct Pose {

    double x, y, theta;

};

namespace Communication {

    namespace Message {

        struct ToPico {

            double v_d, w_d;
            bool pick_box;
            int cp;
            int final_node;
            int status;
            int wait_target;

        };

        struct FromPico {

            Pose odom_pos;
            double v_linear;
            double w_angular;

        };

    };

    struct ConnectionState {

        bool link_ok;
        int missed;

    };

};


class PiPicoDriver {
    
    public:
        PiPicoDriver(ros::NodeHandle& nh_);
        ~PiPicoDriver();

    private: 
        ros::NodeHandle& nh;

        Communication::Message::ToPico messageToSend;
        Communication::Message::FromPico messageToReceive;
        Communication::ConnectionState con_state{false, 0};
    
        int serial_fd_;
        bool debug_comm_;  // Parâmetro para mostrar/ocultar logs de comunicação
        void startSerial(const std::string& port);
        
        // Client-Server Communication between Pico & Pi4
        std::string syncCall(const std::string& cmd, int timeout_ms);
        void decodeMsg(const std::string& msg);

         ros::Timer commTimer;
        void commTick(const ros::TimerEvent&);

        ros::Subscriber velSub;
        void velCallBack(const geometry_msgs::Twist::ConstPtr& msg);

        ros::Subscriber pickBoxSub;
        void pickBoxCallBack(const std_msgs::Bool::ConstPtr& msg);  

        ros::Subscriber plannedPathSub;
        void plannedPathCallBack(const std_msgs::Int32MultiArray::ConstPtr& msg);

        ros::Subscriber navFeedbackSub;
        void navFeedbackCallBack(const plan_handler::CompletionFeedback::ConstPtr& msg);
        
        ros::Subscriber radioWaitTargetSub;
        void radioWaitTargetCallBack(const std_msgs::Int32::ConstPtr& msg);

        ros::Subscriber colorSeqWifiSub_;
        void colorSeqWifiCallback(const std_msgs::String::ConstPtr& msg);

        ros::Publisher posePub;
        ros::Publisher robotIdPub;
        ros::Publisher radioWaitReleasePub;
        ros::Publisher missionColorPub_;
        void pubOdom();

        int robot_id_ = -1;
        bool identity_logged_once_{false};
        bool wait_release_pending_ = false;
        std::string pending_colorseq_for_pico_;
        std::string last_wifi_color_seq_;
        std::string wifi_iwp_sub_topic_;
        std::string color_sequence_pub_topic_;
        std::vector<int> active_path_nodes_;
        size_t active_path_index_ = 0;
        void updateMotionStatus();
        void tryReadBootIdentity();
        bool decodeIdentityLine(const std::string& line, int& out_id) const;
        bool tryParseEmbeddedRobotId(const std::string& msg, int& out_id) const;
        void applyRobotIdentity(int id);

        tf::TransformBroadcaster tf_broadcaster;

        bool debug_identity_{false};
        bool debug_radio_{false};

};