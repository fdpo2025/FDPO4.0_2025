#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <string.h>
#include <iostream>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <std_msgs/Int32MultiArray.h>
#include <vector>
#include <std_msgs/UInt32.h>

struct Pose {

    double x, y, theta;

};

namespace Communication {

    namespace Message {

        struct ToPico {

            double v_d, w_d;
            bool pick_box; 
            uint32_t cp_send;
            std::vector<int32_t> path_send;


        };

        struct FromPico {

            Pose odom_pos;
            double v_linear;
            double w_angular;
            uint32_t cp_rcv;
            std::vector<int32_t> path_rcv;

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

        ros::Publisher posePub;
        void pubOdom();

        tf::TransformBroadcaster tf_broadcaster;

        ros::Subscriber cpSendSub;
        ros::Subscriber pathSendSub;

        ros::Publisher cpRcvPub;
        ros::Publisher pathRcvPub;

        void cpSendCallBack(const std_msgs::UInt32::ConstPtr& msg);
        void pathSendCallBack(const std_msgs::Int32MultiArray::ConstPtr& msg);

        void pubExtraMsgs();
        std::vector<int32_t> parsePathList(const char* s, size_t len);
        int pathToBuffer(const std::vector<int32_t>& path, char* buf, size_t buf_size);

        char cmd_buf_[512];

        std::vector<int32_t> path_to_send_;
        int path_send_retries_ = 0;
        bool cp_dirty_ = false;

        std::vector<int32_t> last_path_rcv_;
        bool has_last_path_rcv_ = false;

        uint32_t last_cp_rcv_ = 0;
        bool has_last_cp_rcv_ = false;

};