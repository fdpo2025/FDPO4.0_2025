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
#include <sstream>
#include <cstdio>

struct Pose {

    double x, y, theta;

};

namespace Communication {

    namespace Message {

        struct ToPico {

            double v_d, w_d;
            bool pick_box; 

        };

        struct FromPico {

            Pose odom_pos;
            double v_linear;
            double w_angular;
            bool box_detection; // w/ ToF mesaure processed at the pico

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

        ros::Publisher detectBoxPub;
        void pubBoxDetection();

        tf::TransformBroadcaster tf_broadcaster;

};