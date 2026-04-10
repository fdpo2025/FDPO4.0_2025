#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <string.h>
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
            bool iman;
            uint32_t cp_send;
            uint32_t np_send;
            bool waiting_send;
            uint32_t target_id_send;
            bool stop_waiting_send;

        };

        struct FromPico {

            Pose odom_pos;
            double v_linear;
            double w_angular;
            uint32_t cp_rcv;
            uint32_t np_rcv;
            bool wt_rcv;
            std::vector<uint32_t> cp_all;
            std::vector<uint32_t> np_all;
            std::vector<uint32_t> wt_all;

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
        bool debug_comm_;
        bool handshake_ok_{false};
        int num_robots_{3};
        static constexpr int kHandshakeTimeoutMs = 500;
        static constexpr int kCommTimeoutMs = 80;

        void startSerial(const std::string& port);
        bool trySerialHandshake();
        static uint8_t crc8DallasMaxim(const uint8_t* data, size_t len);
        void writeSerialRaw(const char* data, size_t len);
        // Client-Server Communication between Pico & Pi4
        std::string readUntilPosLine(const std::string& cmd, int timeout_ms);
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
        ros::Subscriber npSendSub;
        ros::Subscriber wtSendSub;
        ros::Subscriber targetIdSendSub;
        ros::Subscriber stopWaitingSendSub;
        ros::Subscriber colorSequenceSub_;

        ros::Publisher cpRcvPub;
        ros::Publisher npRcvPub;
        ros::Publisher wtRcvPub;
        ros::Publisher rawSerialPub_;
        ros::Publisher networkTablePub_;
        ros::Publisher waitStatePub_;
        ros::Publisher colorSequencePub_;
        /** Latched; id from Pico INIT line after handshake (fleet index 0-based). */
        ros::Publisher robot_identity_pub_;

        void cpSendCallBack(const std_msgs::UInt32::ConstPtr& msg);
        void npSendCallBack(const std_msgs::UInt32::ConstPtr& msg);
        void wtSendCallBack(const std_msgs::Bool::ConstPtr& msg);
        void targetIdSendCallBack(const std_msgs::UInt32::ConstPtr& msg);
        void stopWaitingSendCallBack(const std_msgs::Bool::ConstPtr& msg);
        void colorSequenceCallBack(const std_msgs::String::ConstPtr& msg);

        void pubExtraMsgs();
        void publishRawSerial(const char* direction, const std::string& line);
        void publishNetworkTableAndWaitState();
        std::vector<uint32_t> parseUIntList(const char* s, size_t len);
        void updateReducedStateFromArrays();

        char cmd_buf_[512];
        std::string last_color_from_pico_;
        std::string last_color_sent_to_pico_;
        std::string pending_color_to_pico_;
        bool has_pending_color_to_pico_{false};

        int robot_id_ = 0;

        bool wait_state_initialized_{false};
        bool last_published_wait_state_{false};

};