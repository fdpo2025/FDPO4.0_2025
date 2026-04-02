#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>
#include <netinet/in.h>
#include <std_srvs/SetBool.h>


class WifiDriverNode
{
public:
  explicit WifiDriverNode(ros::NodeHandle& nh);

private:
  ros::NodeHandle nh_;

  // ROS publisher (/color_sequence = compat; cópia para pi_pico encaminhar IWP sem loop)
  ros::Publisher color_pub_;
  ros::Publisher color_pub_iwp_dup_;

  // UDP config
  std::string server_ip_;
  int port_;
  double timeout_s_;

  int sock_fd_;
  sockaddr_in server_addr_;

  bool connected_ = false;
  std::string last_published_;

  ros::Timer timer_;

  ros::ServiceServer start_iwp_srv_;
  bool iwp_enabled_ = false;

  bool have_sequence_ = false;
  bool iwp_active_ = false;      // estamos a tentar obter sequência?
  std::string color_sequence_;


  bool startIwpCb(std_srvs::SetBool::Request& req,
                  std_srvs::SetBool::Response& res);

  void timerCb(const ros::TimerEvent&);

  bool setupSocket();
  bool sendMsg(const std::string& msg);
  bool recvMsg(std::string& out);

  void doPingPong();
  void doIWP();

  bool parseTxxx(const std::string& s, int& out_val);
  bool isColorSeq4(const std::string& s);

  
};
