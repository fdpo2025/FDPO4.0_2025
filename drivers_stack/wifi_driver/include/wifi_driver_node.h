#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>
#include <netinet/in.h>

class WifiDriverNode
{
public:
  explicit WifiDriverNode(ros::NodeHandle& nh);

private:
  ros::NodeHandle nh_;

  // ROS publisher
  ros::Publisher color_pub_;

  // UDP config
  std::string server_ip_;
  int port_;
  double timeout_s_;

  int sock_fd_;
  sockaddr_in server_addr_;

  bool connected_ = false;
  std::string last_published_;

  ros::Timer timer_;

  void timerCb(const ros::TimerEvent&);

  bool setupSocket();
  bool sendMsg(const std::string& msg);
  bool recvMsg(std::string& out);

  void doPingPong();
  void doIWP();
};
