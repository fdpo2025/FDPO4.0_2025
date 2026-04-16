#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <netinet/in.h>
#include <string>

class WifiDriverNode
{
public:
  explicit WifiDriverNode(ros::NodeHandle& nh);

private:
  enum class CommPhase
  {
    CaptureInitialCtl,
    WaitSequence,
    WaitCtlChange
  };

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

  CommPhase phase_ = CommPhase::CaptureInitialCtl;
  int initial_ctl_value_ = -1;
  bool have_sequence_ = false;
  std::string color_sequence_;

  void timerCb(const ros::TimerEvent&);

  bool setupSocket();
  bool sendMsg(const std::string& msg);
  bool recvMsg(std::string& out);

  void doPingPong();
  void doIWP();

  bool parseTxxx(const std::string& s, int& out_val);
  bool isColorSeq4(const std::string& s);
};
