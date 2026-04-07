#include "pico_driver_node.h"

#include <algorithm>

/*
 * Serial protocol (ROS1 <-> Pico):
 *   INIT:ID,COLOR_SEQ
 *   CMD:v,w,iman,my_cp,my_np,my_waiting,target_id,stop_waiting
 *   POS:x,y,theta,v,w;CP:a,b,c;NP:d,e,f;WT:g,h,i
 * Rules:
 *   - IDs are 0-based.
 *   - target_id=255 means broadcast.
 */

PiPicoDriver::PiPicoDriver(ros::NodeHandle& nh_) : nh(nh_) {
  messageToSend.v_d = 0.0;
  messageToSend.w_d = 0.0;
  messageToSend.iman = false;
  messageToSend.cp_send = 0;
  messageToSend.np_send = 0;
  messageToSend.waiting_send = false;
  messageToSend.target_id_send = 255;
  messageToSend.stop_waiting_send = false;

  messageToReceive.odom_pos = {0.0, 0.0, 0.0};
  messageToReceive.v_linear = 0.0;
  messageToReceive.w_angular = 0.0;
  messageToReceive.cp_rcv = 0;
  messageToReceive.np_rcv = 0;
  messageToReceive.wt_rcv = false;

  velSub = nh.subscribe("/cmd_vel", 10, &PiPicoDriver::velCallBack, this);
  pickBoxSub = nh.subscribe("/pick_box", 10, &PiPicoDriver::pickBoxCallBack, this);
  cpSendSub = nh.subscribe("/cp_send", 10, &PiPicoDriver::cpSendCallBack, this);
  npSendSub = nh.subscribe("/np_send", 10, &PiPicoDriver::npSendCallBack, this);
  wtSendSub = nh.subscribe("/wt_send", 10, &PiPicoDriver::wtSendCallBack, this);
  targetIdSendSub = nh.subscribe("/target_id_send", 10, &PiPicoDriver::targetIdSendCallBack, this);
  stopWaitingSendSub = nh.subscribe("/stop_waiting_send", 10, &PiPicoDriver::stopWaitingSendCallBack, this);

  posePub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
  cpRcvPub = nh.advertise<std_msgs::UInt32>("/cp_rcv", 10);
  npRcvPub = nh.advertise<std_msgs::UInt32>("/np_rcv", 10);
  wtRcvPub = nh.advertise<std_msgs::Bool>("/wt_rcv", 10);
  commTimer = nh.createTimer(ros::Duration(0.02), &PiPicoDriver::commTick, this);

  std::string serial_port;
  nh.param<std::string>("serial_port", serial_port, "/dev/ttyACM0");
  nh.param("debug_comm", debug_comm_, false);
  nh.param("robot_id", robot_id_, 0);

  ROS_INFO("[PiPicoDriver] Serial port: %s", serial_port.c_str());
  ROS_INFO("[PiPicoDriver] robot_id=%d debug=%s",
           robot_id_, debug_comm_ ? "ENABLED" : "DISABLED");

  serial_fd_ = -1;
  startSerial(serial_port);
}

PiPicoDriver::~PiPicoDriver() {
  if (serial_fd_ >= 0) close(serial_fd_);
}

void PiPicoDriver::startSerial(const std::string& port) {
  serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0) {
    ROS_ERROR("Erro ao abrir porta serial: %s", port.c_str());
    return;
  }

  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(serial_fd_, &tty) != 0) {
    ROS_ERROR("Erro ao obter atributos da serial");
    return;
  }

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag = 0;
  tty.c_oflag = 0;
  tty.c_lflag = 0;
  tty.c_cc[VMIN]  = 1;
  tty.c_cc[VTIME] = 1;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    ROS_ERROR("Erro ao aplicar configurações na serial");
  }
}

void PiPicoDriver::velCallBack(const geometry_msgs::Twist::ConstPtr& msg) {
  messageToSend.v_d = msg->linear.x;
  messageToSend.w_d = msg->angular.z;
}

void PiPicoDriver::pickBoxCallBack(const std_msgs::Bool::ConstPtr& msg) {
  messageToSend.iman = msg->data;
}

void PiPicoDriver::pubOdom() {
  ros::Time current_time = ros::Time::now();

  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id  = "base_link";
  odom.pose.pose.position.x = messageToReceive.odom_pos.x;
  odom.pose.pose.position.y = messageToReceive.odom_pos.y;
  odom.pose.pose.position.z = 0.0;

  tf::Quaternion q = tf::createQuaternionFromYaw(messageToReceive.odom_pos.theta);
  tf::quaternionTFToMsg(q, odom.pose.pose.orientation);
  odom.twist.twist.linear.x = messageToReceive.v_linear;
  odom.twist.twist.angular.z = messageToReceive.w_angular;
  posePub.publish(odom);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  odom_trans.transform.translation.x = messageToReceive.odom_pos.x;
  odom_trans.transform.translation.y = messageToReceive.odom_pos.y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom.pose.pose.orientation;
  tf_broadcaster.sendTransform(odom_trans);
}

std::string PiPicoDriver::syncCall(const std::string& cmd, int timeout_ms) {
  if (serial_fd_ < 0) {
    ROS_ERROR("Serial closed.");
    return "";
  }

  tcflush(serial_fd_, TCIFLUSH);
  const std::string command = cmd + "\n";
  if (write(serial_fd_, command.c_str(), command.size()) < 0) {
    ROS_ERROR("Erro ao enviar comando para a serial.");
    return "";
  }

  char buf[256];
  std::string response;
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout_duration(timeout_ms / 1000.0);

  while (ros::Time::now() - start_time < timeout_duration) {
    int n = read(serial_fd_, buf, sizeof(buf) - 1);
    if (n > 0) {
      buf[n] = '\0';
      response += std::string(buf);
      size_t newline_pos = response.find('\n');
      if (newline_pos != std::string::npos) {
        std::string line = response.substr(0, newline_pos);
        decodeMsg(line);
        return line;
      }
    }
  }

  ROS_WARN("Timeout esperando resposta da Pico.");
  return "";
}

void PiPicoDriver::decodeMsg(const std::string& msg) {
  if (msg.empty()) return;
  if (msg.rfind("INIT:", 0) == 0) {
    ROS_INFO_THROTTLE(5.0, "[PiPicoDriver] %s", msg.c_str());
    return;
  }

  double x = 0.0, y = 0.0, theta = 0.0, v = 0.0, w = 0.0;
  if (std::sscanf(msg.c_str(), "POS:%lf,%lf,%lf,%lf,%lf", &x, &y, &theta, &v, &w) != 5) {
    ROS_WARN_THROTTLE(10.0, "Mensagem desconhecida: %s", msg.substr(0, 120).c_str());
    return;
  }

  messageToReceive.odom_pos = {x, y, theta};
  messageToReceive.v_linear = v;
  messageToReceive.w_angular = w;
  pubOdom();

  const size_t cp_idx = msg.find(";CP:");
  const size_t np_idx = msg.find(";NP:");
  const size_t wt_idx = msg.find(";WT:");
  if (cp_idx == std::string::npos || np_idx == std::string::npos || wt_idx == std::string::npos) {
    return;
  }

  const char* cp_start = msg.c_str() + cp_idx + 4;
  const char* np_start = msg.c_str() + np_idx + 4;
  const char* wt_start = msg.c_str() + wt_idx + 4;

  messageToReceive.cp_all = parseUIntList(cp_start, (msg.c_str() + np_idx) - cp_start);
  messageToReceive.np_all = parseUIntList(np_start, (msg.c_str() + wt_idx) - np_start);
  messageToReceive.wt_all = parseUIntList(wt_start, msg.c_str() + msg.size() - wt_start);

  updateReducedStateFromArrays();

  std_msgs::UInt32 cp_msg;
  cp_msg.data = messageToReceive.cp_rcv;
  cpRcvPub.publish(cp_msg);

  std_msgs::UInt32 np_msg;
  np_msg.data = messageToReceive.np_rcv;
  npRcvPub.publish(np_msg);

  std_msgs::Bool wt_msg;
  wt_msg.data = messageToReceive.wt_rcv;
  wtRcvPub.publish(wt_msg);
}

void PiPicoDriver::commTick(const ros::TimerEvent&) {
  const int off = std::snprintf(cmd_buf_, sizeof(cmd_buf_),
                                "CMD:%.4f,%.4f,%u,%u,%u,%u,%u,%u",
                                messageToSend.v_d,
                                messageToSend.w_d,
                                messageToSend.iman ? 1U : 0U,
                                messageToSend.cp_send,
                                messageToSend.np_send,
                                messageToSend.waiting_send ? 1U : 0U,
                                messageToSend.target_id_send,
                                messageToSend.stop_waiting_send ? 1U : 0U);
  std::string cmd(cmd_buf_, off > 0 ? off : 0);

  if (debug_comm_) {
    ROS_INFO("Pi4->Pico: %s", cmd.c_str());
  }

  std::string resp = syncCall(cmd, 18);
  if (debug_comm_) {
    ROS_INFO("Pico->Pi4: %s", resp.c_str());
  }

  if (resp.empty()) {
    con_state.missed++;
    if (con_state.missed >= 2) {
      con_state.link_ok = false;
      ROS_WARN_THROTTLE(5.0, "[PiPicoDriver] Connection lost with Pico!");
    }
    return;
  }

  con_state.missed = 0;
  con_state.link_ok = true;
}

void PiPicoDriver::cpSendCallBack(const std_msgs::UInt32::ConstPtr& msg) {
  messageToSend.cp_send = std::min<uint32_t>(msg->data, 255);
}

void PiPicoDriver::npSendCallBack(const std_msgs::UInt32::ConstPtr& msg) {
  messageToSend.np_send = std::min<uint32_t>(msg->data, 255);
}

void PiPicoDriver::wtSendCallBack(const std_msgs::Bool::ConstPtr& msg) {
  messageToSend.waiting_send = msg->data;
}

void PiPicoDriver::targetIdSendCallBack(const std_msgs::UInt32::ConstPtr& msg) {
  messageToSend.target_id_send = std::min<uint32_t>(msg->data, 255);
}

void PiPicoDriver::stopWaitingSendCallBack(const std_msgs::Bool::ConstPtr& msg) {
  messageToSend.stop_waiting_send = msg->data;
}

void PiPicoDriver::pubExtraMsgs() {
  std_msgs::UInt32 cp_msg;
  cp_msg.data = messageToReceive.cp_rcv;
  cpRcvPub.publish(cp_msg);
}

std::vector<uint32_t> PiPicoDriver::parseUIntList(const char* s, size_t len) {
  std::vector<uint32_t> result;
  const char* end = s + len;
  while (s < end) {
    while (s < end && (*s == ' ' || *s == ',' || *s == ';')) ++s;
    if (s >= end) break;
    char* next = nullptr;
    long val = std::strtol(s, &next, 10);
    if (next == s) break;
    if (val < 0) val = 0;
    if (val > 255) val = 255;
    result.push_back(static_cast<uint32_t>(val));
    s = next;
  }
  return result;
}

void PiPicoDriver::updateReducedStateFromArrays() {
  const size_t max_size = std::min({messageToReceive.cp_all.size(),
                                    messageToReceive.np_all.size(),
                                    messageToReceive.wt_all.size()});
  if (max_size == 0) return;

  size_t peer_idx = 0;
  if (max_size > 1 && static_cast<size_t>(robot_id_) < max_size) {
    peer_idx = (robot_id_ == 0) ? 1 : 0;
    if (peer_idx >= max_size) peer_idx = 0;
  }

  messageToReceive.cp_rcv = messageToReceive.cp_all[peer_idx];
  messageToReceive.np_rcv = messageToReceive.np_all[peer_idx];
  messageToReceive.wt_rcv = (messageToReceive.wt_all[peer_idx] != 0);
}
