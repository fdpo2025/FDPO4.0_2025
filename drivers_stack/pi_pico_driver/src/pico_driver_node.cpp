#include "pico_driver_node.h"

#include <algorithm>
#include <cstring>

#include <pi_pico_driver/RadioNetworkTable.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

/*
 * Serial protocol (ROS1 <-> Pico):
 *   Handshake: REQ:INIT -> INIT:id,num_robots,crc8 -> ACK:OK
 *   CMD:v,w,iman,my_cp,my_np,my_waiting,my_timeline_index
 *   Optional: COLOR:RRRR (4 chars R|G|B) before POS; Pi4 ACK_COLOR:OK
 *   POS:x,y,theta,v,w;CP:...;NP:...;WT:...;TI:...
 *   TI: timeline_index por robô (0..255). Usado localmente por custom_planner para libertar WAIT_N.
 */

PiPicoDriver::PiPicoDriver(ros::NodeHandle& nh_) : nh(nh_) {
  messageToSend.v_d = 0.0;
  messageToSend.w_d = 0.0;
  messageToSend.iman = false;
  messageToSend.cp_send = 0;
  messageToSend.np_send = 0;
  messageToSend.waiting_send = false;
  messageToSend.timeline_index_send = 0;

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

  std::string timeline_index_topic;
  nh.param<std::string>("timeline_index_send_topic", timeline_index_topic,
                        std::string("/custom_planner/my_timeline_index"));
  timelineIndexSendSub_ =
      nh.subscribe(timeline_index_topic, 10, &PiPicoDriver::timelineIndexSendCallBack, this);

  colorSequenceSub_ = nh.subscribe("/color_sequence", 10, &PiPicoDriver::colorSequenceCallBack, this);
  if (mirror_this_current_pose_to_cp_send_) {
    thisCurrentPoseForCpSub_ =
        nh.subscribe("/this_current_pose", 10, &PiPicoDriver::thisCurrentPoseForCpSendCb, this);
  }

  posePub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
  cpRcvPub = nh.advertise<std_msgs::UInt32>("/cp_rcv", 10);
  npRcvPub = nh.advertise<std_msgs::UInt32>("/np_rcv", 10);
  wtRcvPub = nh.advertise<std_msgs::Bool>("/wt_rcv", 10);
  rawSerialPub_ = nh.advertise<std_msgs::String>("/pi_pico_driver/raw_serial", 50);
  networkTablePub_ = nh.advertise<pi_pico_driver::RadioNetworkTable>("/pi_pico_driver/network_table", 10);
  waitStatePub_ = nh.advertise<std_msgs::Bool>("/pi_pico_driver/wait_state", 10, true);
  colorSequencePub_ = nh.advertise<std_msgs::String>("/color_sequence", 10, true);
  commTimer = nh.createTimer(ros::Duration(0.02), &PiPicoDriver::commTick, this);

  std::string serial_port;
  nh.param<std::string>("serial_port", serial_port, "/dev/ttyACM0");
  nh.param("debug_comm", debug_comm_, false);
  nh.param("robot_id", robot_id_, 0);
  nh.param("num_robots", num_robots_, 3);
  nh.param("mirror_this_current_pose_to_cp_send", mirror_this_current_pose_to_cp_send_, true);

  std::string robot_identity_topic;
  nh.param<std::string>("robot_identity_topic", robot_identity_topic, "/robot_identity");
  robot_identity_pub_ = nh.advertise<std_msgs::Int32>(robot_identity_topic, 1, true);

  ROS_INFO("[PiPicoDriver] Serial port: %s", serial_port.c_str());
  ROS_INFO("[PiPicoDriver] robot_id=%d num_robots=%d debug=%s mirror_this_current_pose_to_cp_send=%s ti_topic=%s",
           robot_id_, num_robots_, debug_comm_ ? "ENABLED" : "DISABLED",
           mirror_this_current_pose_to_cp_send_ ? "true" : "false",
           timeline_index_topic.c_str());

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

uint8_t PiPicoDriver::crc8DallasMaxim(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; ++j) {
      if (crc & 0x01)
        crc = static_cast<uint8_t>((crc >> 1) ^ 0x8C);
      else
        crc >>= 1;
    }
  }
  return crc;
}

void PiPicoDriver::writeSerialRaw(const char* data, size_t len) {
  if (serial_fd_ < 0 || !data || len == 0) return;
  (void)::write(serial_fd_, data, len);
}

void PiPicoDriver::publishRawSerial(const char* direction, const std::string& line) {
  if (!direction) return;
  std_msgs::String msg;
  msg.data = std::string(direction) + ":" + line;
  rawSerialPub_.publish(msg);
}

bool PiPicoDriver::trySerialHandshake() {
  if (serial_fd_ < 0) return false;
  tcflush(serial_fd_, TCIFLUSH);
  static const char req[] = "REQ:INIT\n";
  if (::write(serial_fd_, req, sizeof(req) - 1) < 0) {
    return false;
  }
  publishRawSerial("TX", "REQ:INIT");

  std::string acc;
  char buf[256];
  const ros::Time start = ros::Time::now();
  const ros::Duration limit(kHandshakeTimeoutMs / 1000.0);

  while (ros::Time::now() - start < limit) {
    const int n = static_cast<int>(::read(serial_fd_, buf, sizeof(buf) - 1));
    if (n > 0) {
      buf[n] = '\0';
      acc += std::string(buf);
      size_t p = 0;
      while ((p = acc.find('\n')) != std::string::npos) {
        std::string line = acc.substr(0, p);
        acc.erase(0, p + 1);
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) line.pop_back();
        if (line.rfind("INIT:", 0) != 0) {
          continue;
        }
        publishRawSerial("RX", line);
        int id = -1;
        int num = -1;
        unsigned crc_in = 0;
        if (std::sscanf(line.c_str(), "INIT:%d,%d,%u", &id, &num, &crc_in) != 3) {
          continue;
        }
        if (num < 2 || num > 4) continue;
        if (id < 0 || id >= num_robots_) continue;
        if (id != robot_id_) continue;
        if (num != num_robots_) continue;
        char pl[16];
        const int m = std::snprintf(pl, sizeof(pl), "%d,%d", id, num);
        if (m <= 0) continue;
        const uint8_t crc = crc8DallasMaxim(reinterpret_cast<const uint8_t*>(pl), static_cast<size_t>(m));
        if (static_cast<unsigned>(crc) != crc_in) continue;
        static const char ack[] = "ACK:OK\n";
        writeSerialRaw(ack, sizeof(ack) - 1);
        publishRawSerial("TX", "ACK:OK");
        if (debug_comm_) {
          ROS_INFO_THROTTLE(1.0, "[PiPicoDriver] Handshake OK id=%d num_robots=%d", id, num);
        }
        // Same id as in INIT from Pico (must match launch robot_id); latched for custom_planner / others.
        std_msgs::Int32 rid_msg;
        rid_msg.data = id;
        robot_identity_pub_.publish(rid_msg);
        return true;
      }
    } else {
      ros::WallDuration(0.001).sleep();
    }
  }
  return false;
}

std::string PiPicoDriver::readUntilPosLine(const std::string& cmd, int timeout_ms) {
  if (serial_fd_ < 0) return "";
  tcflush(serial_fd_, TCIFLUSH);
  const std::string command = cmd + "\n";
  if (::write(serial_fd_, command.c_str(), command.size()) < 0) {
    ROS_ERROR_THROTTLE(5.0, "Erro ao enviar comando para a serial.");
    return "";
  }
  publishRawSerial("TX", cmd);

  std::string acc;
  char buf[256];
  const ros::Time start_time = ros::Time::now();
  const ros::Duration timeout_duration(timeout_ms / 1000.0);

  while (ros::Time::now() - start_time < timeout_duration) {
    const int n = static_cast<int>(::read(serial_fd_, buf, sizeof(buf) - 1));
    if (n > 0) {
      buf[n] = '\0';
      acc += std::string(buf);
      size_t pos = 0;
      while ((pos = acc.find('\n')) != std::string::npos) {
        std::string line = acc.substr(0, pos);
        acc.erase(0, pos + 1);
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) line.pop_back();
        if (line.empty()) continue;
        publishRawSerial("RX", line);
        if (line.rfind("COLOR:", 0) == 0) {
          decodeMsg(line);
          continue;
        }
        if (line.rfind("POS:", 0) == 0) {
          decodeMsg(line);
          return line;
        }
        ROS_WARN_THROTTLE(2.0, "[PiPicoDriver] Linha inesperada antes de POS: %s",
                          line.substr(0, 120).c_str());
      }
    }
  }

  ROS_WARN_THROTTLE(5.0, "Timeout esperando POS da Pico.");
  return "";
}

void PiPicoDriver::decodeMsg(const std::string& msg) {
  if (msg.empty()) return;

  if (msg.rfind("COLOR:", 0) == 0) {
    const std::string pay = msg.size() > 6 ? msg.substr(6) : "";
    if (pay.size() == 4) {
      bool ok = true;
      for (char c : pay) {
        if (c != 'R' && c != 'G' && c != 'B') {
          ok = false;
          break;
        }
      }
      if (ok) {
        std_msgs::String m;
        m.data = pay;
        last_color_from_pico_ = pay;
        colorSequencePub_.publish(m);
        writeSerialRaw("ACK_COLOR:OK\n", 13);
        ROS_INFO_THROTTLE(2.0, "[PiPicoDriver] COLOR -> /color_sequence (%s)", pay.c_str());
      }
    }
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

  // TI: (timeline_index) é opcional para compat; se ausente, fica vazio e custom_planner lida.
  const size_t ti_idx = msg.find(";TI:");

  const char* cp_start = msg.c_str() + cp_idx + 4;
  const char* np_start = msg.c_str() + np_idx + 4;
  const char* wt_start = msg.c_str() + wt_idx + 4;

  const size_t wt_end = (ti_idx == std::string::npos) ? msg.size() : ti_idx;
  messageToReceive.cp_all = parseUIntList(cp_start, (msg.c_str() + np_idx) - cp_start);
  messageToReceive.np_all = parseUIntList(np_start, (msg.c_str() + wt_idx) - np_start);
  messageToReceive.wt_all = parseUIntList(wt_start, (msg.c_str() + wt_end) - wt_start);
  if (ti_idx != std::string::npos) {
    const char* ti_start = msg.c_str() + ti_idx + 4;
    messageToReceive.ti_all =
        parseUIntList(ti_start, msg.c_str() + msg.size() - ti_start);
  } else {
    messageToReceive.ti_all.clear();
  }

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

  publishNetworkTableAndWaitState();
}

void PiPicoDriver::commTick(const ros::TimerEvent&) {
  if (!handshake_ok_) {
    if (trySerialHandshake()) {
      handshake_ok_ = true;
      con_state.missed = 0;
      con_state.link_ok = true;
      // Pico resets color state on REQ:INIT; allow the same /color_sequence to be sent again.
      last_color_from_pico_.clear();
      last_color_sent_to_pico_.clear();
    } else {
      ROS_WARN_THROTTLE(2.0,
                        "[PiPicoDriver] À espera do handshake (REQ:INIT / INIT / ACK)...");
    }
    return;
  }

  if (has_pending_color_to_pico_) {
    const std::string color_cmd = "COLOR:" + pending_color_to_pico_ + "\n";
    writeSerialRaw(color_cmd.c_str(), color_cmd.size());
    publishRawSerial("TX", color_cmd.substr(0, color_cmd.size() - 1));
    if (debug_comm_) {
      ROS_INFO_THROTTLE(2.0, "Pi4->Pico: %s", color_cmd.substr(0, color_cmd.size() - 1).c_str());
    }
    last_color_sent_to_pico_ = pending_color_to_pico_;
    has_pending_color_to_pico_ = false;
  }

  const int off = std::snprintf(cmd_buf_, sizeof(cmd_buf_),
                                "CMD:%.4f,%.4f,%u,%u,%u,%u,%u",
                                messageToSend.v_d,
                                messageToSend.w_d,
                                messageToSend.iman ? 1U : 0U,
                                messageToSend.cp_send,
                                messageToSend.np_send,
                                messageToSend.waiting_send ? 1U : 0U,
                                static_cast<unsigned>(messageToSend.timeline_index_send));
  const std::string cmd(cmd_buf_, off > 0 ? off : 0);

  if (debug_comm_) {
    ROS_INFO_THROTTLE(2.0, "Pi4->Pico: %s", cmd.c_str());
  }

  const std::string resp = readUntilPosLine(cmd, kCommTimeoutMs);
  if (debug_comm_) {
    ROS_INFO_THROTTLE(2.0, "Pico->Pi4 POS: %s", resp.c_str());
  }

  if (resp.empty()) {
    con_state.missed++;
    if (con_state.missed >= 5) {
      handshake_ok_ = false;
      con_state.missed = 0;
      ROS_WARN_THROTTLE(8.0, "[PiPicoDriver] Link perdido; a repetir handshake.");
    }
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

void PiPicoDriver::timelineIndexSendCallBack(const std_msgs::UInt8::ConstPtr& msg) {
  messageToSend.timeline_index_send = msg->data;
}

void PiPicoDriver::thisCurrentPoseForCpSendCb(const std_msgs::UInt32::ConstPtr& msg) {
  messageToSend.cp_send = std::min<uint32_t>(msg->data, 255u);
}

void PiPicoDriver::colorSequenceCallBack(const std_msgs::String::ConstPtr& msg) {
  const std::string seq = msg->data;
  if (seq.size() != 4) return;
  for (char c : seq) {
    if (c != 'R' && c != 'G' && c != 'B') {
      return;
    }
  }
  // Ignore colors that originated from this Pico uplink.
  if (seq == last_color_from_pico_) return;
  if (seq == last_color_sent_to_pico_) return;
  pending_color_to_pico_ = seq;
  has_pending_color_to_pico_ = true;
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
  const size_t n = std::min({messageToReceive.cp_all.size(),
                             messageToReceive.np_all.size(),
                             messageToReceive.wt_all.size()});
  if (n == 0) {
    messageToReceive.cp_rcv = 0;
    messageToReceive.np_rcv = 0;
    messageToReceive.wt_rcv = false;
    return;
  }
  if (robot_id_ < 0 || static_cast<size_t>(robot_id_) >= n) {
    messageToReceive.cp_rcv = 0;
    messageToReceive.np_rcv = 0;
    messageToReceive.wt_rcv = false;
    ROS_WARN_THROTTLE(5.0, "[PiPicoDriver] robot_id=%d out of bounds for CP/NP/WT (n=%zu)",
                      robot_id_, n);
    return;
  }
  const size_t idx = static_cast<size_t>(robot_id_);
  messageToReceive.cp_rcv = messageToReceive.cp_all[idx];
  messageToReceive.np_rcv = messageToReceive.np_all[idx];
  messageToReceive.wt_rcv = (messageToReceive.wt_all[idx] != 0);
}

void PiPicoDriver::publishNetworkTableAndWaitState() {
  const size_t n = std::min({messageToReceive.cp_all.size(),
                             messageToReceive.np_all.size(),
                             messageToReceive.wt_all.size()});
  if (n == 0) return;

  if (messageToReceive.cp_all.size() != n || messageToReceive.np_all.size() != n ||
      messageToReceive.wt_all.size() != n) {
    ROS_WARN_THROTTLE(2.0, "[PiPicoDriver] CP/NP/WT length mismatch cp=%zu np=%zu wt=%zu (using min=%zu)",
                      messageToReceive.cp_all.size(), messageToReceive.np_all.size(),
                      messageToReceive.wt_all.size(), n);
  }

  pi_pico_driver::RadioNetworkTable tbl;
  tbl.header.stamp = ros::Time::now();
  tbl.header.frame_id = "radio_network";
  tbl.num_robots = static_cast<uint8_t>(std::min(n, static_cast<size_t>(255)));
  tbl.cp.resize(n);
  tbl.np.resize(n);
  tbl.waiting.resize(n);
  tbl.timeline_index.resize(n);
  for (size_t i = 0; i < n; ++i) {
    tbl.cp[i] = static_cast<uint8_t>(std::min(messageToReceive.cp_all[i], static_cast<uint32_t>(255)));
    tbl.np[i] = static_cast<uint8_t>(std::min(messageToReceive.np_all[i], static_cast<uint32_t>(255)));
    tbl.waiting[i] = (messageToReceive.wt_all[i] != 0);
    const uint32_t ti = (i < messageToReceive.ti_all.size()) ? messageToReceive.ti_all[i] : 0u;
    tbl.timeline_index[i] = static_cast<uint8_t>(std::min(ti, static_cast<uint32_t>(255)));
  }
  networkTablePub_.publish(tbl);

  if (robot_id_ >= 0 && static_cast<size_t>(robot_id_) < n) {
    const bool w = tbl.waiting[static_cast<size_t>(robot_id_)];
    if (!wait_state_initialized_ || w != last_published_wait_state_) {
      std_msgs::Bool ws;
      ws.data = w;
      waitStatePub_.publish(ws);
      wait_state_initialized_ = true;
      last_published_wait_state_ = w;
      ROS_INFO_THROTTLE(1.0, "[PiPicoDriver] /pi_pico_driver/wait_state=%d (robot_id=%d)",
                        w ? 1 : 0, robot_id_);
    }
  }
}
