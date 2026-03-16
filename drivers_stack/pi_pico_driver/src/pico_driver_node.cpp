#include "pico_driver_node.h"


PiPicoDriver::PiPicoDriver(ros::NodeHandle& nh_) : nh(nh_) {

  // ---------------------- Init structs --------------------
  messageToSend.v_d = 0.0;
  messageToSend.w_d = 0.0;
  messageToSend.pick_box = false;
  messageToSend.cp_send = 0; 
  messageToSend.path_send.clear();
  
  messageToReceive.odom_pos.x = 0.0;
  messageToReceive.odom_pos.y = 0.0;
  messageToReceive.odom_pos.theta = 0.0;
  messageToReceive.v_linear = 0.0;
  messageToReceive.w_angular = 0.0;
  messageToReceive.cp_rcv = 0;
  messageToReceive.path_rcv.clear();
 

  // ----------------------- ROS init -----------------------
  // -> Subs
  velSub = nh.subscribe("/cmd_vel", 10, &PiPicoDriver::velCallBack, this);
  pickBoxSub = nh.subscribe("/pick_box", 10, &PiPicoDriver::pickBoxCallBack, this);
  cpSendSub = nh.subscribe("/cp_send", 10, &PiPicoDriver::cpSendCallBack, this);
  pathSendSub = nh.subscribe("/path_send", 10, &PiPicoDriver::pathSendCallBack, this);
  // -> Pubs
  posePub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
  cpRcvPub = nh.advertise<std_msgs::UInt32>("/cp_rcv", 10);
  pathRcvPub = nh.advertise<std_msgs::Int32MultiArray>("/path_rcv", 10);
  // -> Timer (100 Hz para evitar watchdog timeout no Pico)
  commTimer = nh.createTimer(ros::Duration(0.01), &PiPicoDriver::commTick, this);

  // ---------------------- Serial init ---------------------
  std::string serial_port;
  nh.param<std::string>("serial_port", serial_port, "/dev/ttyACM0");
  nh.param("debug_comm", debug_comm_, false);  // Por padrão, logs desativados
  
  ROS_INFO("[PiPicoDriver] Usando porta serial: %s", serial_port.c_str());
  ROS_INFO("[PiPicoDriver] Debug communication logs: %s", debug_comm_ ? "ENABLED" : "DISABLED");
  
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

  // ---- BLOQUEANTE ----
  tty.c_cc[VMIN]  = 1;  // espera pelo menos 1 byte
  tty.c_cc[VTIME] = 1;  // timeout de 100 ms
  // --------------------

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

  messageToSend.pick_box = msg->data; 
  
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

  odom.twist.twist.linear.x  = messageToReceive.v_linear;
  odom.twist.twist.angular.z = messageToReceive.w_angular;

  posePub.publish(odom);

  // Publish TF transform odom -> base_link
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
  ros::Duration poll_interval(0.01);

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
  if (msg.rfind("ACK:", 0) == 0) return;
  if (msg.empty() || msg.length() < 3) return;
  if (!debug_comm_ && msg.find("dbg") != std::string::npos) return;

  bool found_any = false;
  bool has_new_cp = false;
  bool has_new_path = false;

  // ---------------- POS / V / W ----------------
  size_t pos_idx = msg.find("POS:");
  if (pos_idx != std::string::npos) {
    double x = 0.0, y = 0.0, theta = 0.0;
    double v = 0.0, w = 0.0;

    int n = std::sscanf(
      msg.c_str(),
      "POS: %lf, %lf, %lf, V: %lf, W: %lf",
      &x, &y, &theta, &v, &w
    );

    if (n == 5) {
      messageToReceive.odom_pos = {x, y, theta};
      messageToReceive.v_linear = v;
      messageToReceive.w_angular = w;
      pubOdom();
      found_any = true;
    }
  }

  // ---------------- CP ----------------
  size_t cp_idx = msg.find("CP:");
  if (cp_idx != std::string::npos) {
    unsigned int cp_val = 0;
    if (std::sscanf(msg.c_str() + cp_idx, "CP: %u", &cp_val) == 1) {
      messageToReceive.cp_rcv = cp_val;
      has_new_cp = true;
      found_any = true;
    }
  }

  // ---------------- PATH ----------------
  size_t path_idx = msg.find("PATH:");
  if (path_idx != std::string::npos) {
    std::string path_part = msg.substr(path_idx + 5);

    while (!path_part.empty() && path_part.front() == ' ') {
      path_part.erase(path_part.begin());
    }

    while (!path_part.empty() &&
           (path_part.back() == '\n' || path_part.back() == '\r' || path_part.back() == ' ')) {
      path_part.pop_back();
    }

    // Só processa/publica se PATH não vier vazio
    if (!path_part.empty()) {
      messageToReceive.path_rcv = parsePathList(path_part);
      has_new_path = true;
      found_any = true;
    }
  }

  // Publica CP apenas se chegou novo CP
  if (has_new_cp) {
    std_msgs::UInt32 cp_msg;
    cp_msg.data = messageToReceive.cp_rcv;
    cpRcvPub.publish(cp_msg);
  }

  // Publica PATH apenas se chegou PATH não vazio
  if (has_new_path) {
    std_msgs::Int32MultiArray path_msg;
    path_msg.data = messageToReceive.path_rcv;
    pathRcvPub.publish(path_msg);
  }

  if (!found_any) {
    ROS_WARN_THROTTLE(10.0,
                      "Mensagem desconhecida (mostrando 1 a cada 10s): %s",
                      msg.substr(0, 80).c_str());
  }
}

void PiPicoDriver::commTick(const ros::TimerEvent&) {

  // Decide que PATH enviar nesta iteração
  std::vector<int32_t> path_once;
  if (path_send_retries_ > 0) {
    path_once = path_to_send_;
    path_send_retries_--;
  } else {
    path_once.clear();  // envia PATH vazio
  }

  // 1) Build the command
  std::string cmd = "CMD:" + std::to_string(messageToSend.v_d) + "," +
                           std::to_string(messageToSend.w_d) + "," +
                           (messageToSend.pick_box ? "1" : "0") +
                    " CP:" + std::to_string(messageToSend.cp_send) +
                    " PATH:" + pathToString(path_once);

  if (debug_comm_) {
    ROS_INFO("Pi4 Message: %s", cmd.c_str());
  }

  // 2) Send and wait for response
  std::string resp = syncCall(cmd, 50);

  if (debug_comm_) {
    ROS_INFO("PiPico Message: %s", resp.c_str());
    ROS_INFO("-------------------");
  }

  if (resp.empty()) {
    con_state.missed++;
    if (con_state.missed >= 2) {
      con_state.link_ok = false;
      if (!debug_comm_) {
        ROS_WARN_THROTTLE(5.0, "[PiPicoDriver] Connection lost with Pico!");
      }
    }
    return;
  }

  con_state.missed = 0;
  con_state.link_ok = true;
}

void PiPicoDriver::cpSendCallBack(const std_msgs::UInt32::ConstPtr& msg) {
  messageToSend.cp_send = msg->data;
}

void PiPicoDriver::pathSendCallBack(const std_msgs::Int32MultiArray::ConstPtr& msg) {
  path_to_send_ = msg->data;
  path_send_retries_ = 5;
}

void PiPicoDriver::pubExtraMsgs() {
  std_msgs::UInt32 cp_msg;
  cp_msg.data = messageToReceive.cp_rcv;
  cpRcvPub.publish(cp_msg);

  std_msgs::Int32MultiArray path_msg;
  path_msg.data = messageToReceive.path_rcv;
  pathRcvPub.publish(path_msg);
}

std::string PiPicoDriver::pathToString(const std::vector<int32_t>& path) {
  std::string result;
  for (size_t i = 0; i < path.size(); ++i) {
    result += std::to_string(path[i]);
    if (i + 1 < path.size()) {
      result += ",";
    }
  }
  return result;
}

std::vector<int32_t> PiPicoDriver::parsePathList(const std::string& s) {
  std::vector<int32_t> result;
  std::stringstream ss(s);
  std::string item;

  while (std::getline(ss, item, ',')) {
    while (!item.empty() && item.front() == ' ') {
      item.erase(item.begin());
    }
    while (!item.empty() && item.back() == ' ') {
      item.pop_back();
    }

    if (!item.empty()) {
      try {
        result.push_back(std::stoi(item));
      } catch (...) {
        ROS_WARN("Valor inválido em PATH: %s", item.c_str());
      }
    }
  }

  return result;
}