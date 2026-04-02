#include "pico_driver_node.h"
#include <cmath>
#include <cctype>

PiPicoDriver::PiPicoDriver(ros::NodeHandle& nh_) : nh(nh_) {

  // ---------------------- Init structs --------------------
  messageToSend.v_d = 0.0;
  messageToSend.w_d = 0.0;
  messageToSend.pick_box = false;
  messageToSend.cp = 0;
  messageToSend.final_node = 0;
  messageToSend.status = 0;
  messageToSend.wait_target = -2;

  messageToReceive.odom_pos.x = 0.0;
  messageToReceive.odom_pos.y = 0.0;
  messageToReceive.odom_pos.theta = 0.0;
  messageToReceive.v_linear = 0.0;
  messageToReceive.w_angular = 0.0;

  // ----------------------- ROS init -----------------------
  // -> Subs
  velSub = nh.subscribe("/cmd_vel", 10, &PiPicoDriver::velCallBack, this);
  pickBoxSub = nh.subscribe("/pick_box", 10, &PiPicoDriver::pickBoxCallBack, this);
  plannedPathSub = nh.subscribe("/planned_paths", 10, &PiPicoDriver::plannedPathCallBack, this);
  navFeedbackSub = nh.subscribe("/nav_completion_feedback", 20, &PiPicoDriver::navFeedbackCallBack, this);
  radioWaitTargetSub = nh.subscribe("/radio_wait_target", 10, &PiPicoDriver::radioWaitTargetCallBack, this);
  // -> Pubs
  posePub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
  robotIdPub = nh.advertise<std_msgs::Int32>("/robot_identity", 1, true);
  radioWaitReleasePub = nh.advertise<std_msgs::Bool>("/radio_wait_release", 10);
  // -> Timer (100 Hz para evitar watchdog timeout no Pico)
  commTimer = nh.createTimer(ros::Duration(0.01), &PiPicoDriver::commTick, this);

  // ---------------------- Serial init ---------------------
  std::string serial_port;
  nh.param<std::string>("serial_port", serial_port, "/dev/ttyACM0");
  nh.param("debug_comm", debug_comm_, false);  // Por padrão, logs desativados
  nh.param("debug_identity", debug_identity_, false);
  nh.param("debug_radio", debug_radio_, false);

  ROS_INFO("[PiPicoDriver] Usando porta serial: %s", serial_port.c_str());
  ROS_INFO("[PiPicoDriver] debug_comm=%s debug_identity=%s debug_radio=%s",
           debug_comm_ ? "on" : "off", debug_identity_ ? "on" : "off", debug_radio_ ? "on" : "off");
  
  serial_fd_ = -1;
  startSerial(serial_port);
  tryReadBootIdentity();
  
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

void PiPicoDriver::plannedPathCallBack(const std_msgs::Int32MultiArray::ConstPtr& msg) {
  if (msg->data.empty()) {
    return;
  }

  active_path_nodes_.assign(msg->data.begin(), msg->data.end());
  active_path_index_ = 0;
  messageToSend.cp = active_path_nodes_.front();
  messageToSend.final_node = active_path_nodes_.back();
}

void PiPicoDriver::navFeedbackCallBack(const plan_handler::CompletionFeedback::ConstPtr&) {
  if (active_path_nodes_.empty()) {
    return;
  }

  if (active_path_index_ + 1 < active_path_nodes_.size()) {
    active_path_index_++;
    messageToSend.cp = active_path_nodes_[active_path_index_];
  }
}

void PiPicoDriver::radioWaitTargetCallBack(const std_msgs::Int32::ConstPtr& msg) {
  const int waiter = msg->data;
  /* waiter = robô que está em espera; ele não envia o seu próprio wake no rádio. */
  if (waiter >= 0 && waiter == robot_id_) {
    if (debug_radio_) {
      ROS_INFO_THROTTLE(2.0, "[PiPicoDriver] /radio_wait_target=%d (waiter): este nó não transmite", waiter);
    }
    return;
  }
  if (waiter < -1) {
    ROS_WARN_THROTTLE(5.0, "[PiPicoDriver] /radio_wait_target inválido: %d", waiter);
    return;
  }
  messageToSend.wait_target = waiter;
  wait_release_pending_ = true;
  if (debug_radio_) {
    ROS_INFO("[PiPicoDriver] /radio_wait_target=%d (meu id=%d) -> WAITTO na série", waiter, robot_id_);
  }
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
  int parsed_id = -1;
  if (decodeIdentityLine(msg, parsed_id)) {
    applyRobotIdentity(parsed_id);
    return;
  }

  // Procurar por "POS:" em qualquer posição da mensagem (não só no início)
  size_t pos_idx = msg.find("POS:");
  if (pos_idx != std::string::npos) {
    std::string pos_part = msg.substr(pos_idx);

    double x = 0, y = 0, theta = 0;
    double v = 0.0, w = 0.0;

    int wait_release = 0;
    int n = std::sscanf(
      pos_part.c_str(),
      "POS: %lf, %lf, %lf, V: %lf, W: %lf",
      &x, &y, &theta, &v, &w
    );
    size_t wait_key = msg.find(", WAIT:");
    if (wait_key != std::string::npos) {
      std::sscanf(msg.c_str() + wait_key, ", WAIT: %d", &wait_release);
    }

    if (n >= 5) {
      int emb_id = -1;
      if (tryParseEmbeddedRobotId(msg, emb_id)) {
        applyRobotIdentity(emb_id);
      } else if (robot_id_ < 0 && debug_identity_) {
        ROS_WARN_THROTTLE(10.0,
                          "[PiPicoDriver] POS sem ', ID:' e id desconhecido (msg truncada): %.80s",
                          msg.c_str());
      }

      messageToReceive.odom_pos = {x, y, theta};
      messageToReceive.v_linear = v;
      messageToReceive.w_angular = w;
      pubOdom();
      if (wait_release != 0) {
        if (debug_radio_) {
          ROS_INFO("[PiPicoDriver] POS WAIT:%d -> /radio_wait_release", wait_release);
        }
        std_msgs::Bool wait_msg;
        wait_msg.data = true;
        radioWaitReleasePub.publish(wait_msg);
      }
      return;
    }

    ROS_WARN_THROTTLE(5.0, "Mensagem POS mal formatada (esperado x,y,theta,V,W): %s", pos_part.c_str());
    return;
  }

  // Ignora ACK e mensagens vazias/curtas
  if (msg.rfind("ACK:", 0) == 0) return;
  if (msg.empty() || msg.length() < 3) return;
  
  // Mensagens de debug do Pico (longas) - ignorar silenciosamente se não em modo debug
  if (!debug_comm_ && msg.find("dbg") != std::string::npos) return;

  if (debug_identity_) {
    ROS_WARN_THROTTLE(10.0, "[PiPicoDriver] Mensagem desconhecida: %.200s", msg.c_str());
  } else {
    ROS_WARN_THROTTLE(10.0, "Mensagem desconhecida (mostrando 1 a cada 10s): %s", msg.substr(0, 80).c_str());
  }
}

void PiPicoDriver::commTick(const ros::TimerEvent&) {
  updateMotionStatus();
  
  // 1) Build the command
  std::string cmd = "CMD:" + std::to_string(messageToSend.v_d) + "," +
                             std::to_string(messageToSend.w_d) + "," +
                                           (messageToSend.pick_box ? "1" : "0") +
                    " CP:" + std::to_string(messageToSend.cp) +
                    " FINAL:" + std::to_string(messageToSend.final_node) +
                    " STATUS:" + std::to_string(messageToSend.status) +
                    " WAITTO:" + std::to_string(wait_release_pending_ ? messageToSend.wait_target : -2);

  // 2) Send and Wait for response
  if (debug_comm_) {
    ROS_INFO("[PiPicoDriver] Pi4 -> Pico CMD: %s", cmd.c_str());
  }
  if (debug_radio_ && wait_release_pending_) {
    ROS_INFO_THROTTLE(0.5, "[PiPicoDriver] enviando CMD com WAITTO=%d", messageToSend.wait_target);
  }

  std::string resp = syncCall(cmd, 50);

  if (debug_comm_) {
    ROS_INFO("[PiPicoDriver] Pico -> Pi4 linha (primeira): %s", resp.c_str());
    ROS_INFO("[PiPicoDriver] --- fim tick ---");
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
  con_state.missed = 0; con_state.link_ok = true;
  wait_release_pending_ = false;
  messageToSend.wait_target = -2;

}

void PiPicoDriver::updateMotionStatus() {
  const bool moving = (std::abs(messageToSend.v_d) > 1e-4) || (std::abs(messageToSend.w_d) > 1e-4);
  messageToSend.status = moving ? 1 : 0;
}

void PiPicoDriver::tryReadBootIdentity() {
  if (serial_fd_ < 0) {
    return;
  }

  ros::Time start = ros::Time::now();
  ros::Duration timeout(2.0);
  std::string response;
  char buf[256];

  while (ros::Time::now() - start < timeout) {
    int n = read(serial_fd_, buf, sizeof(buf) - 1);
    if (n <= 0) {
      continue;
    }

    buf[n] = '\0';
    response += std::string(buf);
    size_t newline_pos = response.find('\n');
    while (newline_pos != std::string::npos) {
      std::string line = response.substr(0, newline_pos);
      int id = -1;
      if (decodeIdentityLine(line, id)) {
        applyRobotIdentity(id);
        return;
      }
      response.erase(0, newline_pos + 1);
      newline_pos = response.find('\n');
    }
  }
  if (robot_id_ < 0) {
    ROS_WARN_THROTTLE(30.0, "[PiPicoDriver] sem ID: no boot (usa-se , ID: na POS).");
  }
}

bool PiPicoDriver::decodeIdentityLine(const std::string& line, int& out_id) const {
  std::string s = line;
  while (!s.empty() && (s.back() == '\r' || s.back() == '\n')) {
    s.pop_back();
  }
  const std::string prefix = "ID:";
  if (s.rfind(prefix, 0) != 0) {
    return false;
  }

  try {
    out_id = std::stoi(s.substr(prefix.size()));
    return true;
  } catch (...) {
    return false;
  }
}

bool PiPicoDriver::tryParseEmbeddedRobotId(const std::string& msg, int& out_id) const {
  const std::string key = ", ID:";
  const size_t p = msg.rfind(key);
  if (p == std::string::npos) {
    return false;
  }
  size_t start = p + key.size();
  while (start < msg.size() && (msg[start] == ' ' || msg[start] == '\t')) {
    ++start;
  }
  size_t end = start;
  while (end < msg.size() && (std::isdigit(static_cast<unsigned char>(msg[end])) || msg[end] == '-')) {
    ++end;
  }
  if (end == start) {
    return false;
  }
  try {
    out_id = std::stoi(msg.substr(start, end - start));
    return true;
  } catch (...) {
    return false;
  }
}

void PiPicoDriver::applyRobotIdentity(int id) {
  if (id < 0) {
    return;
  }
  robot_id_ = id;
  std_msgs::Int32 id_msg;
  id_msg.data = robot_id_;
  robotIdPub.publish(id_msg);
  if (!identity_logged_once_) {
    ROS_INFO("[PiPicoDriver] robot_identity=%d (publicado em /robot_identity)", robot_id_);
    identity_logged_once_ = true;
  }
}