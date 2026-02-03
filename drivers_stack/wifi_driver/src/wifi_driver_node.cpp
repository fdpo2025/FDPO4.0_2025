#include "wifi_driver_node.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

WifiDriverNode::WifiDriverNode(ros::NodeHandle& nh)
  : nh_(nh)
{
  nh_.param<std::string>("server_ip", server_ip_, std::string("192.168.50.241"));
  nh_.param<int>("port", port_, 44832);
  nh_.param<double>("timeout", timeout_s_, 1.0);

  // publica sequencia de cores
  color_pub_ = nh_.advertise<std_msgs::String>("/color_sequence", 10, true);

  sock_fd_ = -1;

  if (!setupSocket())
  {
    ROS_ERROR("Falha a criar socket UDP.");
  }

  // Timer: evita bloquear o ROS
  timer_ = nh_.createTimer(ros::Duration(1.0), &WifiDriverNode::timerCb, this);

  ROS_INFO("WifiDriverNode pronto. Vai publicar em /color_sequence quando receber != STOP");
}

bool WifiDriverNode::setupSocket()
{
  sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd_ < 0)
  {
    ROS_ERROR("socket() falhou.");
    return false;
  }

  // configurar timeout do recv
  timeval tv;
  tv.tv_sec = static_cast<int>(timeout_s_);
  tv.tv_usec = static_cast<int>((timeout_s_ - tv.tv_sec) * 1e6);
  setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  std::memset(&server_addr_, 0, sizeof(server_addr_));
  server_addr_.sin_family = AF_INET;
  server_addr_.sin_port = htons(port_);

  if (inet_pton(AF_INET, server_ip_.c_str(), &server_addr_.sin_addr) <= 0)
  {
    ROS_ERROR("inet_pton falhou para IP: %s", server_ip_.c_str());
    return false;
  }

  ROS_INFO("UDP configurado: %s:%d timeout=%.2fs", server_ip_.c_str(), port_, timeout_s_);
  return true;
}

bool WifiDriverNode::sendMsg(const std::string& msg)
{
  if (sock_fd_ < 0) return false;

  int n = sendto(sock_fd_, msg.c_str(), msg.size(), 0,
                 (sockaddr*)&server_addr_, sizeof(server_addr_));

  return (n >= 0);
}

bool WifiDriverNode::recvMsg(std::string& out)
{
  if (sock_fd_ < 0) return false;

  char buffer[1024];
  std::memset(buffer, 0, sizeof(buffer));

  socklen_t addr_len = sizeof(server_addr_);
  int n = recvfrom(sock_fd_, buffer, sizeof(buffer) - 1, 0,
                   (sockaddr*)&server_addr_, &addr_len);

  if (n < 0)
  {
    // timeout ou erro
    return false;
  }

  out = std::string(buffer);
  // trim \r\n / espaços
  while (!out.empty() && (out.back() == '\n' || out.back() == '\r' || out.back() == ' '))
    out.pop_back();

  return true;
}

void WifiDriverNode::doPingPong()
{
  ROS_INFO_THROTTLE(5.0, "[FASE 1] A testar conexao UDP com PING/PONG...");

  sendMsg("PING");

  std::string resp;
  if (recvMsg(resp))
  {
    if (resp == "PONG")
    {
      connected_ = true;
      ROS_INFO("[RX] PONG recebido. Servidor online.");
    }
    else
    {
      ROS_WARN("[RX] Resposta inesperada a PING: '%s'", resp.c_str());
    }
  }
  else
  {
    ROS_WARN_THROTTLE(3.0, "Sem resposta ao PING (timeout).");
  }
}

void WifiDriverNode::doIWP()
{
  // pergunta sequencia
  sendMsg("IWP");

  std::string resp;
  if (!recvMsg(resp))
  {
    ROS_WARN_THROTTLE(3.0, "Timeout durante IWP.");
    return;
  }

  if (resp == "STOP")
  {
    ROS_INFO_THROTTLE(3.0, "[RX] STOP recebido. Nao publica.");
    return;
  }

  if (resp.size() == 4)
  {
    // evita publicar repetido
    if (resp == last_published_)
    {
      ROS_INFO_THROTTLE(2.0, "Sequencia repetida (%s). Ignorar.", resp.c_str());
      return;
    }

    std_msgs::String msg;
    msg.data = resp;
    color_pub_.publish(msg);

    last_published_ = resp;
    ROS_INFO("[PUB] /color_sequence = %s", resp.c_str());
  }
  else
  {
    ROS_WARN("[RX] Resposta invalida IWP: '%s'", resp.c_str());
  }
}

void WifiDriverNode::timerCb(const ros::TimerEvent&)
{
  if (!connected_)
  {
    doPingPong();
    return;
  }

  doIWP();
}
