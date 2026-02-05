#include "wifi_driver_node.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <cctype>

WifiDriverNode::WifiDriverNode(ros::NodeHandle& nh)
  : nh_(nh)
{
    // Defaults
  server_ip_ = "192.168.50.241";
  port_ = 44832;
  timeout_s_ = 1.0;

  // Carregar do parameter server (sobrescreve se existir)
  nh_.param("server_ip", server_ip_, server_ip_);
  nh_.param("port", port_, port_);
  nh_.param("timeout", timeout_s_, timeout_s_);

  ROS_INFO("Config: server_ip=%s port=%d timeout=%.2f",
          server_ip_.c_str(), port_, timeout_s_);

  // publica sequencia de cores
  color_pub_ = nh_.advertise<std_msgs::String>("/color_sequence", 10, true);

  sock_fd_ = -1;

  if (!setupSocket())
  {
    ROS_ERROR("Falha a criar socket UDP.");
  }

  // Timer: evita bloquear o ROS
  timer_ = nh_.createTimer(ros::Duration(1.0), &WifiDriverNode::timerCb, this);

  start_iwp_srv_ = nh_.advertiseService("start_iwp", &WifiDriverNode::startIwpCb, this);
  ROS_INFO("Service 'start_iwp' advertised. Call with data:=true to start IWP.");


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

    timer_.stop();
    ROS_INFO("Sequencia recebida. Vou parar de pedir IWP.");
  }
  else
  {
    ROS_WARN("[RX] Resposta invalida IWP: '%s'", resp.c_str());
  }
}

void WifiDriverNode::timerCb(const ros::TimerEvent&)
{
  if (!iwp_enabled_)
  {
    ROS_INFO_THROTTLE(5.0, "Desativado. Aguardando service start_iwp.");
    return;
  }

  // --- 1) CTL SEMPRE ---
  if (!sendMsg("CTL"))
  {
    ROS_WARN_THROTTLE(3.0, "Falha ao enviar CTL.");
    return;
  }

  std::string resp_ctl;
  if (!recvMsg(resp_ctl))
  {
    ROS_WARN_THROTTLE(3.0, "Timeout à espera de resposta a CTL.");
    return;
  }

  int tval = -1;
  if (!parseTxxx(resp_ctl, tval))
  {
    ROS_WARN("[RX] Esperava T### de CTL, recebi: '%s'", resp_ctl.c_str());
    return;
  }

  ROS_INFO_THROTTLE(2.0, "[RX] %s (=%d)", resp_ctl.c_str(), tval);

  // --- 2) Só começa IWP quando T < 600 ---
  if (!have_sequence_ && tval < 600)
  {
    iwp_active_ = true;
  }

  // --- 3) Enquanto iwp_active_ e sem sequência, manda IWP ---
  if (iwp_active_ && !have_sequence_)
  {
    if (!sendMsg("IWP"))
    {
      ROS_WARN_THROTTLE(3.0, "Falha ao enviar IWP.");
      return;
    }

    std::string resp_iwp;
    if (!recvMsg(resp_iwp))
    {
      ROS_WARN_THROTTLE(3.0, "Timeout à espera de resposta a IWP.");
      return;
    }

    if (isColorSeq4(resp_iwp))
    {
      color_sequence_ = resp_iwp;
      have_sequence_ = true;
      iwp_active_ = false; // --- 4) PARA DE MANDAR IWP ---
      ROS_INFO("[RX] Sequencia recebida via IWP: %s (vou parar de pedir IWP)", color_sequence_.c_str());

      // Publicar imediatamente (já estamos em modo <600 porque só aí ativaste IWP)
      if (color_sequence_ != last_published_)
      {
        std_msgs::String msg;
        msg.data = color_sequence_;
        color_pub_.publish(msg);
        last_published_ = color_sequence_;
        ROS_INFO("[PUB] /color_sequence = %s", color_sequence_.c_str());
      }
    }
    else
    {
      ROS_WARN_THROTTLE(3.0, "[RX] Resposta invalida a IWP: '%s'", resp_iwp.c_str());
    }
  }
}



bool WifiDriverNode::startIwpCb(std_srvs::SetBool::Request& req,
                               std_srvs::SetBool::Response& res)
{
  iwp_enabled_ = req.data;

  if (iwp_enabled_)
  {
    ROS_INFO("IWP ENABLED by service (start).");
    // opcional: reset estado
    // last_published_.clear();
    // timer_.start();  // só se você tinha parado antes
    res.success = true;
    res.message = "IWP enabled";
  }
  else
  {
    ROS_INFO("IWP DISABLED by service (stop).");
    res.success = true;
    res.message = "IWP disabled";
  }

  return true;
}

bool WifiDriverNode::parseTxxx(const std::string& s, int& out_val)
{
  if (s.size() != 4) return false;
  if (s[0] != 'T') return false;
  if (!isdigit(s[1]) || !isdigit(s[2]) || !isdigit(s[3])) return false;
  out_val = (s[1]-'0')*100 + (s[2]-'0')*10 + (s[3]-'0');
  return true;
}

bool WifiDriverNode::isColorSeq4(const std::string& s)
{
  if (s.size() != 4) return false;
  auto ok = [](char c){ return c=='R' || c=='G' || c=='B'; };
  return ok(s[0]) && ok(s[1]) && ok(s[2]) && ok(s[3]);
}
