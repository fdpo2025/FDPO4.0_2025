#include "wifi_driver_node.h"

#include <sys/socket.h>

namespace {
void publishColorBoth(ros::Publisher& main_pub, ros::Publisher& dup_pub, const std_msgs::String& msg) {
  main_pub.publish(msg);
  dup_pub.publish(msg);
}
}  // namespace
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

  std::string iwp_dup_topic;
  nh_.param<std::string>("iwp_duplicate_topic", iwp_dup_topic, std::string("/wifi_iwp_color_sequence"));

  // publica sequencia de cores
  color_pub_ = nh_.advertise<std_msgs::String>("/color_sequence", 10, true);
  color_pub_iwp_dup_ = nh_.advertise<std_msgs::String>(iwp_dup_topic, 10, true);

  sock_fd_ = -1;

  if (!setupSocket())
  {
    ROS_ERROR("Falha a criar socket UDP.");
  }

  // Timer: evita bloquear o ROS
  timer_ = nh_.createTimer(ros::Duration(1.0), &WifiDriverNode::timerCb, this);

  start_iwp_srv_ = nh_.advertiseService("start_iwp", &WifiDriverNode::startIwpCb, this);
  ROS_INFO("Service 'start_iwp' advertised. Call with data:=true to start IWP.");


  ROS_INFO("WifiDriverNode pronto. Publica /color_sequence e duplicado em %s (pico bridge).",
           iwp_dup_topic.c_str());
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

  sockaddr_in from;
  socklen_t from_len = sizeof(from);

  int n = recvfrom(sock_fd_, buffer, sizeof(buffer) - 1, 0,
                   (sockaddr*)&from, &from_len);

  if (n < 0) return false;

  out = std::string(buffer);
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
    publishColorBoth(color_pub_, color_pub_iwp_dup_, msg);

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

  // Se ainda não chegaste ao ponto (T < 600), só faz CTL
  if (!iwp_active_ && !have_sequence_)
  {
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
      ROS_WARN_THROTTLE(3.0, "[RX] Esperava T### de CTL, recebi: '%s'", resp_ctl.c_str());
      return;
    }

    ROS_INFO_THROTTLE(2.0, "[RX] %s (=%d)", resp_ctl.c_str(), tval);

    if (tval < 600)
    {
      iwp_active_ = true;
      ROS_INFO("T < 600. Vou começar a mandar IWP.");
    }

    return; // IMPORTANTÍSSIMO: neste tick só fizeste CTL
  }

  // Se já estás em modo IWP (T < 600) e ainda não tens sequência, só faz IWP
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
      iwp_active_ = false; // para de mandar IWP

      ROS_INFO("[RX] Sequencia recebida: %s (vou parar de pedir IWP)", color_sequence_.c_str());

      if (color_sequence_ != last_published_)
      {
        std_msgs::String msg;
        msg.data = color_sequence_;
        publishColorBoth(color_pub_, color_pub_iwp_dup_, msg);
        last_published_ = color_sequence_;
        ROS_INFO("[PUB] /color_sequence = %s", color_sequence_.c_str());
      }
    }
    else if (resp_iwp == "STOP")
    {
      ROS_INFO_THROTTLE(3.0, "[RX] STOP recebido. Continuo a tentar IWP.");
    }
    else
    {
      ROS_WARN_THROTTLE(3.0, "[RX] Resposta invalida a IWP: '%s'", resp_iwp.c_str());
    }

    return;
  }

  // Se já tens sequência, aqui decides o que fazer:
  // - ou não fazes mais nada,
  // - ou continuas a mandar CTL por monitorização.
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
    have_sequence_ = false;
    iwp_active_ = false;
    color_sequence_.clear();
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
