// ydlidar_x4_sdk_node.cpp
//
// Versão “ROS-completa” (no sentido do que o SdpoDriverLaser2DROS.cpp tem e
// faltava no teu ydlidar_x4): base_frame_id + laser_frame_id + extrínsecos +
// broadcast TF + filtros dist/ângulo via params + serviço para recarregar params.
//
// Continua a publicar PointCloud em "laser_scan_point_cloud" como o SdpoDriver.

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>

#include <cmath>
#include <string>
#include <limits>

// SDK
#include "CYdLidar.h"
#include "core/common/ydlidar_def.h"
#include "core/common/ydlidar_datatype.h"

static inline float normAngRad(float a) {
  // Normaliza para [-pi, pi)
  const float two_pi = 2.0f * static_cast<float>(M_PI);
  while (a >= static_cast<float>(M_PI)) a -= two_pi;
  while (a < -static_cast<float>(M_PI)) a += two_pi;
  return a;
}

class YDLidarX4SdkNode {
public:
  YDLidarX4SdkNode() : nh_(), pnh_("~") {
    // Params do teu nó original
    pnh_.param<std::string>("port", port_, std::string("/dev/ttyUSB0"));
    pnh_.param<int>("baudrate", baudrate_, 128000);
    pnh_.param<float>("frequency", frequency_, 5.0f);

    // Compatibilidade: antigo "frame_id" (se existir) vira laser_frame_id
    std::string legacy_frame_id = "laser";
    pnh_.param<std::string>("frame_id", legacy_frame_id, legacy_frame_id);

    // Params “ao estilo SdpoDriver”
    pnh_.param<std::string>("base_frame_id", base_frame_id_, std::string("base_footprint"));
    pnh_.param<std::string>("laser_frame_id", laser_frame_id_, legacy_frame_id);

    // Extrínsecos (em metros e graus, como no SdpoDriver)
    pnh_.param<float>("laser_pose_x", laser_pose_x_, 0.0f);
    pnh_.param<float>("laser_pose_y", laser_pose_y_, 0.0f);
    pnh_.param<float>("laser_pose_z", laser_pose_z_, 0.0f);
    pnh_.param<float>("laser_pose_yaw",   laser_pose_yaw_,   0.0f);
    pnh_.param<float>("laser_pose_pitch", laser_pose_pitch_, 0.0f);
    pnh_.param<float>("laser_pose_roll",  laser_pose_roll_,  0.0f);

    ROS_INFO("Base frame: %s | Laser frame: %s",
            base_frame_id_.c_str(), laser_frame_id_.c_str());

    ROS_INFO("Laser > Base footprint: "
            "[%f, %f, %f] m , [%f %f %f] deg",
            laser_pose_x_, laser_pose_y_, laser_pose_z_,
            laser_pose_yaw_, laser_pose_pitch_, laser_pose_roll_);

    // Igual ao SdpoDriver: converte graus->rad e guarda internamente
    laser_pose_yaw_   *= static_cast<float>(M_PI) / 180.0f;
    laser_pose_pitch_ *= static_cast<float>(M_PI) / 180.0f;
    laser_pose_roll_  *= static_cast<float>(M_PI) / 180.0f;

    ROS_INFO("Serial: %s @ %d", port_.c_str(), baudrate_);
    ROS_INFO("Scan frequency: %.2f Hz", frequency_);
    ROS_INFO("Base frame: %s | Laser frame: %s", base_frame_id_.c_str(), laser_frame_id_.c_str());
    ROS_INFO("Laser > Base: [%.3f, %.3f, %.3f] m, [yaw=%.2f pitch=%.2f roll=%.2f] deg",
             laser_pose_x_, laser_pose_y_, laser_pose_z_,
             laser_pose_yaw_deg_, laser_pose_pitch_deg_, laser_pose_roll_deg_);

    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud>("laser_scan_point_cloud", 1);

    // Serviço para “recarregar” extrínsecos/frames sem recompilar (substitui dynamic_reconfigure)
    srv_reload_ = pnh_.advertiseService("reload_params", &YDLidarX4SdkNode::reloadParamsCb, this);

    setupSdk();
  }

  ~YDLidarX4SdkNode() {
    try {
      lidar_.turnOff();
      lidar_.disconnecting();
    } catch (...) {}
  }

  void spin() {
    LaserScan scan; // tipo do SDK
    ros::Rate rate(20); // loop rápido; scan chega à freq do lidar

    while (ros::ok()) {
      if (lidar_.doProcessSimple(scan)) {
        publishPointCloudAndTf(scan);
      }
      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  void setupSdk() {
    // Config base (do teu ficheiro original)
    lidar_.setlidaropt(LidarPropSerialPort, port_.c_str(), (int)port_.size());
    lidar_.setlidaropt(LidarPropSerialBaudrate, &baudrate_, sizeof(int));

    int optval = TYPE_TRIANGLE; // X4
    lidar_.setlidaropt(LidarPropLidarType, &optval, sizeof(int));

    optval = YDLIDAR_TYPE_SERIAL;
    lidar_.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));

    optval = 5; // sample rate “code”
    lidar_.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));

    optval = 4; // abnormal check count
    lidar_.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

    bool b = true;
    lidar_.setlidaropt(LidarPropFixedResolution, &b, sizeof(bool));
    lidar_.setlidaropt(LidarPropAutoReconnect, &b, sizeof(bool));

    bool isSingleChannel = true;
    lidar_.setlidaropt(LidarPropSingleChannel, &isSingleChannel, sizeof(bool));

    // Mantém defaults do SDK (podes sobrepor com params ROS se quiseres)
    float f;
    f = 180.0f;  lidar_.setlidaropt(LidarPropMaxAngle, &f, sizeof(float));
    f = -180.0f; lidar_.setlidaropt(LidarPropMinAngle, &f, sizeof(float));
    f = 16.0f;   lidar_.setlidaropt(LidarPropMaxRange, &f, sizeof(float));
    f = 0.08f;   lidar_.setlidaropt(LidarPropMinRange, &f, sizeof(float));

    lidar_.setlidaropt(LidarPropScanFrequency, &frequency_, sizeof(float));

    if (!lidar_.initialize()) {
      ROS_FATAL("YDLidar SDK initialize() falhou (porta=%s baud=%d).",
                port_.c_str(), baudrate_);
      ros::shutdown();
      return;
    }
    if (!lidar_.turnOn()) {
      ROS_FATAL("YDLidar turnOn() falhou.");
      ros::shutdown();
      return;
    }

    ROS_INFO("YDLidar X4 ligado e a publicar.");
  }

  // --- troca o teu reloadParamsCb por este (fica igual ao SdpoDriver) ---

  bool reloadParamsCb(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    pnh_.param<std::string>("base_frame_id", base_frame_id_, base_frame_id_);
    pnh_.param<std::string>("laser_frame_id", laser_frame_id_, laser_frame_id_);

    pnh_.param<float>("laser_pose_x", laser_pose_x_, laser_pose_x_);
    pnh_.param<float>("laser_pose_y", laser_pose_y_, laser_pose_y_);
    pnh_.param<float>("laser_pose_z", laser_pose_z_, laser_pose_z_);
    pnh_.param<float>("laser_pose_yaw",   laser_pose_yaw_,   laser_pose_yaw_);
    pnh_.param<float>("laser_pose_pitch", laser_pose_pitch_, laser_pose_pitch_);
    pnh_.param<float>("laser_pose_roll",  laser_pose_roll_,  laser_pose_roll_);

    ROS_INFO("[ydlidar_x4_sdk_node] Laser > Base footprint updated: "
            "[%f, %f, %f] m , [%f %f %f] deg",
            laser_pose_x_, laser_pose_y_, laser_pose_z_,
            laser_pose_yaw_, laser_pose_pitch_, laser_pose_roll_);

    // Igual ao SdpoDriver: converte graus->rad e guarda internamente
    laser_pose_yaw_   *= static_cast<float>(M_PI) / 180.0f;
    laser_pose_pitch_ *= static_cast<float>(M_PI) / 180.0f;
    laser_pose_roll_  *= static_cast<float>(M_PI) / 180.0f;

    return true;
  }


  inline bool anglePass(float a_rad) const {
    if (!has_angle_range_) return true;
    // Intervalo simples [min, max] em rad (como SdpoDriver assume após swap)
    return (a_rad >= ang_min_rad_ && a_rad <= ang_max_rad_);
  }

  inline bool distPass(float r_m) const {
    if (!has_dist_range_) return true;
    return (r_m >= dist_min_ && r_m <= dist_max_);
  }

  void publishPointCloudAndTf(const LaserScan& scan) {
    sensor_msgs::PointCloud msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = laser_frame_id_;

    // Reserva máximo e depois faz push_back só do que passa filtros
    msg.points.clear();
    msg.points.reserve(scan.points.size());

    for (const auto& p : scan.points) {
      // p.angle em rad, p.range em m (segundo o teu comentário e uso original)
      const float a = normAngRad(static_cast<float>(p.angle));
      const float r = static_cast<float>(p.range);

      if (!std::isfinite(a) || !std::isfinite(r)) continue;
      if (!anglePass(a)) continue;
      if (!distPass(r)) continue;

      geometry_msgs::Point32 pt;
      pt.x = r * std::cos(a);
      pt.y = r * std::sin(a);
      pt.z = 0.0f;
      msg.points.push_back(pt);
    }

    // TF base -> laser (igual ao SdpoDriver)
    tf::StampedTransform laser2base_tf;
    laser2base_tf.setOrigin(tf::Vector3(laser_pose_x_, laser_pose_y_, laser_pose_z_));
    laser2base_tf.setRotation(tf::createQuaternionFromRPY(laser_pose_roll_, laser_pose_pitch_, laser_pose_yaw_));
    laser2base_tf.stamp_ = msg.header.stamp;
    laser2base_tf.frame_id_ = base_frame_id_;
    laser2base_tf.child_frame_id_ = laser_frame_id_;
    tf_broadcaster_.sendTransform(laser2base_tf);

    pub_cloud_.publish(msg);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher pub_cloud_;
  ros::ServiceServer srv_reload_;
  tf::TransformBroadcaster tf_broadcaster_;

  CYdLidar lidar_;

  // SDK params
  std::string port_;
  int baudrate_{128000};
  float frequency_{5.0f};

  // Frames e extrínsecos (estilo SdpoDriver)
 std::string base_frame_id_{"base_footprint"};
  std::string laser_frame_id_{"laser"};

  float laser_pose_x_{0.0f}, laser_pose_y_{0.0f}, laser_pose_z_{0.0f};
  // IMPORTANTE: estes começam em graus e depois ficam guardados em rad internamente
  float laser_pose_yaw_{0.0f}, laser_pose_pitch_{0.0f}, laser_pose_roll_{0.0f};

  // Filtros
  bool has_dist_range_{false};
  float dist_min_{0.0f}, dist_max_{0.0f};

  bool has_angle_range_{false};
  float ang_min_rad_{0.0f}, ang_max_rad_{0.0f};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ydlidar_x4_sdk_node");

  YDLidarX4SdkNode node;
  node.spin();
  return 0;
}
