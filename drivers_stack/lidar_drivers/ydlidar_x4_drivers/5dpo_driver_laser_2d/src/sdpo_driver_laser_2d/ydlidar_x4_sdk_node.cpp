#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <cmath>
#include <string>

// SDK
#include "CYdLidar.h"
#include "core/common/ydlidar_def.h"
#include "core/common/ydlidar_datatype.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ydlidar_x4_sdk_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string port = "/dev/ttyUSB0";
  int baudrate = 128000;
  float frequency = 5.0f;
  std::string frame_id = "laser";

  pnh.param<std::string>("port", port, port);
  pnh.param<int>("baudrate", baudrate, baudrate);
  pnh.param<float>("frequency", frequency, frequency);
  pnh.param<std::string>("frame_id", frame_id, frame_id);

  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud>("laser_scan_point_cloud", 1);

  CYdLidar lidar;

  // ------- parâmetros base (copiados do tri_test.cpp) -------
  lidar.setlidaropt(LidarPropSerialPort, port.c_str(), (int)port.size());
  lidar.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));

  int optval = TYPE_TRIANGLE;                 // X4 usa protocolo triangular
  lidar.setlidaropt(LidarPropLidarType, &optval, sizeof(int));

  optval = YDLIDAR_TYPE_SERIAL;               // serial
  lidar.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));

  // valores típicos do tri_test (podes ajustar depois)
  optval = 5; // sample rate “code” (no tri_test é int)
  lidar.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));

  optval = 4; // abnormal check count (tri_test usa 4)
  lidar.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

  bool b = true;
  lidar.setlidaropt(LidarPropFixedResolution, &b, sizeof(bool));
  lidar.setlidaropt(LidarPropAutoReconnect, &b, sizeof(bool));

  bool isSingleChannel = true; // X4 é single channel
  lidar.setlidaropt(LidarPropSingleChannel, &isSingleChannel, sizeof(bool));

  float f;
  f = 180.0f;  lidar.setlidaropt(LidarPropMaxAngle, &f, sizeof(float));
  f = -180.0f; lidar.setlidaropt(LidarPropMinAngle, &f, sizeof(float));
  f = 16.0f;   lidar.setlidaropt(LidarPropMaxRange, &f, sizeof(float));
  f = 0.08f;   lidar.setlidaropt(LidarPropMinRange, &f, sizeof(float));

  lidar.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));
  // ----------------------------------------------------------

  if (!lidar.initialize()) {
    ROS_FATAL("YDLidar SDK initialize() falhou (porta=%s baud=%d).", port.c_str(), baudrate);
    return 1;
  }
  if (!lidar.turnOn()) {
    ROS_FATAL("YDLidar turnOn() falhou.");
    return 1;
  }

  ydlidar::LaserScan scan; // tipo do SDK (está no ydlidar_datatype.h)

  ros::Rate rate(20); // loop rápido; o scan vem à frequência do lidar
  while (ros::ok()) {
    if (lidar.doProcessSimple(scan)) {
      sensor_msgs::PointCloud msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = frame_id;

      msg.points.resize(scan.points.size());
      for (size_t i = 0; i < scan.points.size(); i++) {
        const auto &p = scan.points[i]; // p.angle em rad, p.range em m
        msg.points[i].x = p.range * std::cos(p.angle);
        msg.points[i].y = p.range * std::sin(p.angle);
        msg.points[i].z = 0.0f;
      }

      pub.publish(msg);
    }

    ros::spinOnce();
    rate.sleep();
  }

  lidar.turnOff();
  lidar.disconnecting();
  return 0;
}
