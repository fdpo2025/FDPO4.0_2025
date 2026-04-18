#pragma once

#include <memory>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

#include "sdpo_driver_laser_2d/LaserExtrinsicParamConfig.h"
#include "sdpo_driver_laser_2d/SdpoDriverLaser2D.h"

namespace sdpo_driver_laser_2d {

class SdpoDriverLaser2DROS {
 private:
  ros::NodeHandle nh;

  ros::Publisher pub_laser_;
  ros::Publisher pub_scan_freq_;
  tf::TransformBroadcaster tf_broad_;

  ros::Time sample_time_;

  dynamic_reconfigure::Server<sdpo_driver_laser_2d::LaserExtrinsicParamConfig>
      cfg_server_;

  std::unique_ptr<SdpoDriverLaser2D> laser_;

  std::string model_;
  std::string serial_port_name_;
  int baud_rate_;
  std::string base_frame_id_;
  std::string laser_frame_id_;
  float laser_pose_x_;
  float laser_pose_y_;
  float laser_pose_z_;
  float laser_pose_yaw_;
  float laser_pose_pitch_;
  float laser_pose_roll_;
  float dist_min_;
  float dist_max_;
  float ang_min_;
  float ang_max_;
  /** Fallback scan frequency (Hz) when device does not report it (e.g. CT[7:1]=0). Used for laser_scan_frequency topic. */
  float default_scan_freq_hz_ = 10.0f;

 public:
  SdpoDriverLaser2DROS();
  ~SdpoDriverLaser2DROS() = default;

  void start();
  void stop();

 private:
  void readParam();
  void cfgServerCallback(sdpo_driver_laser_2d::LaserExtrinsicParamConfig &cfg,
      uint32_t level);
  void pubLaserData();
};

} // namespace sdpo_driver_laser_2d
