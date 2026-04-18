#pragma once

#include "sdpo_driver_laser_2d/SdpoDriverLaser2D.h"
#include <atomic>
#include <thread>
#include <string>

#include "CYdLidar.h"

namespace sdpo_driver_laser_2d {

class YDLIDARX4_SDK : public SdpoDriverLaser2D {
public:
  YDLIDARX4_SDK();
  virtual ~YDLIDARX4_SDK();

  void openSerial();
  void closeSerial();
  void start();
  void stop();
  void restart();

  void processSerialData(unsigned char& ch);
  void processLaserData();
  void printPkgDataInfo() const;
  void printLaserData() const;

private:
  void loop();

private:
  CYdLidar lidar_;
  std::atomic<bool> running_{false};
  std::thread worker_;

  std::string port_;
  int baud_{128000};
  float scan_freq_{5.0f};
};

} // namespace sdpo_driver_laser_2d
