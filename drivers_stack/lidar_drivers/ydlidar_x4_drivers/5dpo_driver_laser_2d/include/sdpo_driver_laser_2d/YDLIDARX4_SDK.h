#pragma once

#include "sdpo_driver_laser_2d/SdpoDriverLaser2D.h"
#include <atomic>
#include <thread>

// YDLidar SDK
#include "CYdLidar.h"

namespace sdpo_driver_laser_2d {

class YDLIDARX4_SDK : public SdpoDriverLaser2D {
public:
  YDLIDARX4_SDK();
  ~YDLIDARX4_SDK() override;

  void openSerial() override;
  void closeSerial() override;

  void start() override;
  void stop() override;

private:
  void loop();

  std::atomic<bool> running_{false};
  std::thread worker_;

  ydlidar::CYdLidar lidar_;
};

} // namespace sdpo_driver_laser_2d
