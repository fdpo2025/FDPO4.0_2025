#pragma once

#include "sdpo_driver_laser_2d/SdpoDriverLaser2D.h"
#include <atomic>
#include <thread>

#include "CYdLidar.h"

namespace sdpo_driver_laser_2d {

class YDLIDARX4_SDK : public SdpoDriverLaser2D {
public:
  YDLIDARX4_SDK();
  ~YDLIDARX4_SDK() override;

  void openSerial();
  void closeSerial();
  void start() override;
  void stop() override;

private:
  void loop();

  std::atomic<bool> running_{false};
  std::thread worker_;

  CYdLidar lidar_;   // <-- TEM de se chamar lidar_
    // Methods required by SdpoDriverLaser2D (not used in SDK mode)
  void restart() override;
  void processSerialData(unsigned char& ch) override;
  void processLaserData() override;
  void printPkgDataInfo() const override;
  void printLaserData() const override;

};

} // namespace sdpo_driver_laser_2d
