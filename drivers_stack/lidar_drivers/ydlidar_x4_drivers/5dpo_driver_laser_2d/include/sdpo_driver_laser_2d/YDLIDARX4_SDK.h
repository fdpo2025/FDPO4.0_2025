#pragma once

#include "sdpo_driver_laser_2d/SdpoDriverLaser2D.h"
#include <atomic>
#include <thread>
#include <string>

// SDK
#include "CYdLidar.h"  // vem do YDLidar-SDK (garante include_directories)

namespace sdpo_driver_laser_2d {

class YDLIDARX4_SDK : public SdpoDriverLaser2D {
public:
  YDLIDARX4_SDK();
  virtual ~YDLIDARX4_SDK();

  // API principal do teu driver
  void openSerial();     // sem override para evitar mismatch
  void closeSerial();    // idem
  void start();
  void stop();
  void restart();

  // A interface antiga exige estes métodos; aqui não são usados
  void processSerialData(unsigned char& ch);
  void processLaserData();
  void printPkgDataInfo() const;
  void printLaserData() const;

private:
  void loop();

private:
  CYdLidar lidar_;                 // SDK object (global namespace)
  std::atomic<bool> running_{false};
  std::thread worker_;

  // guardas locais para config
  std::string port_;
  int baud_{128000};
  float scan_freq_{5.0f};
};

} // namespace sdpo_driver_laser_2d
