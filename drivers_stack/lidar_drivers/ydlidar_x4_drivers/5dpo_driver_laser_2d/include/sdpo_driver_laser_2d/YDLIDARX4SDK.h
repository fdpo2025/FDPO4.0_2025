#pragma once

#include <atomic>
#include <memory>
#include <thread>

#include "sdpo_driver_laser_2d/SdpoDriverLaser2D.h"

class CYdLidar;
struct LaserScan;

namespace sdpo_driver_laser_2d {

class YDLIDARX4SDK : public SdpoDriverLaser2D {
 public:
  YDLIDARX4SDK();
  ~YDLIDARX4SDK() override;

  bool connect(const bool dbg = false) override;
  void disconnect(const bool dbg = false) override;

  void start() override;
  void stop() override;
  void restart() override;

 protected:
  void processSerialData(unsigned char& ch) override;
  void processLaserData() override;

  void printPkgDataInfo() const override;
  void printLaserData() const override;

 private:
  void scanLoop();
  void copyScan(const LaserScan& scan);

  std::unique_ptr<CYdLidar> sdk_;
  std::thread scan_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
};

}  // namespace sdpo_driver_laser_2d
