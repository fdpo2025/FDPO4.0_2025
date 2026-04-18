#include "sdpo_driver_laser_2d/YDLIDARX4_SDK.h"
#include "sdpo_driver_laser_2d/utils.h"
#include "sdpo_driver_laser_2d/YDLIDARX4.h"

#include <cmath>
#include <stdexcept>
#include <chrono>
#include <thread>

namespace sdpo_driver_laser_2d {

YDLIDARX4_SDK::YDLIDARX4_SDK() {
  dist_data.resize(kLaserScanMaxNumSamplesYDLIDARX4);
  ang_data.resize(kLaserScanMaxNumSamplesYDLIDARX4);
}

YDLIDARX4_SDK::~YDLIDARX4_SDK() {
  try {
    stop();
  } catch (...) {}
  try {
    closeSerial();
  } catch (...) {}
}

void YDLIDARX4_SDK::openSerial() {
  port_ = serial_port_name_;
  baud_ = baud_rate_;

  lidar_.setlidaropt(LidarPropSerialPort, port_.c_str(), static_cast<int>(port_.size()));
  lidar_.setlidaropt(LidarPropSerialBaudrate, &baud_, sizeof(int));

  int lidar_type = TYPE_TRIANGLE;
  lidar_.setlidaropt(LidarPropLidarType, &lidar_type, sizeof(int));

  lidar_.setlidaropt(LidarPropScanFrequency, &scan_freq_, sizeof(float));

  if (!lidar_.initialize()) {
    throw std::runtime_error("[YDLIDARX4_SDK] lidar_.initialize() falhou (porta/baud/model/config).");
  }
}

void YDLIDARX4_SDK::closeSerial() {
  stop();
  lidar_.turnOff();
  lidar_.disconnecting();
}

void YDLIDARX4_SDK::start() {
  if (running_) return;

  if (!lidar_.turnOn()) {
    throw std::runtime_error("[YDLIDARX4_SDK] lidar_.turnOn() falhou.");
  }

  running_ = true;
  worker_ = std::thread(&YDLIDARX4_SDK::loop, this);
}

void YDLIDARX4_SDK::stop() {
  if (!running_) return;

  running_ = false;
  if (worker_.joinable()) worker_.join();

  lidar_.turnOff();
}

void YDLIDARX4_SDK::restart() {
  stop();
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  start();
}

void YDLIDARX4_SDK::loop() {
  LaserScan scan;

  while (running_) {
    if (lidar_.doProcessSimple(scan)) {
      const size_t n = std::min(scan.points.size(), dist_data.size());

      data_count = 0;
      for (size_t i = 0; i < n; ++i) {
        const auto &p = scan.points[i];

        const float dist_m = static_cast<float>(p.range);
        const float ang_rad = static_cast<float>(p.angle) * M_PIf32 / 180.0f;

        dist_data[data_count] = dist_m;
        ang_data[data_count]  = normAngRad(-ang_rad);

        bool ok = true;
        if (dist_range_check_) {
          if (dist_m < dist_min_ || dist_m > dist_max_) ok = false;
        }
        if (ang_range_check_) {
          if (ang_data[data_count] < ang_min_ || ang_data[data_count] > ang_max_) ok = false;
        }

        if (ok) data_count++;
      }

      if (pubLaserData) pubLaserData();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void YDLIDARX4_SDK::processSerialData(unsigned char& /*ch*/) {
}

void YDLIDARX4_SDK::processLaserData() {
}

void YDLIDARX4_SDK::printPkgDataInfo() const {
}

void YDLIDARX4_SDK::printLaserData() const {
}

} // namespace sdpo_driver_laser_2d
