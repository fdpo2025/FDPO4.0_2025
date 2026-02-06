#include "sdpo_driver_laser_2d/YDLIDARX4_SDK.h"
#include <stdexcept>

namespace sdpo_driver_laser_2d {

YDLIDARX4_SDK::YDLIDARX4_SDK() {
  dist_data.resize(kLaserScanMaxNumSamplesYDLIDARX4);
  ang_data.resize(kLaserScanMaxNumSamplesYDLIDARX4);
}

YDLIDARX4_SDK::~YDLIDARX4_SDK() {
  stop();
  closeSerial();
}

void YDLIDARX4_SDK::openSerial() {
  lidar_.setlidaropt(ydlidar::LidarPropSerialPort,
                     serial_port_name_.c_str(),
                     serial_port_name_.size());

  int baud = baud_rate_;
  lidar_.setlidaropt(ydlidar::LidarPropSerialBaudrate, &baud, sizeof(int));

  int lidar_type = ydlidar::TYPE_TRIANGLE;
  lidar_.setlidaropt(ydlidar::LidarPropLidarType, &lidar_type, sizeof(int));

  float scan_freq = 5.0f;
  lidar_.setlidaropt(ydlidar::LidarPropScanFrequency, &scan_freq, sizeof(float));

  if (!lidar_.initialize()) {
    throw std::runtime_error("YDLidar-SDK: initialize() failed");
  }
}

void YDLIDARX4_SDK::closeSerial() {
  lidar_.turnOff();
  lidar_.disconnecting();
}

void YDLIDARX4_SDK::start() {
  if (running_) return;

  if (!lidar_.turnOn()) {
    throw std::runtime_error("YDLidar-SDK: turnOn() failed");
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

void YDLIDARX4_SDK::loop() {
  ydlidar::LaserScan scan;   // <-- TEM de existir aqui

  while (running_) {
    if (!lidar_.doProcessSimple(scan)) {  // <-- lidar_ é membro da classe
      continue;
    }

    data_count = 0;

    for (const auto &p : scan.points) {
      if (data_count >= dist_data.size()) break;
      dist_data[data_count] = p.range;
      ang_data[data_count]  = p.angle;
      data_count++;
    }

    if (pubLaserData) pubLaserData();
  }
}

} // namespace sdpo_driver_laser_2d
