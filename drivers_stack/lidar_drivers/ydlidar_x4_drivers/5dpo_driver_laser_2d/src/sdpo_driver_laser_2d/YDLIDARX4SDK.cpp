#include "sdpo_driver_laser_2d/YDLIDARX4SDK.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

#include "CYdLidar.h"
#include "sdpo_driver_laser_2d/utils.h"

namespace sdpo_driver_laser_2d {

namespace {
constexpr int kSampleRate = 5;
constexpr int kAbnormalCheckCount = 4;
constexpr float kDefaultMinRange = 0.05f;
constexpr float kDefaultMaxRange = 16.0f;
constexpr float kDefaultMinAngleDeg = -180.0f;
constexpr float kDefaultMaxAngleDeg = 180.0f;
constexpr float kDefaultScanFrequencyHz = 10.0f;
}  // namespace

YDLIDARX4SDK::YDLIDARX4SDK() : sdk_(new CYdLidar()) {}

YDLIDARX4SDK::~YDLIDARX4SDK() {
  stop();
  disconnect();
}

bool YDLIDARX4SDK::connect(const bool dbg) {
  ydlidar::os_init();

  std::string ignore_array;
  sdk_->setlidaropt(LidarPropSerialPort, serial_port_name_.c_str(),
                    static_cast<int>(serial_port_name_.size()));
  sdk_->setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                    static_cast<int>(ignore_array.size()));

  int baudrate = baud_rate_;
  sdk_->setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));

  int lidar_type = TYPE_TRIANGLE;
  sdk_->setlidaropt(LidarPropLidarType, &lidar_type, sizeof(int));

  int device_type = YDLIDAR_TYPE_SERIAL;
  sdk_->setlidaropt(LidarPropDeviceType, &device_type, sizeof(int));

  int sample_rate = kSampleRate;
  sdk_->setlidaropt(LidarPropSampleRate, &sample_rate, sizeof(int));

  int abnormal_count = kAbnormalCheckCount;
  sdk_->setlidaropt(LidarPropAbnormalCheckCount, &abnormal_count, sizeof(int));

  bool b = false;
  sdk_->setlidaropt(LidarPropFixedResolution, &b, sizeof(bool));
  sdk_->setlidaropt(LidarPropReversion, &b, sizeof(bool));
  sdk_->setlidaropt(LidarPropInverted, &b, sizeof(bool));
  sdk_->setlidaropt(LidarPropSingleChannel, &b, sizeof(bool));
  sdk_->setlidaropt(LidarPropIntenstiy, &b, sizeof(bool));
  sdk_->setlidaropt(LidarPropSupportMotorDtrCtrl, &b, sizeof(bool));
  sdk_->setlidaropt(LidarPropSupportHeartBeat, &b, sizeof(bool));

  b = true;
  sdk_->setlidaropt(LidarPropAutoReconnect, &b, sizeof(bool));

  float min_angle = ang_range_check_ ? (ang_min_ * 180.0f / M_PIf32) : kDefaultMinAngleDeg;
  float max_angle = ang_range_check_ ? (ang_max_ * 180.0f / M_PIf32) : kDefaultMaxAngleDeg;
  float min_range = dist_range_check_ ? dist_min_ : kDefaultMinRange;
  float max_range = dist_range_check_ ? dist_max_ : kDefaultMaxRange;
  float scan_frequency = kDefaultScanFrequencyHz;
  sdk_->setlidaropt(LidarPropMinAngle, &min_angle, sizeof(float));
  sdk_->setlidaropt(LidarPropMaxAngle, &max_angle, sizeof(float));
  sdk_->setlidaropt(LidarPropMinRange, &min_range, sizeof(float));
  sdk_->setlidaropt(LidarPropMaxRange, &max_range, sizeof(float));
  sdk_->setlidaropt(LidarPropScanFrequency, &scan_frequency, sizeof(float));

  sdk_->enableGlassNoise(false);
  sdk_->enableSunNoise(false);
  sdk_->setBottomPriority(true);

  if (!sdk_->initialize()) {
    if (dbg) {
      std::cerr << "[YDLIDARX4SDK] initialize failed: "
                << sdk_->DescribeError() << std::endl;
    }
    return false;
  }

  connected_ = true;
  return true;
}

void YDLIDARX4SDK::disconnect(const bool dbg) {
  (void)dbg;
  if (!connected_) return;
  sdk_->disconnecting();
  connected_ = false;
}

void YDLIDARX4SDK::start() {
  if (!connected_) {
    throw std::runtime_error("YDLIDARX4SDK start called before connect");
  }
  if (running_) return;
  if (!sdk_->turnOn()) {
    throw std::runtime_error(std::string("YDLIDARX4SDK turnOn failed: ") +
                             sdk_->DescribeError());
  }
  running_ = true;
  scan_thread_ = std::thread(&YDLIDARX4SDK::scanLoop, this);
}

void YDLIDARX4SDK::stop() {
  if (!running_) return;
  running_ = false;
  if (scan_thread_.joinable()) scan_thread_.join();
  sdk_->turnOff();
}

void YDLIDARX4SDK::restart() {
  stop();
  start();
}

void YDLIDARX4SDK::processSerialData(unsigned char& ch) {
  (void)ch;
}

void YDLIDARX4SDK::processLaserData() {}

void YDLIDARX4SDK::printPkgDataInfo() const {}

void YDLIDARX4SDK::printLaserData() const {}

void YDLIDARX4SDK::scanLoop() {
  while (running_ && ydlidar::os_isOk()) {
    LaserScan scan;
    if (sdk_->doProcessSimple(scan)) {
      copyScan(scan);
      if (pubLaserData) pubLaserData();
      continue;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void YDLIDARX4SDK::copyScan(const LaserScan& scan) {
  dist_data.clear();
  ang_data.clear();
  dist_data.reserve(scan.points.size());
  ang_data.reserve(scan.points.size());

  scan_freq_hz_ = scan.scanFreq > 0.0f ? scan.scanFreq : kDefaultScanFrequencyHz;

  for (const auto& p : scan.points) {
    float range = p.range;
    float angle = normAngRad(-p.angle);

    bool sample_ok = true;
    if (dist_range_check_ && (range < dist_min_ || range > dist_max_)) {
      sample_ok = false;
    }
    if (ang_range_check_ && (angle < ang_min_ || angle > ang_max_)) {
      sample_ok = false;
    }
    if (!sample_ok) continue;

    dist_data.push_back(range);
    ang_data.push_back(angle);
  }

  data_count = dist_data.size();
}

}  // namespace sdpo_driver_laser_2d
