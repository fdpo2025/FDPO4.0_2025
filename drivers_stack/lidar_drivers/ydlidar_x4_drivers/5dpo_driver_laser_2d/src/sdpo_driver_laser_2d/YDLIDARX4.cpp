#include "sdpo_driver_laser_2d/YDLIDARX4.h"

#include "sdpo_driver_laser_2d/utils.h"
#include <unistd.h>   // usleep
#include <cmath>

namespace sdpo_driver_laser_2d {

YDLIDARX4::YDLIDARX4() {
  dist_data.resize(kLaserScanMaxNumSamplesYDLIDARX4);
  ang_data.resize(kLaserScanMaxNumSamplesYDLIDARX4);
}

YDLIDARX4::~YDLIDARX4() {
  if (isSerialOpen()) {
    stop();
  }
  closeSerial();
}

void YDLIDARX4::start() {
  // Stop (safe)
  char stop_cmd[2] = {(char)0xA5, (char)0x65};
  serial_async_->write(stop_cmd, 2);
  usleep(50 * 1000);

  // Start scan
  char start_cmd[2] = {(char)0xA5, (char)0x60};
  serial_async_->write(start_cmd, 2);
  usleep(50 * 1000);
}

void YDLIDARX4::stop() {
  char stop_scan_cmd[2] = {(char)0xA5, (char)0x65};
  serial_async_->write(stop_scan_cmd, 2);
}

void YDLIDARX4::restart() {
  char rst_cmd[2] = {(char)0xA5, (char)0x80};
  serial_async_->write(rst_cmd, 2);
  usleep(200 * 1000);

  char start_cmd[2] = {(char)0xA5, (char)0x60};
  serial_async_->write(start_cmd, 2);
  usleep(50 * 1000);
}

void YDLIDARX4::processSerialData(unsigned char& ch) {
  // Máquina de estados (robusta)
  switch (state_) {
    case YDLIDARX4State::kIddle:
      // procura início de pacote
      if (ch == 0xAA) {
        state_ = YDLIDARX4State::kPH1;
        byte_count_ = 0;
      }
      sample_count_ = 0;
      break;

    case YDLIDARX4State::kPH1:
      // já vimos 0xAA, agora esperamos 0x55
      // se vier outro 0xAA, mantém-se em kPH1 (resync suave)
      if (ch == 0x55) {
        state_ = YDLIDARX4State::kPH2;
        byte_count_++;
      } else if (ch == 0xAA) {
        // continua à procura do 0x55 depois do novo 0xAA
        byte_count_ = 0;
        state_ = YDLIDARX4State::kPH1;
      } else {
        state_ = YDLIDARX4State::kIddle;
      }
      break;

    case YDLIDARX4State::kPH2:
      byte_count_++;
      state_ = YDLIDARX4State::kCT;
      break;

    case YDLIDARX4State::kCT:
      byte_count_++;
      // bit 0 indica "start of scan" (pacote zero)
      if ((ch & 0x01) == 0x01) {
        pkg_zero_ = true;

        // Publica o scan anterior e limpa contador
        if (pubLaserData) {
          pubLaserData();
        }
        data_count = 0;
      } else {
        pkg_zero_ = false;
      }
      state_ = YDLIDARX4State::kLSN;
      break;

    case YDLIDARX4State::kLSN:
      byte_count_++;
      pkg_num_samples_ = (ch & 0x00FF);
      state_ = YDLIDARX4State::kFSA1;
      break;

    case YDLIDARX4State::kFSA1:
      byte_count_++;
      raw_start_ang_ = (ch & 0x00FF);
      state_ = YDLIDARX4State::kFSA2;
      break;

    case YDLIDARX4State::kFSA2:
      byte_count_++;
      raw_start_ang_ = ((raw_start_ang_ | (ch << 8)) >> 1);
      start_ang_ = rawStartEndAng2Double(raw_start_ang_);
      state_ = YDLIDARX4State::kLSA1;
      break;

    case YDLIDARX4State::kLSA1:
      byte_count_++;
      raw_end_ang_ = (ch & 0x00FF);
      state_ = YDLIDARX4State::kLSA2;
      break;

    case YDLIDARX4State::kLSA2:
      byte_count_++;
      raw_end_ang_ = ((raw_end_ang_ | (ch << 8)) >> 1);
      end_ang_ = rawStartEndAng2Double(raw_end_ang_);
      state_ = YDLIDARX4State::kCS1;
      break;

    case YDLIDARX4State::kCS1:
      byte_count_++;
      pkg_check_code_ = (ch & 0x00FF);
      state_ = YDLIDARX4State::kCS2;
      break;

    case YDLIDARX4State::kCS2:
      byte_count_++;
      pkg_check_code_ = (pkg_check_code_ | (ch << 8));
      sample_count_ = 0;
      state_ = YDLIDARX4State::kSi1;
      break;

    case YDLIDARX4State::kSi1:
      // 1º byte (LSB) da distância
      byte_count_++;
      if (sample_count_ < kLaserScanMaxNumSamplesYDLIDARX4) {
        raw_dist_data_[sample_count_] = (ch & 0x00FF);
        state_ = YDLIDARX4State::kSi2;
      } else {
        // overflow de segurança: resync
        state_ = YDLIDARX4State::kIddle;
      }
      break;

    case YDLIDARX4State::kSi2:
      // 2º byte (MSB) da distância + fecha amostra
      byte_count_++;
      if (sample_count_ < kLaserScanMaxNumSamplesYDLIDARX4) {
        raw_dist_data_[sample_count_] = (raw_dist_data_[sample_count_] | (ch << 8));
        sample_count_++;

        // ✅ pacote completo: processa DEPOIS de fechar a última amostra
        if (sample_count_ >= pkg_num_samples_) {
          processLaserData();
          byte_count_ = 0;
          state_ = YDLIDARX4State::kIddle; // volta a procurar 0xAA
        } else {
          state_ = YDLIDARX4State::kSi1;
        }
      } else {
        state_ = YDLIDARX4State::kIddle;
      }
      break;
  }
}

void YDLIDARX4::processLaserData() {
  // ✅ Não processar pacote zero (ele só marca início da volta)
  // ✅ Evita divisão por zero em (sample_count_ - 1)
  if (pkg_zero_ || sample_count_ < 2) {
    return;
  }

  float delta_ang = end_ang_ - start_ang_;
  if (start_ang_ > end_ang_) {
    delta_ang += 360.0f;
  }

  for (size_t i = 0; i < sample_count_; i++) {
    // ✅ evita overflow / corrupção de memória
    if (data_count >= dist_data.size()) {
      break;
    }

    // distance em mm (para correção de ângulo)
    dist_data[data_count] = rawDist2Double(raw_dist_data_[i]);

    float angle_correction = 0.0f;
    if (raw_dist_data_[i] != 0) {
      angle_correction =
          atanf(21.8f * (155.3f - dist_data[data_count]) /
                (155.3f * dist_data[data_count])) * 360.0f / M_PIf32;
    }

    ang_data[data_count] = normAngRad(
        -(start_ang_ +
          delta_ang * static_cast<float>(i) /
              (static_cast<float>(sample_count_) - 1.0f) +
          angle_correction) * M_PIf32 / 180.0f);

    // distance em metros
    dist_data[data_count] /= 1000.0f;

    bool is_sample_ok = true;

    if (dist_range_check_) {
      if ((dist_data[data_count] < dist_min_) ||
          (dist_data[data_count] > dist_max_)) {
        is_sample_ok = false;
      }
    }

    if (ang_range_check_) {
      if ((ang_data[data_count] < ang_min_) ||
          (ang_data[data_count] > ang_max_)) {
        is_sample_ok = false;
      }
    }

    if (is_sample_ok) {
      data_count++;
    }
  }
}

void YDLIDARX4::printPkgDataInfo() const {
  std::cout << std::endl
            << "Start pkg: " << pkg_zero_
            << " #samples: " << pkg_num_samples_
            << " Raw start angle: " << raw_start_ang_
            << " Raw end angle: " << raw_end_ang_
            << " Checksum: " << pkg_check_code_ << std::endl;
  std::cout << "  Start angle: " << start_ang_
            << "  End angle: " << end_ang_ << std::endl << std::endl;
}

void YDLIDARX4::printLaserData() const {
  if (data_count > 0) {
    std::cout << std::endl
              << "Laser scan (# points: " << data_count << "):" << std::endl
              << "  Dist (m): ";
    for (size_t i = 0; i < data_count; i++) {
      std::cout << dist_data[i] << " ";
    }
    std::cout << std::endl << "  Angle (deg): ";
    for (size_t i = 0; i < data_count; i++) {
      std::cout << (ang_data[i] * 180.0f / M_PIf32) << " ";
    }
    std::cout << std::endl << std::endl;
  }
}

} // namespace sdpo_driver_laser_2d
