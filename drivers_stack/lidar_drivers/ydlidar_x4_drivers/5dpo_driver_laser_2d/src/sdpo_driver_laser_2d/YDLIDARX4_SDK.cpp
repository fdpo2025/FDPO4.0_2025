#include "sdpo_driver_laser_2d/YDLIDARX4_SDK.h"
#include "sdpo_driver_laser_2d/utils.h"
#include "sdpo_driver_laser_2d/YDLIDARX4.h"

#include <cmath>
#include <stdexcept>
#include <chrono>
#include <thread>

namespace sdpo_driver_laser_2d {

YDLIDARX4_SDK::YDLIDARX4_SDK() {
  // O SdpoDriverLaser2D (base) normalmente já tem dist_data/ang_data
  // Aqui garantimos tamanho “grande” para não cortar pontos.
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

bool YDLIDARX4_SDK::connect(const bool dbg) {
  (void)dbg;
  openSerial();
  return true;
}

void YDLIDARX4_SDK::disconnect(const bool dbg) {
  (void)dbg;
  closeSerial();
}

void YDLIDARX4_SDK::openSerial() {
  // O SdpoDriverLaser2DROS chama: setSerialPortParam(port, baud)
  // Por isso aqui vamos buscar o que foi guardado pela base.
  port_ = serial_port_name_;
  baud_ = baud_rate_;

  // Config mínima para X4 (triangular protocol)
  // IMPORTANTÍSSIMO: enums/propriedades são globais no teu SDK
  lidar_.setlidaropt(LidarPropSerialPort, port_.c_str(), static_cast<int>(port_.size()));
  lidar_.setlidaropt(LidarPropSerialBaudrate, &baud_, sizeof(int));

  int lidar_type = TYPE_TRIANGLE;
  lidar_.setlidaropt(LidarPropLidarType, &lidar_type, sizeof(int));

  // Frequência (ajusta para o que tu queres; 5Hz é “seguro”)
  lidar_.setlidaropt(LidarPropScanFrequency, &scan_freq_, sizeof(float));

  // Expor para o ROS wrapper (publicado em /laser_scan_frequency e usado
  // pelo motion_distortion_compensator).
  scan_freq_hz_ = scan_freq_;

  // (Opcional) se o teu X4 for “intensity off”, não mexas.
  // (Opcional) podes ligar/desligar “fixed resolution” aqui se existir na tua versão.

  if (!lidar_.initialize()) {
    throw std::runtime_error("[YDLIDARX4_SDK] lidar_.initialize() falhou (porta/baud/model/config).");
  }
}

void YDLIDARX4_SDK::closeSerial() {
  // Garante parar thread antes de desligar
  stop();

  // turnOff + disconnecting (o SDK tem isto)
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
  // LaserScan no teu SDK é global (não é ydlidar::LaserScan)
  LaserScan scan;

  while (running_) {
    // doProcessSimple devolve um scan completo
    if (lidar_.doProcessSimple(scan)) {
      // Frequência REAL do scan, calculada pelo SDK a partir do tempo
      // total da volta. Cobre desvios em relação ao valor configurado
      // (ex.: o motor não consegue atingir o setpoint).
      // Fallback para o valor configurado se vier inválido.
      if (scan.config.scan_time > 1e-6f) {
        scan_freq_hz_ = 1.0f / scan.config.scan_time;
      } else {
        scan_freq_hz_ = scan_freq_;
      }

      // Copiar para buffers esperados pelo SdpoDriverLaser2DROS
      const size_t n = std::min(scan.points.size(), dist_data.size());

      data_count = 0;
      for (size_t i = 0; i < n; ++i) {
        const auto &p = scan.points[i];

        // No SDK do YDLidar (struct LaserPoint):
        // - p.range em METROS  (unit:m)
        // - p.angle em RADIANOS (unit:rad)
        const float dist_m = static_cast<float>(p.range);
        const float ang_rad = static_cast<float>(p.angle);

        // Mantém a convenção do teu driver antigo (ângulo normalizado)
        dist_data[data_count] = dist_m;
        ang_data[data_count]  = normAngRad(-ang_rad);

        // checks de range que já existem na base
        bool ok = true;
        if (dist_range_check_) {
          if (dist_m < dist_min_ || dist_m > dist_max_) ok = false;
        }
        if (ang_range_check_) {
          if (ang_data[data_count] < ang_min_ || ang_data[data_count] > ang_max_) ok = false;
        }

        if (ok) data_count++;
      }

      // Publica 1x por scan (equivalente ao “pkg_zero_” do protocolo antigo)
      if (pubLaserData) pubLaserData();
    }

    // Pequeno sleep para não ocupar 100% CPU quando falha leitura
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

/* -------------------------------------------------------------------------- */
/*  Métodos exigidos pela interface antiga — aqui não são usados com SDK       */
/* -------------------------------------------------------------------------- */

void YDLIDARX4_SDK::processSerialData(unsigned char& /*ch*/) {
  // Não usado: com SDK não fazemos parsing byte-a-byte.
}

void YDLIDARX4_SDK::processLaserData() {
  // Não usado: o SDK já entrega scan pronto.
}

void YDLIDARX4_SDK::printPkgDataInfo() const {
  // Não aplicável no modo SDK.
}

void YDLIDARX4_SDK::printLaserData() const {
  // Debug simples do que foi produzido
  // (podes deixar vazio se não precisares)
}

} // namespace sdpo_driver_laser_2d
