#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <lidar_calibration/GetStatistics.h>
#include <yaml-cpp/yaml.h>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <map>
#include <string>

class StatisticsCalculator {
public:
    StatisticsCalculator() : nh_() {
        // Obter caminho do ficheiro YAML (usar ~/.ros/lidar_calibration/data)
        const char* home = std::getenv("HOME");
        if (home) {
            yaml_file_ = std::string(home) + "/.ros/lidar_calibration/data/measurements.yaml";
        } else {
            yaml_file_ = "/tmp/lidar_calibration/data/measurements.yaml";
        }
        
        // Serviço para calcular estatísticas
        get_stats_service_ = nh_.advertiseService("calculate_statistics", 
                                                   &StatisticsCalculator::calculateStatistics, this);
        
        ROS_INFO("[StatisticsCalculator] Node initialized.");
        ROS_INFO("[StatisticsCalculator] YAML file: %s", yaml_file_.c_str());
        ROS_INFO("[StatisticsCalculator] Service: /calculate_statistics");
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceServer get_stats_service_;
    std::string yaml_file_;
    
    double normalizeAngle(double theta) {
        while(theta > M_PI) theta -= 2.0 * M_PI;
        while(theta <= -M_PI) theta += 2.0 * M_PI;
        return theta;
    }
    
    bool calculateStatistics(lidar_calibration::GetStatistics::Request& req,
                             lidar_calibration::GetStatistics::Response& res) {
        try {
            // Ler ficheiro YAML
            YAML::Node root = YAML::LoadFile(yaml_file_);
            
            if (!root["iterations"]) {
                res.success = false;
                res.message = "No iterations found in YAML file";
                return true;
            }
            
            // Estrutura: [beacon_name] -> estatísticas locais por iteração
            struct LocalStats {
                double var_r;
                double var_theta;
                size_t N;
            };
            std::map<std::string, std::vector<LocalStats>> stats_by_beacon;
            
            // PASSO 1: Calcular estatísticas POR ITERAÇÃO (robô parado)
            for (const auto& iteration : root["iterations"]) {
                if (!iteration["beacons"]) continue;
                
                for (const auto& beacon_node : iteration["beacons"]) {
                    std::string beacon_name = beacon_node.first.as<std::string>();
                    
                    if (!beacon_node.second["r"] || !beacon_node.second["theta"]) continue;
                    
                    std::vector<double> r_vals = beacon_node.second["r"].as<std::vector<double>>();
                    std::vector<double> theta_vals = beacon_node.second["theta"].as<std::vector<double>>();
                    
                    if (r_vals.empty() || r_vals.size() != theta_vals.size() || r_vals.size() < 2) continue;
                    
                    // Calcular MÉDIA LOCAL (desta iteração)
                    double mean_r_j = 0.0;
                    for (double r : r_vals) {
                        mean_r_j += r;
                    }
                    mean_r_j /= r_vals.size();
                    
                    // Média circular de theta
                    double sum_cos = 0.0, sum_sin = 0.0;
                    for (double theta : theta_vals) {
                        sum_cos += std::cos(theta);
                        sum_sin += std::sin(theta);
                    }
                    double mean_cos = sum_cos / theta_vals.size();
                    double mean_sin = sum_sin / theta_vals.size();
                    double mean_theta_j = std::atan2(mean_sin, mean_cos);
                    
                    // Calcular RESÍDUOS LOCAIS e VARIÂNCIA LOCAL
                    double var_r_j = 0.0, var_theta_j = 0.0;
                    for (size_t i = 0; i < r_vals.size(); ++i) {
                        double res_r = r_vals[i] - mean_r_j;
                        var_r_j += res_r * res_r;
                        
                        double res_theta = normalizeAngle(theta_vals[i] - mean_theta_j);
                        var_theta_j += res_theta * res_theta;
                    }
                    
                    // Correção de Bessel: dividir por (N-1) para estimativa não enviesada
                    size_t N_j = r_vals.size();
                    if (N_j > 1) {
                        var_r_j /= (N_j - 1);
                        var_theta_j /= (N_j - 1);
                    } else {
                        var_r_j = 0.0;
                        var_theta_j = 0.0;
                    }
                    
                    // Guardar estatísticas locais desta iteração
                    LocalStats local;
                    local.var_r = var_r_j;
                    local.var_theta = var_theta_j;
                    local.N = N_j;
                    stats_by_beacon[beacon_name].push_back(local);
                }
            }
            
            if (stats_by_beacon.empty()) {
                res.success = false;
                res.message = "No measurements found in YAML file";
                return true;
            }
            
            res.success = true;
            res.message = "Statistics computed successfully (per-iteration method)";
            
            // PASSO 2: Combinar variâncias das iterações (média ponderada)
            // σ² = Σ[(N_j - 1) * σ²_j] / Σ(N_j - 1)
            for (const auto& beacon_pair : stats_by_beacon) {
                const std::string& beacon_name = beacon_pair.first;
                const std::vector<LocalStats>& local_stats = beacon_pair.second;
                
                if (local_stats.empty()) continue;
                
                // Calcular variância combinada (média ponderada)
                double sum_weighted_var_r = 0.0;
                double sum_weighted_var_theta = 0.0;
                double sum_weights = 0.0;
                size_t total_samples = 0;
                
                for (const auto& local : local_stats) {
                    if (local.N > 1) {
                        double weight = local.N - 1;  // Peso = (N_j - 1)
                        sum_weighted_var_r += weight * local.var_r;
                        sum_weighted_var_theta += weight * local.var_theta;
                        sum_weights += weight;
                    }
                    total_samples += local.N;
                }
                
                double var_r = 0.0, var_theta = 0.0;
                if (sum_weights > 0) {
                    var_r = sum_weighted_var_r / sum_weights;
                    var_theta = sum_weighted_var_theta / sum_weights;
                }
                
                // Guardar resultados
                res.beacon_names.push_back(beacon_name);
                res.mean_r.push_back(0.0);  // Média global não é relevante para Cov(v)
                res.mean_theta.push_back(0.0);  // Média global não é relevante para Cov(v)
                res.var_r.push_back(var_r);
                res.var_theta.push_back(var_theta);
                res.num_samples.push_back((int)total_samples);
                
                ROS_INFO("[StatisticsCalculator] Beacon '%s':", beacon_name.c_str());
                ROS_INFO("  Variance r (sensor noise): %.9f m²", var_r);
                ROS_INFO("  Variance theta (sensor noise): %.9f rad² (%.6f deg²)", 
                         var_theta, var_theta * (180.0 / M_PI) * (180.0 / M_PI));
                ROS_INFO("  Std dev r: %.6f m", std::sqrt(var_r));
                ROS_INFO("  Std dev theta: %.6f rad (%.2f deg)", 
                         std::sqrt(var_theta), std::sqrt(var_theta) * 180.0 / M_PI);
                ROS_INFO("  Total samples: %zu (from %zu iterations)", total_samples, local_stats.size());
            }
            
            return true;
            
        } catch (const std::exception& e) {
            res.success = false;
            res.message = std::string("Error reading YAML: ") + e.what();
            ROS_ERROR("[StatisticsCalculator] %s", res.message.c_str());
            return true;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "statistics_calculator_node");
    
    StatisticsCalculator calculator;
    
    ROS_INFO("[StatisticsCalculator] Node running. Waiting for service calls...");
    ros::spin();
    
    return 0;
}

