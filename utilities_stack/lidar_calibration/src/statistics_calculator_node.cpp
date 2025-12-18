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
            
            // Estrutura: [beacon_name] -> todas as medições de todas as iterações
            std::map<std::string, std::vector<double>> all_r_by_beacon;
            std::map<std::string, std::vector<double>> all_theta_by_beacon;
            
            // Agregar todas as medições de todas as iterações
            for (const auto& iteration : root["iterations"]) {
                if (!iteration["beacons"]) continue;
                
                for (const auto& beacon_node : iteration["beacons"]) {
                    std::string beacon_name = beacon_node.first.as<std::string>();
                    
                    if (beacon_node.second["r"] && beacon_node.second["theta"]) {
                        std::vector<double> r_vals = beacon_node.second["r"].as<std::vector<double>>();
                        std::vector<double> theta_vals = beacon_node.second["theta"].as<std::vector<double>>();
                        
                        // Adicionar às listas agregadas
                        for (double r : r_vals) {
                            all_r_by_beacon[beacon_name].push_back(r);
                        }
                        for (double theta : theta_vals) {
                            all_theta_by_beacon[beacon_name].push_back(theta);
                        }
                    }
                }
            }
            
            if (all_r_by_beacon.empty()) {
                res.success = false;
                res.message = "No measurements found in YAML file";
                return true;
            }
            
            res.success = true;
            res.message = "Statistics computed successfully";
            
            // Para cada beacon, calcular estatísticas
            for (const auto& beacon_pair : all_r_by_beacon) {
                const std::string& beacon_name = beacon_pair.first;
                const std::vector<double>& all_r = beacon_pair.second;
                const std::vector<double>& all_theta = all_theta_by_beacon[beacon_name];
                
                if (all_r.empty() || all_r.size() != all_theta.size()) continue;
                
                // 1. Calcular MÉDIA de r e theta
                double mean_r = 0.0, mean_theta = 0.0;
                for (size_t i = 0; i < all_r.size(); ++i) {
                    mean_r += all_r[i];
                    mean_theta += all_theta[i];
                }
                mean_r /= all_r.size();
                mean_theta /= all_theta.size();
                mean_theta = normalizeAngle(mean_theta);
                
                // 2. Calcular RESÍDUOS (cada medição - média)
                std::vector<double> residuals_r, residuals_theta;
                for (size_t i = 0; i < all_r.size(); ++i) {
                    residuals_r.push_back(all_r[i] - mean_r);
                    residuals_theta.push_back(normalizeAngle(all_theta[i] - mean_theta));
                }
                
                // 3. Calcular VARIÂNCIA a partir do somatório dos resíduos
                // Var = (1/N) * Σ(residual²)
                double var_r = 0.0, var_theta = 0.0;
                for (size_t i = 0; i < residuals_r.size(); ++i) {
                    var_r += residuals_r[i] * residuals_r[i];
                    var_theta += residuals_theta[i] * residuals_theta[i];
                }
                var_r /= residuals_r.size();
                var_theta /= residuals_theta.size();
                
                // Guardar resultados
                res.beacon_names.push_back(beacon_name);
                res.mean_r.push_back(mean_r);
                res.mean_theta.push_back(mean_theta);
                res.var_r.push_back(var_r);
                res.var_theta.push_back(var_theta);
                res.num_samples.push_back((int)all_r.size());
                
                ROS_INFO("[StatisticsCalculator] Beacon '%s':", beacon_name.c_str());
                ROS_INFO("  Mean r: %.6f m", mean_r);
                ROS_INFO("  Mean theta: %.6f rad (%.2f deg)", mean_theta, mean_theta * 180.0 / M_PI);
                ROS_INFO("  Variance r: %.9f m²", var_r);
                ROS_INFO("  Variance theta: %.9f rad² (%.6f deg²)", 
                         var_theta, var_theta * (180.0 / M_PI) * (180.0 / M_PI));
                ROS_INFO("  Std dev r: %.6f m", std::sqrt(var_r));
                ROS_INFO("  Std dev theta: %.6f rad (%.2f deg)", 
                         std::sqrt(var_theta), std::sqrt(var_theta) * 180.0 / M_PI);
                ROS_INFO("  Samples: %zu", all_r.size());
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

