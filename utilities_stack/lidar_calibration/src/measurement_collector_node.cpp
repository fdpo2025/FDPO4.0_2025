#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <localizer/BeaconMatch.h>
#include <localizer/Cluster.h>
#include <lidar_calibration/MeasurementCollection.h>
#include <cmath>
#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <sstream>
#include <iomanip>

class MeasurementCollector {
public:
    MeasurementCollector() : nh_(), collecting_(false), current_iteration_(0) {
        // Obter caminho para guardar ficheiros YAML
        std::string package_path = ros::package::getPath("lidar_calibration");
        yaml_dir_ = package_path + "/data";
        
        // Criar diretório se não existir
        std::string mkdir_cmd = "mkdir -p " + yaml_dir_;
        system(mkdir_cmd.c_str());
        
        yaml_file_ = yaml_dir_ + "/measurements.yaml";
        
        // Subscriber para medições de beacons
        beacon_sub_ = nh_.subscribe("/beacon_estimation", 10, 
                                    &MeasurementCollector::beaconCallback, this);
        
        // Serviços
        start_service_ = nh_.advertiseService("start_collection", 
                                              &MeasurementCollector::startCollection, this);
        stop_service_ = nh_.advertiseService("stop_collection", 
                                              &MeasurementCollector::stopCollection, this);
        
        ROS_INFO("[MeasurementCollector] Node initialized.");
        ROS_INFO("[MeasurementCollector] YAML file: %s", yaml_file_.c_str());
        ROS_INFO("[MeasurementCollector] Services:");
        ROS_INFO("  - /start_collection: Start collecting measurements (for current iteration)");
        ROS_INFO("  - /stop_collection: Stop collecting and save to YAML");
    }

private:
    struct Measurement {
        double r;      // distância
        double theta;  // ângulo
        ros::Time stamp;
    };

    struct SessionData {
        std::vector<Measurement> measurements;
    };

    ros::NodeHandle nh_;
    ros::Subscriber beacon_sub_;
    
    ros::ServiceServer start_service_;
    ros::ServiceServer stop_service_;
    
    bool collecting_;
    int current_iteration_;
    std::string yaml_dir_;
    std::string yaml_file_;
    
    // Dados da iteração atual: [beacon_name] -> vector of measurements
    std::map<std::string, std::vector<Measurement>> current_data_;
    
    double normalizeAngle(double theta) {
        while(theta > M_PI) theta -= 2.0 * M_PI;
        while(theta <= -M_PI) theta += 2.0 * M_PI;
        return theta;
    }
    
    void beaconCallback(const localizer::BeaconMatch::ConstPtr& msg) {
        if (!collecting_) return;
        
        for (const auto& cluster : msg->clusters) {
            // O beacon_match_name já vem do beacon_detector após o matching
            // beacon_detector faz: clustering -> matching -> atribui beacon_match_name
            std::string beacon_name = cluster.beacon_match_name;
            
            if (beacon_name.empty()) {
                ROS_WARN_THROTTLE(1.0, "[MeasurementCollector] Received cluster with empty beacon_match_name, skipping");
                continue;
            }
            
            // Usar TODOS os pontos do cluster, não apenas o centróide
            // Cada ponto é uma medição independente do LiDAR
            for (const auto& point : cluster.points) {
                // Calcular r e theta para cada ponto
                double r = std::hypot(point.x, point.y);
                double theta = normalizeAngle(std::atan2(point.y, point.x));
                
                Measurement meas;
                meas.r = r;
                meas.theta = theta;
                meas.stamp = msg->header.stamp;
                
                // Guardar na iteração atual (organizado por beacon)
                current_data_[beacon_name].push_back(meas);
            }
        }
    }
    
    bool startCollection(lidar_calibration::MeasurementCollection::Request& req,
                        lidar_calibration::MeasurementCollection::Response& res) {
        if (collecting_) {
            res.success = false;
            res.message = "Collection already in progress";
            return true;
        }
        
        // Limpar dados da iteração anterior
        current_data_.clear();
        
        collecting_ = true;
        current_iteration_++;
        res.success = true;
        res.message = "Started collecting measurements for iteration " + std::to_string(current_iteration_);
        ROS_INFO("[MeasurementCollector] Started collecting measurements (iteration %d)", current_iteration_);
        return true;
    }
    
    bool stopCollection(lidar_calibration::MeasurementCollection::Request& req,
                      lidar_calibration::MeasurementCollection::Response& res) {
        if (!collecting_) {
            res.success = false;
            res.message = "Collection not in progress";
            return true;
        }
        
        collecting_ = false;
        
        // Mostrar resumo dos dados recolhidos
        ROS_INFO("[MeasurementCollector] Stopped collection. Summary:");
        int total = 0;
        for (const auto& pair : current_data_) {
            int count = pair.second.size();
            total += count;
            ROS_INFO("  - Beacon '%s': %d measurements", pair.first.c_str(), count);
        }
        ROS_INFO("  Total: %d measurements", total);
        
        // Guardar em YAML
        if (saveToYAML()) {
            res.success = true;
            res.message = "Stopped collecting and saved to YAML (iteration " + 
                         std::to_string(current_iteration_) + ")";
            ROS_INFO("[MeasurementCollector] Saved iteration %d to YAML", current_iteration_);
        } else {
            res.success = false;
            res.message = "Failed to save to YAML";
        }
        
        // Limpar dados atuais (já foram guardados)
        current_data_.clear();
        
        return true;
    }
    
    bool saveToYAML() {
        try {
            YAML::Node root;
            
            // Ler ficheiro existente se existir
            std::ifstream file_in(yaml_file_);
            if (file_in.good()) {
                file_in.close();
                root = YAML::LoadFile(yaml_file_);
            }
            
            // Garantir que existe nó "iterations"
            if (!root["iterations"]) {
                root["iterations"] = YAML::Node(YAML::NodeType::Sequence);
            }
            
            // Adicionar nova iteração
            YAML::Node iteration;
            iteration["iteration_id"] = current_iteration_;
            
            for (const auto& beacon_pair : current_data_) {
                const std::string& beacon_name = beacon_pair.first;
                YAML::Node beacon_data;
                
                std::vector<double> r_values, theta_values;
                for (const auto& meas : beacon_pair.second) {
                    r_values.push_back(meas.r);
                    theta_values.push_back(meas.theta);
                }
                
                beacon_data["r"] = r_values;
                beacon_data["theta"] = theta_values;
                beacon_data["num_samples"] = (int)r_values.size();
                
                iteration["beacons"][beacon_name] = beacon_data;
            }
            
            root["iterations"].push_back(iteration);
            
            // Guardar ficheiro
            std::ofstream file_out(yaml_file_);
            file_out << root;
            file_out.close();
            
            ROS_INFO("[MeasurementCollector] Saved to %s", yaml_file_.c_str());
            return true;
            
        } catch (const std::exception& e) {
            ROS_ERROR("[MeasurementCollector] Error saving YAML: %s", e.what());
            return false;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "measurement_collector_node");
    
    MeasurementCollector collector;
    
    ROS_INFO("[MeasurementCollector] Node running. Waiting for service calls...");
    ros::spin();
    
    return 0;
}


