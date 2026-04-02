#include "warehouse_approach_topology.h"

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>

WarehouseApproachTopology::WarehouseApproachTopology(const std::string& yaml_path)
{
    YAML::Node root = YAML::LoadFile(yaml_path);
    for (auto wit = root.begin(); wit != root.end(); ++wit) {
        int w = std::stoi(wit->first.as<std::string>());
        const YAML::Node& nmap = wit->second;
        for (auto nit = nmap.begin(); nit != nmap.end(); ++nit) {
            int n = std::stoi(nit->first.as<std::string>());
            const YAML::Node& pmap = nit->second;
            for (auto pit = pmap.begin(); pit != pmap.end(); ++pit) {
                int p = std::stoi(pit->first.as<std::string>());
                std::string side = pit->second.as<std::string>();
                bool right = (side == "right" || side == "direita" || side == "d");
                table_[w][n][p] = right;
            }
        }
    }
}

bool WarehouseApproachTopology::isRightApproach(
    int warehouse, int neighbor, int predecessor) const
{
    auto wit = table_.find(warehouse);
    if (wit == table_.end()) return false;
    auto nit = wit->second.find(neighbor);
    if (nit == wit->second.end()) return false;
    auto pit = nit->second.find(predecessor);
    if (pit == nit->second.end()) {
        ROS_WARN_THROTTLE(
            30.0,
            "warehouse_approach_topology: sem entrada para W=%d N=%d P=%d — a assumir esquerda (+100)",
            warehouse, neighbor, predecessor);
        return false;
    }
    return pit->second;
}
