#include "warehouse_approach_topology.h"

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>

#include <cctype>
#include <fstream>
#include <stdexcept>

namespace {

int yamlKeyToInt(const YAML::Node& key)
{
    if (!key.IsScalar())
        throw std::runtime_error("chave não escalar no warehouse_approach_topology");
    try {
        return key.as<int>();
    } catch (const YAML::Exception&) {
        return std::stoi(key.as<std::string>());
    }
}

std::string yamlValueToSideString(const YAML::Node& val)
{
    if (!val.IsScalar()) {
        throw std::runtime_error("valor de lado (left/right) inválido");
    }
    std::string s = val.as<std::string>();
    for (char& c : s)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

}  // namespace

void WarehouseApproachTopology::loadFromFile(const std::string& yaml_path)
{
    table_.clear();

    std::ifstream test(yaml_path.c_str());
    if (!test.good()) {
        ROS_ERROR("warehouse_approach_topology: ficheiro inexistente ou ilegível: %s",
                  yaml_path.c_str());
        return;
    }
    test.close();

    try {
        YAML::Node root = YAML::LoadFile(yaml_path);
        if (!root.IsDefined() || !root.IsMap()) {
            ROS_ERROR(
                "warehouse_approach_topology: raiz inválida (esperado mapa): %s",
                yaml_path.c_str());
            return;
        }

        for (auto wit = root.begin(); wit != root.end(); ++wit) {
            int w = yamlKeyToInt(wit->first);
            const YAML::Node& nmap = wit->second;
            if (!nmap.IsMap()) {
                ROS_WARN("warehouse_approach_topology: ignorar warehouse %d (não é mapa)", w);
                continue;
            }
            for (auto nit = nmap.begin(); nit != nmap.end(); ++nit) {
                int n = yamlKeyToInt(nit->first);
                const YAML::Node& pmap = nit->second;
                if (!pmap.IsMap()) {
                    ROS_WARN("warehouse_approach_topology: ignorar W=%d N=%d (não é mapa)", w, n);
                    continue;
                }
                for (auto pit = pmap.begin(); pit != pmap.end(); ++pit) {
                    int p = yamlKeyToInt(pit->first);
                    std::string side = yamlValueToSideString(pit->second);
                    bool right = (side == "right" || side == "direita" || side == "d");
                    table_[w][n][p] = right;
                }
            }
        }
        ROS_INFO("warehouse_approach_topology: carregado de %s", yaml_path.c_str());
    } catch (const YAML::Exception& e) {
        ROS_ERROR("warehouse_approach_topology: YAML error em %s: %s", yaml_path.c_str(),
                  e.what());
    } catch (const std::exception& e) {
        ROS_ERROR("warehouse_approach_topology: %s", e.what());
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
