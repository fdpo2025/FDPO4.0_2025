#include "warehouse_approach_topology.h"

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>

#include <cctype>
#include <fstream>

namespace {

bool tryYamlKeyToInt(const YAML::Node& key, int* out)
{
    if (!key.IsDefined() || out == nullptr) return false;

    try {
        *out = key.as<int>();
        return true;
    } catch (const YAML::Exception&) {
    }

    try {
        *out = std::stoi(key.as<std::string>());
        return true;
    } catch (const YAML::Exception&) {
    } catch (const std::invalid_argument&) {
    } catch (const std::out_of_range&) {
    }
    return false;
}

std::string yamlNodeSnippet(const YAML::Node& n)
{
    try {
        return YAML::Dump(n);
    } catch (...) {
        return "(dump failed)";
    }
}

bool parseSideString(const std::string& raw, bool* right_out)
{
    std::string s = raw;
    for (char& c : s)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (s == "right" || s == "direita" || s == "d") {
        *right_out = true;
        return true;
    }
    if (s == "left" || s == "esquerda" || s == "e" || s == "l") {
        *right_out = false;
        return true;
    }
    return false;
}

bool yamlValueToRight(const YAML::Node& val, bool* right_out)
{
    if (!val.IsDefined() || right_out == nullptr) return false;

    try {
        std::string s = val.as<std::string>();
        return parseSideString(s, right_out);
    } catch (const YAML::Exception&) {
    }

    try {
        if (val.as<bool>()) {
            *right_out = true;
            return true;
        }
    } catch (const YAML::Exception&) {
    }

    ROS_WARN_STREAM("warehouse_approach_topology: invalid side value, expected left/right: "
                    << yamlNodeSnippet(val));
    return false;
}

}  // namespace

void WarehouseApproachTopology::loadFromFile(const std::string& yaml_path)
{
    table_.clear();

    std::ifstream test(yaml_path.c_str());
    if (!test.good()) {
        ROS_ERROR("warehouse_approach_topology: missing or unreadable file: %s", yaml_path.c_str());
        return;
    }
    test.close();

    try {
        YAML::Node root = YAML::LoadFile(yaml_path);
        if (!root.IsDefined() || !root.IsMap()) {
            ROS_ERROR("warehouse_approach_topology: root must be a map: %s", yaml_path.c_str());
            return;
        }

        for (auto wit = root.begin(); wit != root.end(); ++wit) {
            const YAML::Node& nmap = wit->second;
            if (!nmap.IsMap()) {
                ROS_WARN_STREAM("warehouse_approach_topology: skip root entry (not a map), key="
                                << yamlNodeSnippet(wit->first));
                continue;
            }

            int w = 0;
            if (!tryYamlKeyToInt(wit->first, &w)) {
                ROS_WARN_STREAM("warehouse_approach_topology: skip root key (not int id): "
                                << yamlNodeSnippet(wit->first));
                continue;
            }

            for (auto nit = nmap.begin(); nit != nmap.end(); ++nit) {
                const YAML::Node& pmap = nit->second;
                if (!pmap.IsMap()) {
                    ROS_WARN_STREAM("warehouse_approach_topology: skip W=" << w
                                                                          << " neighbor entry (not a map) key="
                                                                          << yamlNodeSnippet(nit->first));
                    continue;
                }

                int n = 0;
                if (!tryYamlKeyToInt(nit->first, &n)) {
                    ROS_WARN_STREAM("warehouse_approach_topology: skip neighbor key under W=" << w << ": "
                                                                                            << yamlNodeSnippet(
                                                                                                   nit->first));
                    continue;
                }

                for (auto pit = pmap.begin(); pit != pmap.end(); ++pit) {
                    int p = 0;
                    if (!tryYamlKeyToInt(pit->first, &p)) {
                        ROS_WARN_STREAM("warehouse_approach_topology: skip predecessor key W=" << w << " N=" << n
                                                                                             << ": "
                                                                                             << yamlNodeSnippet(
                                                                                                    pit->first));
                        continue;
                    }

                    bool right = false;
                    if (!yamlValueToRight(pit->second, &right)) {
                        ROS_WARN_STREAM("warehouse_approach_topology: skip W=" << w << " N=" << n << " P=" << p
                                                                                             << " bad value");
                        continue;
                    }
                    table_[w][n][p] = right;
                }
            }
        }
        ROS_INFO("warehouse_approach_topology: loaded from %s", yaml_path.c_str());
    } catch (const YAML::Exception& e) {
        ROS_ERROR("warehouse_approach_topology: YAML error in %s: %s", yaml_path.c_str(), e.what());
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
            "warehouse_approach_topology: no entry for W=%d N=%d P=%d — assuming left (+100)",
            warehouse, neighbor, predecessor);
        return false;
    }
    return pit->second;
}
