#include "warehouse_approach_topology.h"

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>

#include <cctype>
#include <fstream>
#include <vector>

namespace {

// Copia todos os pares de um mapa para um vector ANTES de processar. Evita SIGSEGV no
// yaml-cpp ao chamar YAML::Dump ou outras operações durante operator++ do iterador.
std::vector<std::pair<YAML::Node, YAML::Node>> snapshotMap(const YAML::Node& map)
{
    std::vector<std::pair<YAML::Node, YAML::Node>> rows;
    if (!map.IsDefined() || !map.IsMap()) return rows;
    for (YAML::const_iterator it = map.begin(); it != map.end(); ++it)
        rows.emplace_back(it->first, it->second);
    return rows;
}

std::string trimWhitespace(std::string s)
{
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r' || s.back() == '\n'))
        s.pop_back();
    size_t i = 0;
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n'))
        ++i;
    return s.substr(i);
}

bool tryYamlKeyToInt(const YAML::Node& key, int* out)
{
    if (!key.IsDefined() || out == nullptr) return false;

    try {
        *out = key.as<int>();
        return true;
    } catch (const YAML::Exception&) {
    }

    try {
        std::string s = trimWhitespace(key.as<std::string>());
        *out = std::stoi(s);
        return true;
    } catch (const YAML::Exception&) {
    } catch (const std::invalid_argument&) {
    } catch (const std::out_of_range&) {
    }
    return false;
}

// Descricao de chave sem iterar mapas pais; evita YAML::Dump salvo como ultimo recurso.
std::string describeKey(const YAML::Node& key)
{
    if (!key.IsDefined()) return "(undefined)";
    try {
        return std::to_string(key.as<int>());
    } catch (const YAML::Exception&) {
    }
    try {
        return trimWhitespace(key.as<std::string>());
    } catch (const YAML::Exception&) {
    }
    try {
        return YAML::Dump(key);
    } catch (...) {
        return "(unprintable)";
    }
}

std::string describeValue(const YAML::Node& val)
{
    if (!val.IsDefined()) return "(undefined)";
    try {
        return trimWhitespace(val.as<std::string>());
    } catch (const YAML::Exception&) {
    }
    try {
        return YAML::Dump(val);
    } catch (...) {
        return "(unprintable)";
    }
}

bool parseSideString(const std::string& raw, bool* right_out)
{
    std::string s = trimWhitespace(raw);
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
        if (parseSideString(s, right_out)) return true;
    } catch (const YAML::Exception&) {
    }

    try {
        if (val.as<bool>()) {
            *right_out = true;
            return true;
        }
    } catch (const YAML::Exception&) {
    }

    ROS_WARN("warehouse_approach_topology: invalid side value (expected left/right): %s",
             describeValue(val).c_str());
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

        for (const auto& wrow : snapshotMap(root)) {
            const YAML::Node& nmap = wrow.second;
            if (!nmap.IsMap()) {
                ROS_WARN("warehouse_approach_topology: skip root entry (not a map), key=%s",
                         describeKey(wrow.first).c_str());
                continue;
            }

            int w = 0;
            if (!tryYamlKeyToInt(wrow.first, &w)) {
                ROS_WARN("warehouse_approach_topology: skip root key (not int id): %s",
                         describeKey(wrow.first).c_str());
                continue;
            }

            for (const auto& nrow : snapshotMap(nmap)) {
                const YAML::Node& pmap = nrow.second;
                if (!pmap.IsMap()) {
                    ROS_WARN("warehouse_approach_topology: skip W=%d neighbor (not a map), key=%s",
                             w, describeKey(nrow.first).c_str());
                    continue;
                }

                int n = 0;
                if (!tryYamlKeyToInt(nrow.first, &n)) {
                    ROS_WARN("warehouse_approach_topology: skip neighbor key under W=%d: %s",
                             w, describeKey(nrow.first).c_str());
                    continue;
                }

                for (const auto& prow : snapshotMap(pmap)) {
                    int p = 0;
                    if (!tryYamlKeyToInt(prow.first, &p)) {
                        ROS_WARN(
                            "warehouse_approach_topology: skip predecessor key W=%d N=%d: %s",
                            w, n, describeKey(prow.first).c_str());
                        continue;
                    }

                    bool right = false;
                    if (!yamlValueToRight(prow.second, &right)) {
                        ROS_WARN("warehouse_approach_topology: skip W=%d N=%d P=%d bad value",
                                 w, n, p);
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
            "warehouse_approach_topology: no entry for W=%d N=%d P=%d - assuming left (+100)",
            warehouse, neighbor, predecessor);
        return false;
    }
    return pit->second;
}
