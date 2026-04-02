#pragma once

#include <string>
#include <unordered_map>

// Lado de aproximação ao warehouse definido pelo grafo (imagem), não por coordenadas.
// predecessor -> neighbor -> warehouse: "right" = ID base, "left" = ID + 100.
class WarehouseApproachTopology {
public:
    explicit WarehouseApproachTopology(const std::string& yaml_path);

    // true = direita (usar warehouse_node), false = esquerda (warehouse_node + 100)
    bool isRightApproach(int warehouse, int neighbor, int predecessor) const;

private:
    // warehouse -> neighbor -> predecessor -> is_right
    std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, bool>>> table_;
};
