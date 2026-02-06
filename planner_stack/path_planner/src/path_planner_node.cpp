#include "path_planner_node.h"

using namespace std;

// =================== HELPERS DE GEOMETRIA ===================

static bool colinearAndSameDir(const P& a, const P& b, const P& c) {
    long long abx = b.x - a.x, aby = b.y - a.y;
    long long bcx = c.x - b.x, bcy = c.y - b.y;
    long long cross = abx * bcy - aby * bcx;
    if (cross != 0) return false;
    return (abx * bcx + aby * bcy) > 0;
}

std::vector<int> PathPlannerNode::simplifyPath(const std::vector<int>& path) {
    if (path.size() <= 2) return path;

    std::vector<int> out;
    out.push_back(path.front());

    for (size_t i = 1; i + 1 < path.size(); ++i) {
        int cur = path[i];
        if (forced_keep_.count(cur)) {
            out.push_back(cur);
            continue;
        }
        if (colinearAndSameDir(coords_[path[i-1]], coords_[cur], coords_[path[i+1]])) continue;
        out.push_back(cur);
    }
    out.push_back(path.back());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

// =================== LÓGICA DO GRAFO ===================

void PathPlannerNode::loadConfig() {
    XmlRpc::XmlRpcValue points_map, forced_list;
    
    if (!nh_.getParam("points_map", points_map)) {
        ROS_ERROR("ERRO: Nao foi possivel carregar 'points_map' de factory_graph.yaml");
        return;
    }

    for (auto it = points_map.begin(); it != points_map.end(); ++it) {
        int id = std::stoi(it->first);
        XmlRpc::XmlRpcValue& data = it->second;

        // 1. Coordenadas (Multiplicar por 1000 para manter precisão em long long se necessário)
        coords_[id] = { (long long)(double(data["x"]) * 1000), (long long)(double(data["y"]) * 1000) };

        // 2. Adjacências
        if (data.hasMember("links")) {
            for (int i = 0; i < data["links"].size(); ++i) {
                adj_[id].push_back(static_cast<int>(data["links"][i]));
            }
        }

        // 3. Metadados de lógica
        if (data.hasMember("type")) {
            string t = static_cast<string>(data["type"]);
            if (t == "input") input_nodes_.insert(id);
            if (t == "output") output_nodes_.insert(id);
            if (t == "proc_in") {
                proc_in_nodes_.insert(id);
                spawn_map_[id] = static_cast<int>(data["spawns"]);
            }
        }
    }

    if (nh_.getParam("forced_keep_nodes", forced_list)) {
        for (int i = 0; i < forced_list.size(); ++i) 
            forced_keep_.insert(static_cast<int>(forced_list[i]));
    }
}

vector<int> PathPlannerNode::shortestPathBFS(int start, int goal) {
    if (start == goal) return {start};
    queue<int> q;
    unordered_map<int, int> parent;
    unordered_set<int> vis = {start};
    q.push(start);
    parent[start] = -1;

    while (!q.empty()) {
        int u = q.front(); q.pop();
        for (int v : adj_[u]) {
            if (vis.count(v)) continue;
            vis.insert(v);
            parent[v] = u;
            if (v == goal) {
                vector<int> path;
                for (int cur = goal; cur != -1; cur = parent[cur]) path.push_back(cur);
                reverse(path.begin(), path.end());
                return path;
            }
            q.push(v);
        }
    }
    return {};
}

// =================== CORE PLANNER ===================

PathPlannerNode::PathPlannerNode(ros::NodeHandle& nh) : nh_(nh), running_(false) {
    color_seq_sub_ = nh_.subscribe("/color_sequence", 1, &PathPlannerNode::onColorSequence, this);
    planned_paths_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/planned_paths", 100, true);
    loadConfig();
    ROS_INFO("Planner inicializado com %zu nos carregados.", coords_.size());
}

void PathPlannerNode::runPlanner(const std::string& comb) {
    map<int, char> boxesAt;
    vector<int> inputs = {0, 1, 2, 3}; // Conforme imagem
    for (size_t i = 0; i < comb.size() && i < inputs.size(); ++i) boxesAt[inputs[i]] = comb[i];

    unordered_set<int> usedWarehouse;
    int current_pos = 31; // Nó central como fallback inicial
    bool carrying = false;
    char carry_type = '?';
    std::vector<int> full_path = {current_pos};

    auto moveRobot = [&](int to) {
        if (to == current_pos) return;
        auto path = shortestPathBFS(current_pos, to);
        if (path.empty()) return;
        auto simp = simplifyPath(path);
        for (size_t i = (full_path.back() == simp.front() ? 1 : 0); i < simp.size(); ++i)
            full_path.push_back(simp[i]);
        current_pos = to;
    };

    auto pathLen = [&](int a, int b) -> int {
        auto p = shortestPathBFS(a, b);
        return p.empty() ? INT_MAX : (int)p.size();
    };

    auto hasValidDestination = [&](char t) -> bool {
        if (t == 'B') {
            for (int out : {35,36,37,38})
                if (!usedWarehouse.count(out)) return true;
            return false;
        }
        if (t == 'R') {
            for (int proc : {17, 24}) {
                if (!proc_in_nodes_.count(proc)) continue;
                if (!spawn_map_.count(proc)) continue;
                int sp = spawn_map_[proc];
                if (boxesAt.find(sp) == boxesAt.end()) return true; // spawn livre
            }
            return false;
        }
        if (t == 'G') {
            for (int proc : {13, 20}) {
                if (!proc_in_nodes_.count(proc)) continue;
                if (!spawn_map_.count(proc)) continue;
                int sp = spawn_map_[proc];
                if (boxesAt.find(sp) == boxesAt.end()) return true;
            }
            return false;
        }
        return false;
    };


    while (!boxesAt.empty() || carrying) {
        if (!carrying) {
            // Escolher sempre a caixa mais próxima (menor caminho BFS)
            int bestNode = -1;
            int bestDist = INT_MAX;

            for (auto const& [node, type] : boxesAt) {
                if (!hasValidDestination(type)) continue;   // <- evita deadlock
                int d = pathLen(current_pos, node);
                if (d < bestDist) {
                    bestDist = d;
                    bestNode = node;
                }
            }

            if (bestNode == -1) break; // não há caixas entregáveis agora

            moveRobot(bestNode);
            carry_type = boxesAt[bestNode];
            carrying = true;
            boxesAt.erase(bestNode);


        } else {
            int target = -1;

            auto spawnFree = [&](int proc) {
                int sp = spawn_map_[proc];
                return boxesAt.find(sp) == boxesAt.end(); // só permite se o spawn estiver livre
            };
            
            if (carry_type == 'B') {
                // Azuis -> armazém 35..38 (primeiro livre)
                for (int out : {35, 36, 37, 38}) {
                    if (!usedWarehouse.count(out)) { target = out; break; }
                }
            } 
            else if (carry_type == 'R') {
                // Vermelhas -> 17 ou 24, mas só se 18/25 respetivamente estiver livre
                std::vector<int> candidates = {17, 24};

                int best = INT_MAX;
                for (int proc : candidates) {
                    if (!proc_in_nodes_.count(proc)) continue;          // segurança
                    if (!spawn_map_.count(proc)) continue;              // segurança
                    if (!spawnFree(proc)) continue;                     // regra crítica

                    int d = pathLen(current_pos, proc);
                    if (d < best) { best = d; target = proc; }
                }
            }
            else if (carry_type == 'G') {
                // Verdes -> 13 ou 20, mas só se 14/21 respetivamente estiver livre
                std::vector<int> candidates = {13, 20};

                int best = INT_MAX;
                for (int proc : candidates) {
                    if (!proc_in_nodes_.count(proc)) continue;
                    if (!spawn_map_.count(proc)) continue;
                    if (!spawnFree(proc)) continue;

                    int d = pathLen(current_pos, proc);
                    if (d < best) { best = d; target = proc; }
                }
            }

            if (target == -1) break; // sem destinos válidos (deadlock)
            moveRobot(target);

            if (output_nodes_.count(target)) {
                usedWarehouse.insert(target);
            } else if (proc_in_nodes_.count(target)) {
                // Transformação depende do tipo carregado
                char nextType = '?';
                if (carry_type == 'R') nextType = 'G';
                else if (carry_type == 'G') nextType = 'B';

                if (nextType != '?') {
                    boxesAt[spawn_map_[target]] = nextType;
                }
            }

            carrying = false;
        }

    }

    publishPlannedPath(full_path);
}

void PathPlannerNode::onColorSequence(const std_msgs::String::ConstPtr& msg) {
    if (running_) return;
    running_ = true;
    std::string seq = msg->data;
    std::thread([this, seq]() {
        runPlanner(seq);
        running_ = false;
    }).detach();
}

void PathPlannerNode::publishPlannedPath(const std::vector<int>& node_path) {
    std_msgs::Int32MultiArray msg;
    msg.data = node_path;
    planned_paths_pub_.publish(msg);
}