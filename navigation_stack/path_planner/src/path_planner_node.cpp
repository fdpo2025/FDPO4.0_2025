#include "path_planner/path_planner_node.h"

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <limits>
#include <map>
#include <stdexcept>
#include <thread>

using namespace std;

// =================== TEU CÓDIGO (helpers) ===================

static bool colinearAndSameDir(const P& a, const P& b, const P& c) {
    long long abx = b.x - a.x, aby = b.y - a.y;
    long long bcx = c.x - b.x, bcy = c.y - b.y;

    long long cross = abx * bcy - aby * bcx;
    if (cross != 0) return false;

    long long dot = abx * bcx + aby * bcy;
    return dot > 0;
}

static std::vector<int> simplifyPath(
    const std::vector<int>& path,
    const std::unordered_map<int, P>& coord,
    const std::unordered_set<int>& forcedKeep
) {
    if (path.size() <= 2) return path;

    auto getP = [&](int node) -> const P& {
        auto it = coord.find(node);
        if (it == coord.end()) throw std::runtime_error("Falta coordenada para nó " + std::to_string(node));
        return it->second;
    };

    std::vector<int> out;
    out.reserve(path.size());
    out.push_back(path.front());

    for (size_t i = 1; i + 1 < path.size(); ++i) {
        int prev = path[i - 1];
        int cur  = path[i];
        int next = path[i + 1];

        if (forcedKeep.count(cur)) {
            out.push_back(cur);
            continue;
        }

        const P& A = getP(prev);
        const P& B = getP(cur);
        const P& C = getP(next);

        if (colinearAndSameDir(A, B, C)) {
            continue;
        }

        out.push_back(cur);
    }

    out.push_back(path.back());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

static void addEdge(unordered_map<int, vector<int>>& adj, int a, int b) {
    adj[a].push_back(b);
    adj[b].push_back(a);
}

static vector<int> shortestPathBFS(const unordered_map<int, vector<int>>& adj, int start, int goal) {
    if (start == goal) return {start};
    if (!adj.count(start) || !adj.count(goal)) return {};

    queue<int> q;
    unordered_set<int> vis;
    unordered_map<int,int> parent;

    vis.insert(start);
    parent[start] = -1;
    q.push(start);

    while(!q.empty()) {
        int u = q.front(); q.pop();
        for (int v : adj.at(u)) {
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

static int distBFS(const unordered_map<int, vector<int>>& adj, int a, int b) {
    auto p = shortestPathBFS(adj, a, b);
    if (p.empty()) return numeric_limits<int>::max()/4;
    return (int)p.size() - 1;
}

static bool isWarehouse(int n) { return n==62 || n==63 || n==64 || n==65; }
static bool isGreenDrop(int n) { return n==7 || n==14; }
static bool isRedDrop(int n)   { return n==11 || n==18; }

static int spawnedBlueFromGreenDrop(int n) { return (n==7) ? 8 : 15; }
static int spawnedGreenFromRedDrop(int n)  { return (n==11) ? 12 : 19; }

struct Robot {
    int pos;
    bool carrying = false;
    char type = '?';
};

static bool validSeq(const std::string& s)
{
  if (s.size() != 4) return false;
  for (char c : s) if (c!='B' && c!='G' && c!='R') return false;
  return true;
}

// =================== CLASSE ROS ===================

PathPlannerNode::PathPlannerNode(ros::NodeHandle& nh)
  : nh_(nh)
{
  nh_.param<std::string>("frame_id", frame_id_, std::string("map"));

  color_seq_sub_ = nh_.subscribe("/color_sequence", 1, &PathPlannerNode::onColorSequence, this);
  planned_paths_pub_ = nh_.advertise<nav_msgs::Path>("/planned_paths", 1, true); // latched

  coords_ = buildCoords();
  forced_keep_ = buildForcedKeep();

  ROS_INFO("PathPlannerNode pronto. A escutar /color_sequence e a publicar /planned_paths (frame_id=%s)",
           frame_id_.c_str());
}

void PathPlannerNode::onColorSequence(const std_msgs::String::ConstPtr& msg)
{
  const std::string seq = msg->data;

  if (!validSeq(seq))
  {
    ROS_WARN("Sequencia invalida em /color_sequence: '%s' (esperado 4 chars em {B,G,R})", seq.c_str());
    return;
  }

  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (running_)
    {
      ROS_WARN("Planner ainda a correr. Ignorando nova sequencia: %s", seq.c_str());
      return;
    }
    if (!last_sequence_.empty() && seq == last_sequence_)
    {
      ROS_INFO("Sequencia repetida (%s). Ignorando.", seq.c_str());
      return;
    }
    running_ = true;
    last_sequence_ = seq;
  }

  ROS_INFO("Recebi sequencia: %s. A iniciar planeamento...", seq.c_str());

  // corre em thread para não bloquear callbacks do ROS
  std::thread([this, seq](){
    try {
      runPlanner(seq);
    } catch (const std::exception& e) {
      ROS_ERROR("Erro no planner: %s", e.what());
    }

    std::lock_guard<std::mutex> lk(mtx_);
    running_ = false;
    ROS_INFO("Planeamento terminado para sequencia: %s", seq.c_str());
  }).detach();
}

void PathPlannerNode::publishPlannedPath(const std::vector<int>& node_path)
{
  nav_msgs::Path msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id_;

  msg.poses.reserve(node_path.size());

  for (int node : node_path)
  {
    auto it = coords_.find(node);
    if (it == coords_.end())
      throw std::runtime_error("Falta coordenada para nó " + std::to_string(node));

    geometry_msgs::PoseStamped ps;
    ps.header = msg.header;
    ps.pose.position.x = it->second.x;
    ps.pose.position.y = it->second.y;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0;

    msg.poses.push_back(ps);
  }

  planned_paths_pub_.publish(msg);
}

// ============ O TEU MAIN ANTIGO -> AGORA É ISTO ============

void PathPlannerNode::runPlanner(const std::string& comb)
{
  unordered_map<int, vector<int>> adj;

  // ===== Grafo (igual ao teu) =====
  addEdge(adj, 0, 1); addEdge(adj, 1, 2); addEdge(adj, 2, 3); addEdge(adj, 3, 4);
  addEdge(adj, 4, 30); addEdge(adj, 30, 31); addEdge(adj, 31, 5);

  addEdge(adj, 50, 0); addEdge(adj, 0, 34); addEdge(adj, 34, 10);
  addEdge(adj, 10, 17); addEdge(adj, 17, 21);

  addEdge(adj, 4, 6); addEdge(adj, 6, 13); addEdge(adj, 13, 20); addEdge(adj, 20, 22);

  addEdge(adj, 5, 9); addEdge(adj, 9, 16); addEdge(adj, 16, 35); addEdge(adj, 35, 26);

  addEdge(adj, 6, 7); addEdge(adj, 8, 9);

  addEdge(adj, 10, 11);
  addEdge(adj, 12, 13);
  addEdge(adj, 13, 14);
  addEdge(adj, 15, 16);

  addEdge(adj, 17, 18); addEdge(adj, 19, 20);

  addEdge(adj, 21, 32); addEdge(adj, 32, 33); addEdge(adj, 33, 22);
  addEdge(adj, 22, 23); addEdge(adj, 23, 24); addEdge(adj, 24, 25); addEdge(adj, 25, 26);

  addEdge(adj, 23, 62); addEdge(adj, 24, 63); addEdge(adj, 25, 64); addEdge(adj, 26, 65);

  addEdge(adj, 51, 1); addEdge(adj, 52, 2); addEdge(adj, 53, 3);

  auto& coords = coords_;
  auto& forced = forced_keep_;

  // ===== Entrada: combinação inicial em 50,51,52,53 =====
  map<int,char> boxesAt;
  vector<int> startNodes = {50,51,52,53};
  for (int i = 0; i < 4; i++) boxesAt[startNodes[i]] = comb[i];

  unordered_set<int> usedWarehouse;

  Robot rb{ .pos = 21 };

  auto isFreeNode = [&](int node) -> bool {
      return boxesAt.find(node) == boxesAt.end();
  };

  auto greenDropFeasible = [&](int drop) -> bool {
      if (!isGreenDrop(drop)) return false;
      int spawnB = spawnedBlueFromGreenDrop(drop);
      return isFreeNode(spawnB);
  };

  auto redDropFeasible = [&](int drop) -> bool {
      if (!isRedDrop(drop)) return false;
      int spawnG = spawnedGreenFromRedDrop(drop);
      return isFreeNode(spawnG);
  };

  auto computeBlockingNodesToClear = [&]() -> unordered_set<int> {
      unordered_set<int> mustClear;

      bool hasAnyGreen = false;
      for (auto &kv : boxesAt) if (kv.second == 'G') { hasAnyGreen = true; break; }

      bool hasAnyRed = false;
      for (auto &kv : boxesAt) if (kv.second == 'R') { hasAnyRed = true; break; }

      if (hasAnyGreen) {
          bool occ8  = !isFreeNode(8);
          bool occ15 = !isFreeNode(15);
          if (occ8 && occ15) {
              mustClear.insert(8);
              mustClear.insert(15);
          }
      }

      if (hasAnyRed) {
          bool occ12 = !isFreeNode(12);
          bool occ19 = !isFreeNode(19);
          if (occ12 && occ19) {
              mustClear.insert(12);
              mustClear.insert(19);
          }
      }

      return mustClear;
  };

  auto moveRobot = [&](int to) -> bool {
      auto path = shortestPathBFS(adj, rb.pos, to);
      if (path.empty()) return false;

      auto simp = simplifyPath(path, coords, forced);

      // ✅ AQUI é onde "sempre que obtenho um path eu publico"
      publishPlannedPath(simp);

      rb.pos = to;
      return true;
  };

  auto doPick = [&](int node) {
      auto it = boxesAt.find(node);
      if (it == boxesAt.end()) throw runtime_error("Nao existe caixa no no " + to_string(node));
      rb.carrying = true;
      rb.type = it->second;
      boxesAt.erase(it);
  };

  auto chooseWarehouse = [&](int fromPos) -> int {
      vector<int> wh = {62,63,64,65};
      int best = -1, bestD = numeric_limits<int>::max();
      for (int w : wh) {
          if (usedWarehouse.count(w)) continue;
          int d = distBFS(adj, fromPos, w);
          if (d < bestD) { bestD = d; best = w; }
      }
      return best;
  };

  auto chooseDropForCarried = [&](int fromPos, char t) -> int {
      if (t == 'B') {
          return chooseWarehouse(fromPos);
      } else if (t == 'G') {
          vector<int> opts = {7,14};
          int best = -1, bestD = numeric_limits<int>::max();
          for (int d : opts) {
              if (!greenDropFeasible(d)) continue;
              int dist = distBFS(adj, fromPos, d);
              if (dist < bestD) { bestD = dist; best = d; }
          }
          return best;
      } else { // 'R'
          vector<int> opts = {11,18};
          int best = -1, bestD = numeric_limits<int>::max();
          for (int d : opts) {
              if (!redDropFeasible(d)) continue;
              int dist = distBFS(adj, fromPos, d);
              if (dist < bestD) { bestD = dist; best = d; }
          }
          return best;
      }
  };

  auto doDrop = [&]() {
      int n = rb.pos;
      char t = rb.type;

      if (t == 'B') {
          if (!isWarehouse(n) || usedWarehouse.count(n)) throw runtime_error("Drop BLUE invalido/ocupado");
          usedWarehouse.insert(n);
      }
      else if (t == 'G') {
          if (!isGreenDrop(n)) throw runtime_error("Drop GREEN invalido");
          int spawnB = spawnedBlueFromGreenDrop(n);
          if (!isFreeNode(spawnB)) throw runtime_error("Buffer ocupado (BLUE)");
          boxesAt[spawnB] = 'B';
      }
      else if (t == 'R') {
          if (!isRedDrop(n)) throw runtime_error("Drop RED invalido");
          int spawnG = spawnedGreenFromRedDrop(n);
          if (!isFreeNode(spawnG)) throw runtime_error("Buffer ocupado (GREEN)");
          boxesAt[spawnG] = 'G';
      }

      rb.carrying = false;
      rb.type = '?';
  };

  auto pickNextBoxNode = [&](int fromPos) -> int {
      auto mustClear = computeBlockingNodesToClear();
      if (!mustClear.empty()) {
          int bestNode = -1, bestD = numeric_limits<int>::max();
          for (int node : mustClear) {
              auto it = boxesAt.find(node);
              if (it == boxesAt.end()) continue;
              int d = distBFS(adj, fromPos, node);
              if (d < bestD) { bestD = d; bestNode = node; }
          }
          if (bestNode != -1) return bestNode;
      }

      vector<char> prio = {'B','R','G'};
      for (char wanted : prio) {
          int bestNode = -1;
          int bestD = numeric_limits<int>::max();

          for (auto &kv : boxesAt) {
              int node = kv.first;
              char t = kv.second;
              if (t != wanted) continue;

              if (t == 'G') {
                  if (!greenDropFeasible(7) && !greenDropFeasible(14)) continue;
              }
              if (t == 'R') {
                  if (!redDropFeasible(11) && !redDropFeasible(18)) continue;
              }

              int d = distBFS(adj, fromPos, node);
              if (d < bestD) { bestD = d; bestNode = node; }
          }

          if (bestNode != -1) return bestNode;
      }

      vector<int> buffers = {8, 15, 12, 19};
      int bestNode = -1, bestD = numeric_limits<int>::max();
      for (int b : buffers) {
          auto it = boxesAt.find(b);
          if (it == boxesAt.end()) continue;
          int d = distBFS(adj, fromPos, b);
          if (d < bestD) { bestD = d; bestNode = b; }
      }
      return bestNode;
  };

  // ===== Loop (igual ao teu) =====
  while (true) {
      if (!rb.carrying && boxesAt.empty()) break;

      if (!rb.carrying) {
          int nextNode = pickNextBoxNode(rb.pos);
          if (nextNode == -1) break;

          if (!moveRobot(nextNode)) throw runtime_error("Sem caminho para ir buscar caixa");

          doPick(nextNode);
      } else {
          int drop = chooseDropForCarried(rb.pos, rb.type);

          if ((rb.type == 'G' || rb.type == 'R') && drop == -1)
              throw runtime_error("Deadlock: sem destino viavel");

          if (rb.type == 'B' && drop == -1) break;

          if (!moveRobot(drop)) throw runtime_error("Sem caminho para entregar");

          doDrop();
      }

      if ((int)usedWarehouse.size() == 4 && !rb.carrying) break;
  }
}

// =================== coords/forced ===================

std::unordered_map<int, P> PathPlannerNode::buildCoords()
{
  std::unordered_map<int, P> c;

  c[0]={0,0}; c[1]={1,0}; c[2]={2,0}; c[3]={3,0}; c[4]={4,0}; c[30]={5,0}; c[31]={6,0}; c[5]={8,0};

  c[50]={0,1}; c[34]={0,-1}; c[10]={0,-2}; c[17]={0,-3}; c[21]={0,-4};

  c[51]={1,1}; c[52]={2,1}; c[53]={3,1};

  c[32]={1,-4}; c[33]={2,-4}; c[22]={4,-4}; c[23]={5,-4}; c[24]={6,-4}; c[25]={7,-4}; c[26]={8,-4};

  c[6]={4,-1}; c[13]={4,-2}; c[20]={4,-3};

  c[9]={8,-1}; c[16]={8,-2}; c[35]={8,-3};

  c[11]={1,-2}; c[12]={3,-2}; c[14]={5,-2}; c[15]={6,-2};

  c[18]={1,-3}; c[19]={3,-3};

  c[7]={5,-1}; c[8]={7,-1};

  c[62]={5,-5}; c[63]={6,-5}; c[64]={7,-5}; c[65]={8,-5};

  return c;
}

std::unordered_set<int> PathPlannerNode::buildForcedKeep()
{
  return {33,22,23, 3,4,30, 0};
}
