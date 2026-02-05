#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <xmlrpcpp/XmlRpcValue.h> // Necessário para processar o dicionário do YAML

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <vector>
#include <mutex>
#include <thread>
#include <algorithm>
#include <queue>

struct P { long long x, y; }; // Alterado para long long para manter precisão com coordenadas do YAML

class PathPlannerNode
{
public:
  explicit PathPlannerNode(ros::NodeHandle& nh);

private:
  ros::NodeHandle nh_;

  ros::Subscriber color_seq_sub_;
  ros::Publisher planned_paths_pub_;

  std::string frame_id_ = "map";

  std::mutex mtx_;
  bool running_ = false;          
  std::string last_sequence_;     

  // Estruturas de Dados do Grafo
  std::unordered_map<int, P> coords_;
  std::unordered_map<int, std::vector<int>> adj_;
  std::unordered_set<int> forced_keep_;

  // Mapeamento de Funções dos Nós (carregado via YAML)
  std::unordered_set<int> input_nodes_;   // Nós 0, 1, 2, 3
  std::unordered_set<int> output_nodes_;  // Nós 35, 36, 37, 38
  std::unordered_set<int> proc_in_nodes_; // Nós 13, 17, 20, 24
  std::unordered_map<int, int> spawn_map_; // 17 -> 18, 13 -> 14, etc.

  void onColorSequence(const std_msgs::String::ConstPtr& msg);
  
  // Carrega toda a configuração do factory_graph.yaml
  void loadConfig();

  // Executa o algoritmo de planeamento
  void runPlanner(const std::string& comb);

  // Auxiliares de navegação
  std::vector<int> shortestPathBFS(int start, int goal);
  std::vector<int> simplifyPath(const std::vector<int>& path);

  // Publica o caminho final
  void publishPlannedPath(const std::vector<int>& node_path);
};