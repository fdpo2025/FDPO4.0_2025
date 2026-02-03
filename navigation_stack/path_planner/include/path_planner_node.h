#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <mutex>

struct P { int x, y; };

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
  bool running_ = false;          // evita correr dois planners ao mesmo tempo
  std::string last_sequence_;     // evita repetir a mesma sequência

  std::unordered_map<int, P> coords_;
  std::unordered_set<int> forced_keep_;

  void onColorSequence(const std_msgs::String::ConstPtr& msg);

  // corre o teu algoritmo (adaptado do teu main antigo)
  void runPlanner(const std::string& comb);

  // publica um path (nós -> nav_msgs/Path)
  void publishPlannedPath(const std::vector<int>& node_path);

  static std::unordered_map<int, P> buildCoords();
  static std::unordered_set<int> buildForcedKeep();
};
