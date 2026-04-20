#include "custom_planner_node.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <regex>
#include <set>
#include <sstream>
#include <unordered_set>

namespace {
int approachNodeForInputShelf(int shelf) {
  switch (shelf) {
    case 0:
      return 8;
    case 1:
      return 9;
    case 2:
      return 10;
    case 3:
      return 11;
    default:
      return -1;
  }
}

int approach900NodeForApproach(int approach_normal) {
  switch (approach_normal) {
    case 8:
      return 908;
    case 9:
      return 909;
    case 10:
      return 910;
    case 11:
      return 911;
    default:
      return -1;
  }
}

/** Mission timeline encoding:
 *   physical node: v >= 0
 *   MSG_N        : v in [-1255, -1000]  (N = -1000 - v)
 *   WAIT_N       : v in [-2255, -2000]  (N = -2000 - v)
 */
bool isMsgLegToken(int v) { return v <= -1000 && v >= -1255; }
bool isWaitLegToken(int v) { return v <= -2000 && v >= -2255; }
int msgChannelOf(int v) { return -1000 - v; }
int waitChannelOf(int v) { return -2000 - v; }
int encodeMsg(int channel) { return -1000 - channel; }
int encodeWait(int channel) { return -2000 - channel; }

/** Graph node id in expanded leg (excludes WAIT_N and MSG tokens). Matches /planned_paths body. */
bool isPhysicalGraphNode(int v) { return v >= 0; }

}  // namespace

bool CustomPlannerNode::isWarehouseCoordinate(int node_id) {
  const int resolved = (node_id >= 100) ? node_id - 100 : node_id;
  static const std::unordered_set<int> kIds = {
      0,   1,   2,   3,                    // input (plan_handler_node.cpp)
      13,  14,  17,  18,  20,  21,  24,  25,  // process
      35,  36,  37,  38};                   // output
  return kIds.count(resolved) != 0;
}

#define CP_DBG(...)        \
  do {                     \
    if (debug_verbose_) {  \
      ROS_INFO(__VA_ARGS__); \
    }                      \
  } while (0)

CustomPlannerNode::CustomPlannerNode(ros::NodeHandle& nh) : nh_(nh) {
  nh_.param("debug_verbose", debug_verbose_, false);
  nh_.param("num_robots", num_robots_, 2);
  if (num_robots_ < 2) num_robots_ = 2;
  if (num_robots_ > 4) num_robots_ = 4;

  color_input_node_by_index_[0] = 0;
  color_input_node_by_index_[1] = 1;
  color_input_node_by_index_[2] = 2;
  color_input_node_by_index_[3] = 3;

  loadValidNodeIds();
  loadMissions();

  nh_.param<std::string>("color_sequence_topic", color_sequence_topic_, "/color_sequence");
  nh_.param<std::string>("this_current_pose_topic", this_current_pose_topic_, "/this_current_pose");
  nh_.param<std::string>("network_table_topic", network_table_topic_, "/pi_pico_driver/network_table");
  nh_.param<std::string>("my_timeline_index_topic", my_timeline_index_topic_,
                         "/custom_planner/my_timeline_index");

  mission_color_sub_ = nh_.subscribe(color_sequence_topic_, 1, &CustomPlannerNode::onMissionColorSequence, this);
  robot_identity_sub_ = nh_.subscribe("/robot_identity", 1, &CustomPlannerNode::onRobotIdentity, this);
  network_table_sub_ =
      nh_.subscribe(network_table_topic_, 20, &CustomPlannerNode::onNetworkTable, this);
  this_pose_sub_ = nh_.subscribe(this_current_pose_topic_, 50, &CustomPlannerNode::onThisCurrentPose, this);

  planned_paths_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/planned_paths", 100, true);
  std::string pause_topic;
  nh_.param<std::string>("nav_pause_after_wp_index_topic", pause_topic, std::string("/nav_pause_after_wp_index"));
  nav_pause_after_wp_index_pub_ = nh_.advertise<std_msgs::UInt32MultiArray>(pause_topic, 1, true);
  std::string pause_req_topic;
  nh_.param<std::string>("nav_route_pause_request_topic", pause_req_topic, std::string("/nav_route_pause_request"));
  nav_route_pause_request_sub_ =
      nh_.subscribe(pause_req_topic, 10, &CustomPlannerNode::onNavRoutePauseRequest, this);
  std::string consumed_topic;
  nh_.param<std::string>("nav_plan_waypoint_consumed_topic", consumed_topic,
                         std::string("/nav_plan_waypoint_consumed"));
  nav_plan_waypoint_consumed_sub_ =
      nh_.subscribe(consumed_topic, 50, &CustomPlannerNode::onNavPlanWaypointConsumed, this);
  mission_state_pub_ = nh_.advertise<std_msgs::String>("/custom_planner/state", 10, true);
  wt_send_pub_ = nh_.advertise<std_msgs::Bool>("/wt_send", 10, false);
  my_timeline_index_pub_ = nh_.advertise<std_msgs::UInt8>(my_timeline_index_topic_, 10, true);
  timeline_tokens_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/custom_planner/timeline_tokens", 1, true);
  timeline_cursor_pub_ = nh_.advertise<std_msgs::UInt32>("/custom_planner/timeline_cursor", 10, true);
  active_task_nav_nodes_pub_ =
      nh_.advertise<std_msgs::Int32MultiArray>("/custom_planner/active_task_nav_nodes", 1, true);
  last_executed_token_pub_ =
      nh_.advertise<std_msgs::Int32MultiArray>("/custom_planner/last_executed_token", 10, true);
  spawn_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/custom_planner/spawn_pose", 1, true);

  setState(STATE_IDLE);
  {
    std::lock_guard<std::mutex> lock(mtx_);
    publishMyTimelineIndexLocked();
  }

  int initial_robot_id = -1;
  nh_.param("initial_robot_id", initial_robot_id, -1);
  if (initial_robot_id >= 0) {
    applyRobotIdentity(initial_robot_id);
  }

  ROS_INFO(
      "custom_planner ready: identity via /robot_identity; color=%s this_pose=%s net=%s "
      "my_ti=%s (num_robots=%d)",
      color_sequence_topic_.c_str(), this_current_pose_topic_.c_str(),
      network_table_topic_.c_str(), my_timeline_index_topic_.c_str(), num_robots_);
}

bool CustomPlannerNode::loadMissions() {
  std::string missions_key = "missions_2";
  if (num_robots_ == 3) {
    missions_key = "missions_3";
  } else if (num_robots_ >= 4) {
    missions_key = "missions_4";
  }
  if (!nh_.getParam(missions_key, missions_root_)) {
    ROS_ERROR("Missing '%s' parameter. Load config/missions.yaml in launch.", missions_key.c_str());
    return false;
  }
  ROS_INFO("Missions loaded from %s (manga_1 / manga_2 / manga_3).", missions_key.c_str());
  return true;
}

void CustomPlannerNode::loadValidNodeIds() {
  valid_node_ids_.clear();
  XmlRpc::XmlRpcValue points_map;
  if (!nh_.getParam("points_map", points_map)) {
    ROS_WARN("points_map not found. Path validation disabled.");
    return;
  }
  for (auto it = points_map.begin(); it != points_map.end(); ++it) {
    try {
      valid_node_ids_.push_back(std::stoi(it->first));
    } catch (...) {}
  }
  std::sort(valid_node_ids_.begin(), valid_node_ids_.end());
}

std::string CustomPlannerNode::selectMangaKey(const std::string& color_sequence) const {
  bool has_r = false, has_g = false, has_b = false;
  bool all_b = !color_sequence.empty();
  for (char ch : color_sequence) {
    if (!std::isalpha(static_cast<unsigned char>(ch))) continue;
    const char u = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    has_r = has_r || (u == 'R');
    has_g = has_g || (u == 'G');
    has_b = has_b || (u == 'B');
    if (u != 'B') all_b = false;
  }
  if (has_r && has_g && has_b) return "manga_3";
  if (has_b && has_g) return "manga_2";
  if (all_b && has_b) return "manga_1";
  ROS_WARN("Color sequence not matching expected rules. Falling back to manga_1.");
  return "manga_1";
}

void CustomPlannerNode::onMissionColorSequence(const std_msgs::String::ConstPtr& msg) {
  startMission(msg->data);
}

void CustomPlannerNode::applyRobotIdentity(int new_id) {
  std::string replay;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    robot_id_ = new_id;
    replay = pending_color_sequence_;
    pending_color_sequence_.clear();
  }
  ROS_INFO("custom_planner robot identity=%d", new_id);
  if (!replay.empty()) startMission(replay);
}

void CustomPlannerNode::onRobotIdentity(const std_msgs::Int32::ConstPtr& msg) {
  applyRobotIdentity(msg->data);
}

void CustomPlannerNode::onNavRoutePauseRequest(const std_msgs::Bool::ConstPtr& msg) {
  (void)msg;
  // Deprecated path: waits are timeline-driven in custom_planner.
}

void CustomPlannerNode::onThisCurrentPose(const std_msgs::UInt32::ConstPtr& msg) {
  (void)msg;
  // Legacy segment completion signal no longer used.
}

void CustomPlannerNode::onNavPlanWaypointConsumed(const std_msgs::UInt32MultiArray::ConstPtr& msg) {
  if (msg->data.size() < 2) return;
  std::lock_guard<std::mutex> lock(mtx_);
  if (!has_active_task_ || !mission_route_published_) return;
  const uint32_t seq = msg->data[0];
  const uint32_t plan_index = msg->data[1];

  if (!active_nav_plan_seq_valid_) {
    active_nav_plan_seq_ = seq;
    active_nav_plan_seq_valid_ = true;
  }
  if (seq != active_nav_plan_seq_) return;

  const uint32_t consumed_count =
      plan_index + 1u + static_cast<uint32_t>(active_task_.plan_handler_skips_first_nav_waypoint);
  if (consumed_count <= consumed_nav_nodes_count_) return;
  consumed_nav_nodes_count_ = consumed_count;
  processTimelineLocked();
}

void CustomPlannerNode::startMission(const std::string& color_sequence) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (robot_id_ < 0) {
    pending_color_sequence_ = color_sequence;
    ROS_WARN("custom_planner: robot identity unknown, mission queued.");
    return;
  }
  last_color_sequence_ = color_sequence;
  const std::string manga_key = selectMangaKey(color_sequence);
  publishSpawnPose(manga_key);
  pending_tasks_.clear();
  has_active_task_ = false;
  active_task_ = MissionTask();
  last_executed_token_index_ = -1;
  last_executed_token_value_ = 0;

  // Reset sync state for the new mission.
  peer_timelines_.clear();
  observed_timeline_index_.assign(num_robots_, 0);
  ticks_on_channel_.clear();
  consumed_on_channel_.clear();
  waiting_on_channel_ = -1;
  my_timeline_index_ = 0;
  publishMyTimelineIndexLocked();
  publishActiveTaskDebugLocked();

  if (!buildPeerTimelinesLocked(manga_key, color_sequence)) {
    ROS_ERROR("custom_planner: failed to build peer timelines for manga %s.", manga_key.c_str());
    setState(STATE_IDLE);
    return;
  }
  if (!validateChannelsLocked()) {
    ROS_ERROR(
        "custom_planner: mission validation failed (ver logs). Missão abortada para prevenir deadlock.");
    setState(STATE_IDLE);
    return;
  }

  // The active robot's own tasks are derived from the raw YAML (leg-per-task), same as before.
  if (!missions_root_.valid() || !missions_root_.hasMember(manga_key)) {
    setState(STATE_IDLE);
    return;
  }
  const XmlRpc::XmlRpcValue manga_cfg = missions_root_[manga_key];
  const std::string robot_key = "robot_" + std::to_string(robot_id_);
  if (!manga_cfg.hasMember(robot_key)) {
    setState(STATE_IDLE);
    return;
  }
  const XmlRpc::XmlRpcValue robot_cfg = manga_cfg[robot_key];
  if (!robot_cfg.hasMember("targets")) {
    setState(STATE_IDLE);
    return;
  }
  const XmlRpc::XmlRpcValue targets = robot_cfg["targets"];
  if (targets.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    setState(STATE_IDLE);
    return;
  }

  uint32_t cumulative_timeline_offset = 0;
  for (int i = 0; i < targets.size(); ++i) {
    std::vector<int> leg_nodes = resolveMissionLeg(targets[i], color_sequence, nullptr, false);
    if (leg_nodes.empty()) continue;
    MissionTask task;
    task.timeline_tokens.reserve(leg_nodes.size());
    for (int t : leg_nodes) {
      if (!task.timeline_tokens.empty() && isPhysicalGraphNode(t)
          && isPhysicalGraphNode(task.timeline_tokens.back())
          && task.timeline_tokens.back() == t) {
        continue;
      }
      task.timeline_tokens.push_back(t);
      if (isPhysicalGraphNode(t)) task.nav_nodes.push_back(t);
    }
    if (task.nav_nodes.empty()) continue;
    task.plan_handler_skips_first_nav_waypoint =
        isWarehouseCoordinate(task.nav_nodes[0]) ? static_cast<uint8_t>(1) : static_cast<uint8_t>(0);
    if (task.plan_handler_skips_first_nav_waypoint && debug_verbose_) {
      ROS_INFO("custom_planner: task first nav node %d is warehouse; compensating (+1) consumed count",
               task.nav_nodes[0]);
    }
    uint32_t physical_before = 0;
    task.token_required_consumed_count.reserve(task.timeline_tokens.size());
    for (int t : task.timeline_tokens) {
      if (isPhysicalGraphNode(t)) {
        ++physical_before;
        task.token_required_consumed_count.push_back(physical_before);
      } else {
        task.token_required_consumed_count.push_back(physical_before);
      }
    }
    task.task_start_global_index = cumulative_timeline_offset;
    cumulative_timeline_offset += static_cast<uint32_t>(task.timeline_tokens.size());
    pending_tasks_.push_back(task);
  }

  if (pending_tasks_.empty()) {
    setState(STATE_IDLE);
    return;
  }

  timeline_cursor_ = 0;
  consumed_nav_nodes_count_ = 0;
  active_nav_plan_seq_ = 0;
  active_nav_plan_seq_valid_ = false;
  mission_route_published_ = false;
  if (!loadNextTaskLocked()) {
    setState(STATE_IDLE);
    return;
  }
  setState(STATE_NAVIGATING);
  processTimelineLocked();
}

std::vector<int> CustomPlannerNode::resolveMissionLeg(const XmlRpc::XmlRpcValue& leg,
                                                      const std::string& color_sequence,
                                                      std::vector<uint32_t>* leg_pause_out,
                                                      bool collapse_duplicates) const {
  std::vector<int> out;
  std::vector<std::pair<int, int>> shelf_pick_wait_pairs;
  int last_source_physical_node = -1;
  if (leg.getType() != XmlRpc::XmlRpcValue::TypeArray) return out;
  for (int i = 0; i < leg.size(); ++i) {
    const XmlRpc::XmlRpcValue& item = leg[i];
    int wait_channel = -1;
    if (parseWaitToken(item, wait_channel)) {
      out.push_back(encodeWait(wait_channel));
      continue;
    }
    int msg_channel = -1;
    if (parseMessageToken(item, msg_channel)) {
      out.push_back(encodeMsg(msg_channel));
      continue;
    }
    if (item.getType() == XmlRpc::XmlRpcValue::TypeString) {
      std::string token = static_cast<std::string>(item);
      std::string color_token;
      bool wait_pick = false;
      int wait_pick_channel = -1;
      bool use_900 = false;
      bool end_on_900 = false;
      if (parseColorWith900Suffix(token, color_token, wait_pick, wait_pick_channel, use_900, end_on_900)) {
        const int input = resolveIndexedColorNode(color_token, color_sequence);
        if (input >= 0) {
          const int approach = approachNodeForInputShelf(input);
          const int target_shelf = (last_source_physical_node == 8) ? (input + 100) : input;

          std::vector<int> base;
          base.reserve(wait_pick ? 4 : 3);
          appendWarehousePickupTraversal(input, target_shelf, base, wait_pick, wait_pick_channel);

          if (use_900) {
            const int approach900 = approach900NodeForApproach(approach);
            if (approach900 >= 0) out.push_back(approach900);
          }
          out.insert(out.end(), base.begin(), base.end());
          if (use_900 && end_on_900) {
            const int approach900 = approach900NodeForApproach(approach);
            if (approach900 >= 0) out.push_back(approach900);
          }

          if (wait_pick) {
            if (approach >= 0) shelf_pick_wait_pairs.push_back(std::make_pair(target_shelf, approach));
          }
          if (approach >= 0) last_source_physical_node = approach;
        }
        continue;
      }
      const int input = resolveIndexedColorNode(token, color_sequence);
      if (input >= 0) {
        const int approach = approachNodeForInputShelf(input);
        const int target_shelf = (last_source_physical_node == 8) ? (input + 100) : input;
        appendWarehousePickupTraversal(input, target_shelf, out, false, -1);
        if (approach >= 0) last_source_physical_node = approach;
        continue;
      }
    }
    int v = -1;
    if (tryReadInt(item, v)) {
      out.push_back(v);
      last_source_physical_node = v;
    }
  }
  if (collapse_duplicates) {
    collapseConsecutiveDuplicateNodes(out);
  }
  if (leg_pause_out && !shelf_pick_wait_pairs.empty()) {
    size_t search_from = 0;
    for (const auto& pr : shelf_pick_wait_pairs) {
      const int shelf = pr.first;
      const int ap = pr.second;
      bool found = false;
      for (size_t j = search_from; j + 2 < out.size(); ++j) {
        if (out[j] == ap && out[j + 1] == shelf && out[j + 2] == ap) {
          uint32_t cnt = 0;
          for (size_t k = 0; k < j + 1; ++k) {
            if (isPhysicalGraphNode(out[k])) ++cnt;
          }
          leg_pause_out->push_back(cnt);
          search_from = j + 3;
          found = true;
          break;
        }
      }
      if (!found) {
        ROS_WARN("custom_planner: could not locate shelf %d / approach %d triplet after collapse", shelf, ap);
      }
    }
  }
  return out;
}

int CustomPlannerNode::resolveIndexedColorNode(const std::string& token, const std::string& color_sequence) const {
  std::string u;
  u.reserve(token.size());
  for (char c : token) u.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));

  int occ = 1;
  char color = '\0';
  std::smatch m;
  if (std::regex_match(u, m, std::regex("^([1-4])([RGB])$"))) {
    occ = std::stoi(m[1]);
    color = m[2].str()[0];
  } else if (std::regex_match(u, m, std::regex("^([RGB])$"))) {
    occ = 1;
    color = m[1].str()[0];
  } else {
    return -1;
  }

  int seen = 0;
  for (size_t i = 0; i < color_sequence.size() && i < 4; ++i) {
    const char c = static_cast<char>(std::toupper(static_cast<unsigned char>(color_sequence[i])));
    if (c != color) continue;
    seen++;
    if (seen == occ) {
      auto it = color_input_node_by_index_.find(static_cast<int>(i));
      return (it == color_input_node_by_index_.end()) ? -1 : it->second;
    }
  }
  ROS_WARN("Token %s cannot be resolved with color sequence %s", token.c_str(), color_sequence.c_str());
  return -1;
}

bool CustomPlannerNode::parseColorWith900Suffix(const std::string& token, std::string& color_token_out,
                                                 bool& wait_at_pick_out, int& wait_channel_out,
                                                 bool& use_900_approach_out,
                                                 bool& end_on_900_out) const {
  // Suportados:
  //   "1B", "2G", ... (sem pause)
  //   "1B_900", "1B_900f"
  //   "1B_W_N" / "1B_W_N_900"  (novo: sempre obrigatório sufixo _N)
  wait_at_pick_out = false;
  wait_channel_out = -1;
  use_900_approach_out = false;
  end_on_900_out = false;

  std::string u;
  u.reserve(token.size());
  for (char c : token) u.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));

  auto ends_with = [&](const std::string& suf) -> bool {
    return u.size() >= suf.size() && u.compare(u.size() - suf.size(), suf.size(), suf) == 0;
  };

  // 900f variants (sem W)
  if (ends_with("_900F")) {
    use_900_approach_out = true;
    end_on_900_out = true;
    color_token_out = token.substr(0, token.size() - 5);
    return !color_token_out.empty();
  }
  // _W_N_900  (com wait + canal + aux900)
  {
    std::smatch m;
    if (std::regex_match(u, m, std::regex("^(.+)_W_([0-9]+)_900$"))) {
      wait_at_pick_out = true;
      wait_channel_out = std::stoi(m[2].str());
      use_900_approach_out = true;
      color_token_out = token.substr(0, m[1].str().size());
      return !color_token_out.empty();
    }
  }
  if (ends_with("_900")) {
    use_900_approach_out = true;
    color_token_out = token.substr(0, token.size() - 4);
    return !color_token_out.empty();
  }
  // _W_N puro (novo esquema obrigatório)
  {
    std::smatch m;
    if (std::regex_match(u, m, std::regex("^(.+)_W_([0-9]+)$"))) {
      wait_at_pick_out = true;
      wait_channel_out = std::stoi(m[2].str());
      color_token_out = token.substr(0, m[1].str().size());
      return !color_token_out.empty();
    }
  }
  // _W sem canal — rejeitado explicitamente no novo esquema
  {
    std::smatch m;
    if (std::regex_match(u, m, std::regex("^(.+)_W$"))) {
      ROS_ERROR(
          "custom_planner: token '%s' usa '_W' sem sufixo _N. É obrigatório '_W_<N>' (ex.: '3B_W_2'). Ignorado.",
          token.c_str());
      return false;
    }
  }
  // Token sem sufixo de wait/900 — deixa o caller tratar como token simples
  return false;
}

bool CustomPlannerNode::parseMessageToken(const XmlRpc::XmlRpcValue& item, int& msg_channel_out) const {
  if (item.getType() != XmlRpc::XmlRpcValue::TypeString) return false;
  std::string token = static_cast<std::string>(item);
  std::string upper;
  for (char c : token) upper.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
  if (upper.rfind("MSG_", 0) != 0) return false;
  std::string suffix = upper.substr(4);
  if (suffix == "ALL") {
    msg_channel_out = 255;
    return true;
  }
  try {
    msg_channel_out = std::stoi(suffix);
    return true;
  } catch (...) {
    return false;
  }
}

bool CustomPlannerNode::parseWaitToken(const XmlRpc::XmlRpcValue& item, int& channel_out) const {
  if (item.getType() != XmlRpc::XmlRpcValue::TypeString) return false;
  std::string s = static_cast<std::string>(item);
  std::string u;
  for (char c : s) u.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
  std::smatch m;
  if (std::regex_match(u, m, std::regex("^W(?:AIT)?_([0-9]+)$"))) {
    channel_out = std::stoi(m[1].str());
    return true;
  }
  if (u == "W" || u == "WAIT") {
    ROS_ERROR(
        "custom_planner: token '%s' sem sufixo _N. É obrigatório 'WAIT_<N>'. Ignorado (validação dispara).",
        s.c_str());
    return false;
  }
  return false;
}

bool CustomPlannerNode::tryReadInt(const XmlRpc::XmlRpcValue& item, int& value) const {
  if (item.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    value = static_cast<int>(item);
    return true;
  }
  if (item.getType() == XmlRpc::XmlRpcValue::TypeString) {
    try {
      value = std::stoi(static_cast<std::string>(item));
      return true;
    } catch (...) { return false; }
  }
  return false;
}

void CustomPlannerNode::appendWarehousePickupTraversal(int approach_source_shelf_node, int target_shelf_node,
                                                       std::vector<int>& out, bool wait_at_pick,
                                                       int wait_channel) const {
  const int approach = approachNodeForInputShelf(approach_source_shelf_node);
  if (approach < 0) {
    out.push_back(target_shelf_node);
    return;
  }
  out.push_back(approach);
  out.push_back(target_shelf_node);
  if (wait_at_pick) {
    if (wait_channel >= 0) {
      out.push_back(encodeWait(wait_channel));
    } else {
      ROS_ERROR(
          "custom_planner: _W sem canal em appendWarehousePickupTraversal (shelf=%d). "
          "Canal obrigatório; pause ignorado.",
          target_shelf_node);
    }
  }
  out.push_back(approach);
}

void CustomPlannerNode::collapseConsecutiveDuplicateNodes(std::vector<int>& nodes) const {
  std::vector<int> out;
  out.reserve(nodes.size());
  for (int x : nodes) {
    if (out.empty() || out.back() != x) out.push_back(x);
  }
  nodes.swap(out);
}

void CustomPlannerNode::publishMissionRoute(const std::vector<int>& nav_nodes) {
  if (!validatePath(nav_nodes)) {
    ROS_ERROR("custom_planner: invalid mission route.");
    setState(STATE_IDLE);
    return;
  }

  std_msgs::UInt32MultiArray pause_msg;
  nav_pause_after_wp_index_pub_.publish(pause_msg);

  std_msgs::Int32MultiArray path_msg;
  path_msg.data = nav_nodes;
  planned_paths_pub_.publish(path_msg);
}

void CustomPlannerNode::publishActiveTaskDebugLocked() {
  std_msgs::Int32MultiArray timeline_msg;
  std_msgs::Int32MultiArray nav_msg;
  std_msgs::Int32MultiArray last_exec_msg;
  std_msgs::UInt32 cursor_msg;

  if (has_active_task_) {
    timeline_msg.data = active_task_.timeline_tokens;
    nav_msg.data = active_task_.nav_nodes;
    cursor_msg.data = static_cast<uint32_t>(timeline_cursor_);
  } else {
    timeline_msg.data.clear();
    nav_msg.data.clear();
    cursor_msg.data = 0;
  }

  timeline_tokens_pub_.publish(timeline_msg);
  active_task_nav_nodes_pub_.publish(nav_msg);
  timeline_cursor_pub_.publish(cursor_msg);
  last_exec_msg.data = {last_executed_token_index_, last_executed_token_value_};
  last_executed_token_pub_.publish(last_exec_msg);
}

void CustomPlannerNode::publishMyTimelineIndexLocked() {
  std_msgs::UInt8 m;
  m.data = static_cast<uint8_t>(std::min<uint32_t>(my_timeline_index_, 255u));
  my_timeline_index_pub_.publish(m);
}

void CustomPlannerNode::recomputeTicksLocked() {
  ticks_on_channel_.clear();
  for (size_t p = 0; p < peer_timelines_.size(); ++p) {
    const PeerTimeline& pt = peer_timelines_[p];
    uint32_t obs = (p < observed_timeline_index_.size()) ? observed_timeline_index_[p] : 0;
    // Own timeline uses the local cursor directly (broadcast value may lag).
    if (static_cast<int>(p) == robot_id_) {
      obs = my_timeline_index_;
    }
    for (const auto& kv : pt.msg_positions) {
      const int ch = kv.first;
      const auto& positions = kv.second;
      // MSG_N at position K fires when observed_index > K  (cursor advanced past K).
      uint32_t fired = 0;
      for (uint32_t pos : positions) {
        if (obs > pos) ++fired;
      }
      ticks_on_channel_[ch] += fired;
    }
  }
}

void CustomPlannerNode::tryReleaseWaitingLocked() {
  if (state_ != STATE_WAITING || waiting_on_channel_ < 0) return;
  const int ch = waiting_on_channel_;
  const uint32_t needed = consumed_on_channel_[ch] + 1u;
  auto it = ticks_on_channel_.find(ch);
  const uint32_t have = (it != ticks_on_channel_.end()) ? it->second : 0u;
  if (have >= needed) {
    waiting_on_channel_ = -1;
    setState(STATE_NAVIGATING);
    ROS_INFO("custom_planner: WAIT_%d released (ticks=%u, need=%u)", ch, have, needed);
    // processTimelineLocked volta ao WAIT, incrementa consumed_on_channel_ e avança cursor.
    processTimelineLocked();
  }
}

void CustomPlannerNode::onNetworkTable(const pi_pico_driver::RadioNetworkTable::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (robot_id_ < 0 || peer_timelines_.empty()) return;
  const size_t n = std::min({msg->cp.size(), msg->timeline_index.size(),
                             peer_timelines_.size(), observed_timeline_index_.size()});
  for (size_t i = 0; i < n; ++i) {
    if (static_cast<int>(i) == robot_id_) continue;  // nosso índice é autoritativo
    const uint32_t new_idx = msg->timeline_index[i];
    const uint32_t obs = observed_timeline_index_[i];
    if (new_idx <= obs) continue;  // ignora valores retrógrados ou repetidos
    // Coherence check: cp[i] deve bater com o expected_cp da posição imediatamente anterior
    // a new_idx (último nó físico visto). Se não, aceita na mesma mas loga (pode ser atraso
    // entre atualizações de my_cp/my_np vs my_timeline_index).
    const PeerTimeline& pt = peer_timelines_[i];
    if (new_idx <= pt.timeline_tokens.size()) {
      const size_t idx_check = (new_idx == 0) ? 0 : (new_idx - 1);
      if (idx_check < pt.has_expected_cp_at_index.size() &&
          pt.has_expected_cp_at_index[idx_check]) {
        const uint8_t expected = pt.expected_cp_at_index[idx_check];
        const uint8_t actual = msg->cp[i];
        if (expected != actual) {
          ROS_WARN_THROTTLE(1.0,
                            "custom_planner: peer %zu timeline_index=%u cp=%u mas esperado=%u (idx_check=%zu)",
                            i, new_idx, actual, expected, idx_check);
          // Não rejeita — a navegação pode estar ligeiramente atrás ou à frente do cursor; só loga.
        }
      }
    }
    observed_timeline_index_[i] = new_idx;
  }
  recomputeTicksLocked();
  tryReleaseWaitingLocked();
}

void CustomPlannerNode::processTimelineLocked() {
  if (!has_active_task_) return;
  if (state_ == STATE_WAITING) return;

  auto updateMyIndex = [&]() {
    my_timeline_index_ = active_task_.task_start_global_index + static_cast<uint32_t>(timeline_cursor_);
    publishMyTimelineIndexLocked();
  };

  while (timeline_cursor_ < active_task_.timeline_tokens.size()) {
    if (timeline_cursor_ >= active_task_.token_required_consumed_count.size()) break;
    if (consumed_nav_nodes_count_ < active_task_.token_required_consumed_count[timeline_cursor_]) break;

    const int tok = active_task_.timeline_tokens[timeline_cursor_];
    last_executed_token_index_ = static_cast<int32_t>(timeline_cursor_);
    last_executed_token_value_ = static_cast<int32_t>(tok);

    if (isWaitLegToken(tok)) {
      const int ch = waitChannelOf(tok);
      recomputeTicksLocked();
      const uint32_t needed = consumed_on_channel_[ch] + 1u;
      const uint32_t have = ticks_on_channel_[ch];
      if (have < needed) {
        waiting_on_channel_ = ch;
        setState(STATE_WAITING);
        ROS_INFO("custom_planner: STATE_WAITING canal=%d (ticks=%u, need=%u)", ch, have, needed);
        publishActiveTaskDebugLocked();
        return;
      }
      consumed_on_channel_[ch] = needed;
      ROS_INFO("custom_planner: WAIT_%d liberação imediata (ticks=%u, need=%u)", ch, have, needed);
      ++timeline_cursor_;
      updateMyIndex();
      publishActiveTaskDebugLocked();
      continue;
    }
    if (isMsgLegToken(tok)) {
      ++timeline_cursor_;
      updateMyIndex();
      // Atualiza ticks locais caso sejamos produtor+waiter no mesmo canal.
      recomputeTicksLocked();
      publishActiveTaskDebugLocked();
      continue;
    }
    // Physical node (ou sentinela não tratada) — só avançamos.
    ++timeline_cursor_;
    updateMyIndex();
    publishActiveTaskDebugLocked();
  }

  if (timeline_cursor_ >= active_task_.timeline_tokens.size()) {
    if (loadNextTaskLocked()) {
      setState(STATE_NAVIGATING);
      processTimelineLocked();
      return;
    }
    has_active_task_ = false;
    last_executed_token_index_ = -1;
    last_executed_token_value_ = 0;
    publishActiveTaskDebugLocked();
    setState(STATE_IDLE);
  } else if (state_ == STATE_IDLE) {
    setState(STATE_NAVIGATING);
  }

  if (has_active_task_ && state_ == STATE_NAVIGATING && !mission_route_published_) {
    publishMissionRoute(active_task_.nav_nodes);
    mission_route_published_ = true;
  }
}

bool CustomPlannerNode::loadNextTaskLocked() {
  if (pending_tasks_.empty()) return false;
  active_task_ = pending_tasks_.front();
  pending_tasks_.pop_front();
  has_active_task_ = true;
  timeline_cursor_ = 0;
  last_executed_token_index_ = -1;
  last_executed_token_value_ = 0;
  consumed_nav_nodes_count_ = 0;
  active_nav_plan_seq_ = 0;
  active_nav_plan_seq_valid_ = false;
  mission_route_published_ = false;
  my_timeline_index_ = active_task_.task_start_global_index;
  publishMyTimelineIndexLocked();
  publishActiveTaskDebugLocked();
  return true;
}

void CustomPlannerNode::setState(MissionState new_state) {
  const MissionState prev = state_;
  state_ = new_state;
  if (prev == STATE_WAITING && new_state != STATE_WAITING) {
    std_msgs::Bool wt;
    wt.data = false;
    wt_send_pub_.publish(wt);
  }
  if (new_state == STATE_WAITING) {
    std_msgs::Bool wt;
    wt.data = true;
    wt_send_pub_.publish(wt);
  }
  std_msgs::String msg;
  msg.data = (state_ == STATE_IDLE) ? "IDLE" : (state_ == STATE_NAVIGATING ? "NAVIGATING" : "WAITING");
  mission_state_pub_.publish(msg);
}

bool CustomPlannerNode::validatePath(const std::vector<int>& path) const {
  if (path.empty()) return false;
  if (valid_node_ids_.empty()) return true;
  const std::unordered_set<int> valid(valid_node_ids_.begin(), valid_node_ids_.end());
  return std::all_of(path.begin(), path.end(), [&](int n) {
    return valid.count(n) > 0;
  });
}

bool CustomPlannerNode::tryReadDouble(const XmlRpc::XmlRpcValue& v, double& out) const {
  if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    out = static_cast<double>(v);
    return true;
  }
  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    out = static_cast<double>(static_cast<int>(v));
    return true;
  }
  if (v.getType() == XmlRpc::XmlRpcValue::TypeString) {
    try {
      out = std::stod(static_cast<std::string>(v));
      return true;
    } catch (...) { return false; }
  }
  return false;
}

void CustomPlannerNode::publishSpawnPose(const std::string& manga_key) {
  if (robot_id_ < 0 || !missions_root_.valid() || !missions_root_.hasMember(manga_key)) return;
  const XmlRpc::XmlRpcValue& manga_cfg = missions_root_[manga_key];
  if (!manga_cfg.hasMember("robot_spawns")) return;
  const XmlRpc::XmlRpcValue& spawns = manga_cfg["robot_spawns"];
  if (spawns.getType() != XmlRpc::XmlRpcValue::TypeStruct) return;
  const std::string key = "robot_" + std::to_string(robot_id_);
  if (!spawns.hasMember(key)) return;

  const XmlRpc::XmlRpcValue& entry = spawns[key];
  double x = 0.0, y = 0.0, theta = 0.0;
  if (!entry.hasMember("x") || !tryReadDouble(entry["x"], x)) return;
  if (!entry.hasMember("y") || !tryReadDouble(entry["y"], y)) return;
  if (!entry.hasMember("theta") || !tryReadDouble(entry["theta"], theta)) return;

  geometry_msgs::PoseStamped ps;
  ps.header.stamp = ros::Time::now();
  ps.header.frame_id = "map";
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.position.z = 0.0;
  const double half = 0.5 * theta;
  ps.pose.orientation.x = 0.0;
  ps.pose.orientation.y = 0.0;
  ps.pose.orientation.z = std::sin(half);
  ps.pose.orientation.w = std::cos(half);
  spawn_pose_pub_.publish(ps);
}

bool CustomPlannerNode::buildPeerTimelinesLocked(const std::string& manga_key,
                                                  const std::string& color_sequence) {
  peer_timelines_.assign(num_robots_, PeerTimeline());
  if (!missions_root_.valid() || !missions_root_.hasMember(manga_key)) return false;
  const XmlRpc::XmlRpcValue manga_cfg = missions_root_[manga_key];

  for (int r = 0; r < num_robots_; ++r) {
    PeerTimeline& pt = peer_timelines_[r];
    const std::string rkey = "robot_" + std::to_string(r);
    if (!manga_cfg.hasMember(rkey)) continue;
    const XmlRpc::XmlRpcValue robot_cfg = manga_cfg[rkey];
    if (!robot_cfg.hasMember("targets")) continue;
    const XmlRpc::XmlRpcValue targets = robot_cfg["targets"];
    if (targets.getType() != XmlRpc::XmlRpcValue::TypeArray) continue;

    std::vector<int> full_tokens;
    for (int i = 0; i < targets.size(); ++i) {
      std::vector<int> leg_nodes = resolveMissionLeg(targets[i], color_sequence, nullptr, false);
      // Colapso apenas dentro da leg (coerente com a construção de pending_tasks_ local).
      std::vector<int> leg_collapsed;
      leg_collapsed.reserve(leg_nodes.size());
      for (int t : leg_nodes) {
        if (!leg_collapsed.empty() && isPhysicalGraphNode(t)
            && isPhysicalGraphNode(leg_collapsed.back()) && leg_collapsed.back() == t) {
          continue;
        }
        leg_collapsed.push_back(t);
      }
      full_tokens.insert(full_tokens.end(), leg_collapsed.begin(), leg_collapsed.end());
    }
    pt.timeline_tokens = full_tokens;
    pt.expected_cp_at_index.assign(full_tokens.size(), 0);
    pt.has_expected_cp_at_index.assign(full_tokens.size(), 0);
    int last_phys = -1;
    for (size_t i = 0; i < full_tokens.size(); ++i) {
      int tok = full_tokens[i];
      if (isPhysicalGraphNode(tok)) last_phys = tok;
      if (last_phys >= 0) {
        pt.expected_cp_at_index[i] = static_cast<uint8_t>(std::min(last_phys, 255));
        pt.has_expected_cp_at_index[i] = 1;
      }
      if (isMsgLegToken(tok)) {
        const int ch = msgChannelOf(tok);
        if (ch >= 0 && ch < 255) {  // 255 = MSG_ALL, não contribui para ticks
          pt.msg_positions[ch].push_back(static_cast<uint32_t>(i));
        }
      }
    }
  }
  return true;
}

bool CustomPlannerNode::validateChannelsLocked() const {
  bool ok = true;
  std::map<int, uint32_t> produced_total;
  std::map<int, uint32_t> waited_total;
  std::map<int, std::vector<int>> producers;  // channel -> robot_ids
  std::map<int, std::vector<int>> waiters;    // channel -> robot_ids

  for (size_t r = 0; r < peer_timelines_.size(); ++r) {
    const PeerTimeline& pt = peer_timelines_[r];
    for (const auto& kv : pt.msg_positions) {
      produced_total[kv.first] += static_cast<uint32_t>(kv.second.size());
      producers[kv.first].push_back(static_cast<int>(r));
    }
    for (int tok : pt.timeline_tokens) {
      if (isWaitLegToken(tok)) {
        const int ch = waitChannelOf(tok);
        waited_total[ch] += 1u;
        auto& v = waiters[ch];
        if (v.empty() || v.back() != static_cast<int>(r)) v.push_back(static_cast<int>(r));
      }
    }
  }

  std::set<int> all_channels;
  for (const auto& kv : produced_total) all_channels.insert(kv.first);
  for (const auto& kv : waited_total) all_channels.insert(kv.first);

  for (int ch : all_channels) {
    const uint32_t prod = produced_total[ch];
    const uint32_t wait = waited_total[ch];
    if (wait == 0 && prod > 0) {
      ROS_WARN(
          "custom_planner: canal %d tem %u MSG produzidos mas 0 WAIT. Não é erro fatal mas verifica.",
          ch, prod);
    }
    if (wait > 0 && prod < wait) {
      ROS_ERROR(
          "custom_planner: canal %d tem %u WAIT mas apenas %u MSG produzidos → deadlock garantido.",
          ch, wait, prod);
      ok = false;
    }
    std::ostringstream oss_p;
    for (int r : producers[ch]) oss_p << r << ",";
    std::ostringstream oss_w;
    for (int r : waiters[ch]) oss_w << r << ",";
    ROS_INFO("custom_planner: canal %d produtores=[%s] waiters=[%s] prod=%u wait=%u",
             ch, oss_p.str().c_str(), oss_w.str().c_str(), prod, wait);
  }
  return ok;
}
