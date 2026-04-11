#include "custom_planner_node.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <regex>
#include <unordered_set>

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
  if (num_robots_ > 3) num_robots_ = 3;

  color_input_node_by_index_[0] = 0;
  color_input_node_by_index_[1] = 1;
  color_input_node_by_index_[2] = 2;
  color_input_node_by_index_[3] = 3;

  loadValidNodeIds();
  loadMissions();

  nh_.param<std::string>("color_sequence_topic", color_sequence_topic_, "/color_sequence");
  nh_.param<std::string>("wait_state_topic", wait_state_topic_, "/pi_pico_driver/wait_state");
  nh_.param<std::string>("this_current_pose_topic", this_current_pose_topic_, "/this_current_pose");

  mission_color_sub_ = nh_.subscribe(color_sequence_topic_, 1, &CustomPlannerNode::onMissionColorSequence, this);
  robot_identity_sub_ = nh_.subscribe("/robot_identity", 1, &CustomPlannerNode::onRobotIdentity, this);
  wait_state_sub_ = nh_.subscribe(wait_state_topic_, 20, &CustomPlannerNode::onWaitState, this);
  this_pose_sub_ = nh_.subscribe(this_current_pose_topic_, 50, &CustomPlannerNode::onThisCurrentPose, this);

  planned_paths_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/planned_paths", 100, true);
  mission_state_pub_ = nh_.advertise<std_msgs::String>("/custom_planner/state", 10, true);
  radio_wait_target_pub_ = nh_.advertise<std_msgs::Int32>("/radio_wait_target", 10, false);
  spawn_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/custom_planner/spawn_pose", 1, true);

  setState(STATE_IDLE);

  int initial_robot_id = -1;
  nh_.param("initial_robot_id", initial_robot_id, -1);
  if (initial_robot_id >= 0) {
    applyRobotIdentity(initial_robot_id);
  }

  ROS_INFO(
      "custom_planner ready: identity via /robot_identity (pi_pico_driver após INIT) ou param "
      "initial_robot_id; topics %s, %s, %s (num_robots=%d)",
      color_sequence_topic_.c_str(), wait_state_topic_.c_str(), this_current_pose_topic_.c_str(),
      num_robots_);
}

bool CustomPlannerNode::loadMissions() {
  const std::string missions_key = (num_robots_ == 3) ? "missions_3" : "missions_2";
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

void CustomPlannerNode::onWaitState(const std_msgs::Bool::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  // wait_state=true: robot is waiting (frozen). false: running — resume YAML sequence.
  if (state_ != STATE_WAITING || msg->data) return;
  if (!pending_segments_.empty()) {
    publishCurrentSegment();
    return;
  }
  setState(STATE_IDLE);
}

void CustomPlannerNode::onThisCurrentPose(const std_msgs::UInt32::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (state_ != STATE_NAVIGATING || active_segment_nodes_.empty()) return;
  const int last_node = active_segment_nodes_.back();
  if (static_cast<int>(msg->data) != last_node) return;
  ROS_INFO_THROTTLE(1.0, "custom_planner: segment done (this_current_pose=%u == last drop node %d)",
                    static_cast<unsigned>(msg->data), last_node);
  advanceStateMachineAfterSegment();
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
  std::vector<MissionSegment> segments = buildMissionSegments(color_sequence, manga_key);
  pending_segments_.clear();
  for (const auto& s : segments) pending_segments_.push_back(s);
  if (pending_segments_.empty()) {
    setState(STATE_IDLE);
    return;
  }
  publishCurrentSegment();
}

std::vector<CustomPlannerNode::MissionSegment> CustomPlannerNode::buildMissionSegments(
    const std::string& color_sequence, const std::string& manga_key) const {
  std::vector<MissionSegment> out;
  if (!missions_root_.valid() || !missions_root_.hasMember(manga_key)) return out;
  const XmlRpc::XmlRpcValue manga_cfg = missions_root_[manga_key];
  const std::string robot_key = "robot_" + std::to_string(robot_id_);
  if (!manga_cfg.hasMember(robot_key)) return out;
  const XmlRpc::XmlRpcValue robot_cfg = manga_cfg[robot_key];
  if (!robot_cfg.hasMember("targets")) return out;
  const XmlRpc::XmlRpcValue targets = robot_cfg["targets"];
  if (targets.getType() != XmlRpc::XmlRpcValue::TypeArray) return out;

  for (int i = 0; i < targets.size(); ++i) {
    std::vector<int> leg_nodes = resolveMissionLeg(targets[i], color_sequence);
    if (leg_nodes.empty()) continue;
    MissionSegment seg;
    for (int v : leg_nodes) {
      if (v == -1) {
        if (!seg.nodes.empty()) {
          seg.wait_after = true;
          out.push_back(seg);
          seg = MissionSegment();
        } else {
          // Leading "W" at start of a leg (e.g. robot_1 manga_1): wait before any path.
          // Previously this -1 was dropped because seg was empty, so both robots moved together.
          MissionSegment wait_before_move;
          wait_before_move.wait_after = true;
          out.push_back(wait_before_move);
        }
      } else if (v <= -1000) {
        if (!seg.nodes.empty()) {
          seg.notify_robot_id = -1000 - v;
          out.push_back(seg);
          seg = MissionSegment();
        } else if (!out.empty()) {
          out.back().notify_robot_id = -1000 - v;
        }
      } else {
        seg.nodes.push_back(v);
      }
    }
    if (!seg.nodes.empty()) out.push_back(seg);
  }
  return out;
}

std::vector<int> CustomPlannerNode::resolveMissionLeg(const XmlRpc::XmlRpcValue& leg,
                                                      const std::string& color_sequence) const {
  std::vector<int> out;
  if (leg.getType() != XmlRpc::XmlRpcValue::TypeArray) return out;
  for (int i = 0; i < leg.size(); ++i) {
    const XmlRpc::XmlRpcValue& item = leg[i];
    if (isWaitToken(item)) {
      out.push_back(-1);
      continue;
    }
    int msg_target = -2;
    if (parseMessageToken(item, msg_target)) {
      out.push_back(-1000 - msg_target);
      continue;
    }
    if (item.getType() == XmlRpc::XmlRpcValue::TypeString) {
      std::string token = static_cast<std::string>(item);
      std::string color_token;
      bool wait_pick = false;
      if (parseColorWithWaitSuffix(token, color_token, wait_pick)) {
        const int input = resolveIndexedColorNode(color_token, color_sequence);
        if (input >= 0) appendWarehousePickupTraversal(input, out, wait_pick);
        continue;
      }
      const int input = resolveIndexedColorNode(token, color_sequence);
      if (input >= 0) {
        appendWarehousePickupTraversal(input, out, false);
        continue;
      }
    }
    int v = -1;
    if (tryReadInt(item, v)) out.push_back(v);
  }
  collapseConsecutiveDuplicateNodes(out);
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

bool CustomPlannerNode::parseColorWithWaitSuffix(const std::string& token, std::string& color_token_out,
                                                 bool& wait_at_pick_out) const {
  const size_t p = token.find('_');
  if (p == std::string::npos || p == 0) return false;
  std::string left = token.substr(0, p);
  std::string right = token.substr(p + 1);
  for (char& c : right) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  if (right != "W") return false;
  color_token_out = left;
  wait_at_pick_out = true;
  return true;
}

bool CustomPlannerNode::parseMessageToken(const XmlRpc::XmlRpcValue& item, int& target_robot_id) const {
  if (item.getType() != XmlRpc::XmlRpcValue::TypeString) return false;
  std::string token = static_cast<std::string>(item);
  std::string upper;
  for (char c : token) upper.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
  if (upper.rfind("MSG_", 0) != 0) return false;
  std::string suffix = upper.substr(4);
  if (suffix == "ALL") {
    target_robot_id = 255;
    return true;
  }
  try {
    target_robot_id = std::stoi(suffix);
    return true;
  } catch (...) {
    return false;
  }
}

bool CustomPlannerNode::isWaitToken(const XmlRpc::XmlRpcValue& item) const {
  if (item.getType() != XmlRpc::XmlRpcValue::TypeString) return false;
  const std::string s = static_cast<std::string>(item);
  return s == "W" || s == "w" || s == "WAIT" || s == "wait";
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

void CustomPlannerNode::appendWarehousePickupTraversal(int input_shelf_node, std::vector<int>& out,
                                                       bool wait_at_pick) const {
  int approach = -1;
  switch (input_shelf_node) {
    case 0: approach = 8; break;
    case 1: approach = 9; break;
    case 2: approach = 10; break;
    case 3: approach = 11; break;
    default:
      out.push_back(input_shelf_node);
      return;
  }
  out.push_back(approach);
  out.push_back(input_shelf_node);
  if (wait_at_pick) out.push_back(-1);
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

void CustomPlannerNode::publishCurrentSegment() {
  if (pending_segments_.empty()) {
    setState(STATE_IDLE);
    return;
  }
  MissionSegment current = pending_segments_.front();
  pending_segments_.pop_front();
  if (current.nodes.empty() && current.wait_after) {
    setState(STATE_WAITING);
    publishRadioWakeRequest(robot_id_);
    return;
  }
  if (!validatePath(current.nodes)) {
    ROS_ERROR("custom_planner: invalid segment path.");
    setState(STATE_IDLE);
    return;
  }
  active_segment_nodes_ = current.nodes;
  std_msgs::Int32MultiArray path_msg;
  path_msg.data.clear();
  if (current.notify_robot_id != -2) {
    path_msg.data.push_back(-900000000 - current.notify_robot_id);
  }
  path_msg.data.insert(path_msg.data.end(), active_segment_nodes_.begin(), active_segment_nodes_.end());
  planned_paths_pub_.publish(path_msg);
  setState(STATE_NAVIGATING);

  if (current.wait_after) {
    MissionSegment wait_marker;
    wait_marker.wait_after = true;
    pending_segments_.push_front(wait_marker);
  }
  if (current.notify_robot_id != -2) {
    publishRadioWakeRequest(current.notify_robot_id);
  }
}

void CustomPlannerNode::advanceStateMachineAfterSegment() {
  if (!pending_segments_.empty() && pending_segments_.front().nodes.empty() && pending_segments_.front().wait_after) {
    pending_segments_.pop_front();
    setState(STATE_WAITING);
    publishRadioWakeRequest(robot_id_);
    return;
  }
  if (!pending_segments_.empty()) {
    publishCurrentSegment();
    return;
  }
  setState(STATE_IDLE);
}

void CustomPlannerNode::setState(MissionState new_state) {
  state_ = new_state;
  std_msgs::String msg;
  msg.data = (state_ == STATE_IDLE) ? "IDLE" : (state_ == STATE_NAVIGATING ? "NAVIGATING" : "WAITING");
  mission_state_pub_.publish(msg);
}

bool CustomPlannerNode::validatePath(const std::vector<int>& path) const {
  if (path.empty()) return false;
  if (valid_node_ids_.empty()) return true;
  const std::unordered_set<int> valid(valid_node_ids_.begin(), valid_node_ids_.end());
  auto is_msg_sentinel = [](int n) {
    return n <= -900000000 && n >= -900000000 - 255;
  };
  return std::all_of(path.begin(), path.end(), [&](int n) {
    if (is_msg_sentinel(n)) return true;
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

void CustomPlannerNode::publishRadioWakeRequest(int wait_topic_value) {
  std_msgs::Int32 msg;
  msg.data = wait_topic_value;
  radio_wait_target_pub_.publish(msg);
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

