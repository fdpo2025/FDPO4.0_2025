#include "hardcoded_intelligent_node.h"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <unordered_set>
#include <std_msgs/Int32.h>

HardcodedIntelligentNode::HardcodedIntelligentNode(ros::NodeHandle& nh)
  : nh_(nh)
{
  color_input_node_by_index_[0] = 0;
  color_input_node_by_index_[1] = 1;
  color_input_node_by_index_[2] = 2;
  color_input_node_by_index_[3] = 3;

  loadValidNodeIds();
  loadMissions();

  color_seq_sub_ = nh_.subscribe("/color_sequence", 1, &HardcodedIntelligentNode::onColorSequence, this);
  robot_identity_sub_ = nh_.subscribe("/robot_identity", 1, &HardcodedIntelligentNode::onRobotIdentity, this);
  nav_feedback_sub_ = nh_.subscribe("/nav_completion_feedback", 20, &HardcodedIntelligentNode::onNavigationFeedback, this);
  wait_release_sub_ = nh_.subscribe("/radio_wait_release", 5, &HardcodedIntelligentNode::onWaitRelease, this);
  planned_paths_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/planned_paths", 100, true);
  mission_state_pub_ = nh_.advertise<std_msgs::String>("/hardcoded_intelligent/state", 10, true);
  radio_wait_target_pub_ = nh_.advertise<std_msgs::Int32>("/radio_wait_target", 10, false);

  setState(STATE_IDLE);
  ROS_INFO("hardcoded_intelligent initialized. Waiting for /robot_identity and /color_sequence");
}

void HardcodedIntelligentNode::loadValidNodeIds()
{
  valid_node_ids_.clear();

  XmlRpc::XmlRpcValue points_map;
  if (!nh_.getParam("points_map", points_map)) {
    ROS_WARN("points_map not found. Path validation disabled.");
    return;
  }

  for (auto it = points_map.begin(); it != points_map.end(); ++it) {
    try {
      valid_node_ids_.push_back(std::stoi(it->first));
    } catch (...) {
      ROS_WARN("Ignoring non-numeric node id in points_map: %s", it->first.c_str());
    }
  }

  std::sort(valid_node_ids_.begin(), valid_node_ids_.end());
}

bool HardcodedIntelligentNode::loadMissions()
{
  if (!nh_.getParam("missions", missions_root_)) {
    ROS_ERROR("Missing 'missions' parameter. Load config/missions.yaml in launch.");
    return false;
  }

  ROS_INFO("Missions loaded (manga_1 / manga_2 / manga_3).");
  return true;
}

std::string HardcodedIntelligentNode::selectMangaKey(const std::string& color_sequence) const
{
  bool has_r = false;
  bool has_g = false;
  bool has_b = false;
  for (char ch : color_sequence) {
    if (!std::isalpha(static_cast<unsigned char>(ch))) {
      continue;
    }
    const char u = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    if (u == 'R') {
      has_r = true;
    }
    if (u == 'G') {
      has_g = true;
    }
    if (u == 'B') {
      has_b = true;
    }
  }

  if (has_r && has_g && has_b) {
    return "manga_3";
  }
  if (has_b && has_g) {
    return "manga_2";
  }

  bool all_b = !color_sequence.empty();
  for (char ch : color_sequence) {
    if (!std::isalpha(static_cast<unsigned char>(ch))) {
      continue;
    }
    const char u = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    if (u != 'B') {
      all_b = false;
      break;
    }
  }
  if (all_b && has_b) {
    return "manga_1";
  }

  ROS_WARN("Color sequence does not match manga rules (expected all-B, B+G, or R+G+B); using manga_1.");
  return "manga_1";
}

void HardcodedIntelligentNode::onColorSequence(const std_msgs::String::ConstPtr& msg)
{
  startMission(msg->data);
}

void HardcodedIntelligentNode::onRobotIdentity(const std_msgs::Int32::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  robot_id_ = msg->data;
  ROS_INFO("Received robot identity: %d", robot_id_);
}

void HardcodedIntelligentNode::onNavigationFeedback(const plan_handler::CompletionFeedback::ConstPtr&)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (state_ != STATE_NAVIGATING || active_segment_nodes_.size() < 2) {
    return;
  }

  active_segment_feedback_count_++;
  const int required_feedback = static_cast<int>(active_segment_nodes_.size()) - 1;
  if (active_segment_feedback_count_ < required_feedback) {
    return;
  }

  advanceStateMachineAfterSegment();
}

void HardcodedIntelligentNode::onWaitRelease(const std_msgs::Bool::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (state_ != STATE_WAITING || !msg->data) {
    return;
  }
  ROS_INFO("Wait released by radio confirmation.");
  if (!pending_segments_.empty()) {
    publishCurrentSegment();
    return;
  }
  setState(STATE_IDLE);
}

void HardcodedIntelligentNode::startMission(const std::string& color_sequence)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (robot_id_ < 0) {
    ROS_WARN("Ignoring mission: robot identity not known yet.");
    return;
  }
  last_color_sequence_ = color_sequence;
  std::vector<MissionSegment> segments = buildMissionSegments(color_sequence);
  if (segments.empty()) {
    ROS_WARN("No mission segments built for robot_%d.", robot_id_);
    setState(STATE_IDLE);
    return;
  }

  pending_segments_.clear();
  for (const auto& segment : segments) {
    pending_segments_.push_back(segment);
  }

  publishCurrentSegment();
}

void HardcodedIntelligentNode::publishCurrentSegment()
{
  if (pending_segments_.empty()) {
    setState(STATE_IDLE);
    return;
  }

  const MissionSegment current = pending_segments_.front();
  pending_segments_.pop_front();

  if (!validatePath(current.nodes)) {
    ROS_ERROR("Mission segment has invalid nodes. Segment skipped.");
    setState(STATE_IDLE);
    return;
  }

  active_segment_nodes_ = current.nodes;
  active_segment_feedback_count_ = 0;

  std_msgs::Int32MultiArray path_msg;
  path_msg.data = active_segment_nodes_;
  planned_paths_pub_.publish(path_msg);
  setState(STATE_NAVIGATING);

  if (current.wait_after) {
    // Insert a sentinel empty segment with wait flag encoded as state transition trigger.
    MissionSegment wait_marker;
    wait_marker.wait_after = true;
    wait_marker.nodes = {};
    pending_segments_.push_front(wait_marker);
  }

  if (current.notify_robot_id != -2) {
    std_msgs::Int32 msg;
    msg.data = current.notify_robot_id;
    radio_wait_target_pub_.publish(msg);
    ROS_INFO("Published wait-release message target: %d", current.notify_robot_id);
  }

  ROS_INFO("Published mission segment with %zu nodes.", active_segment_nodes_.size());
}

void HardcodedIntelligentNode::advanceStateMachineAfterSegment()
{
  if (!pending_segments_.empty() && pending_segments_.front().nodes.empty() && pending_segments_.front().wait_after) {
    pending_segments_.pop_front();
    setState(STATE_WAITING);
    return;
  }

  if (!pending_segments_.empty()) {
    publishCurrentSegment();
    return;
  }

  setState(STATE_IDLE);
}

void HardcodedIntelligentNode::setState(MissionState new_state)
{
  state_ = new_state;
  std_msgs::String msg;
  if (state_ == STATE_IDLE) {
    msg.data = "IDLE";
  } else if (state_ == STATE_NAVIGATING) {
    msg.data = "NAVIGATING";
  } else {
    msg.data = "WAITING";
  }
  mission_state_pub_.publish(msg);
}

std::vector<HardcodedIntelligentNode::MissionSegment> HardcodedIntelligentNode::buildMissionSegments(const std::string& color_sequence) const
{
  std::vector<MissionSegment> result;
  if (!missions_root_.valid()) {
    return result;
  }

  const std::string manga_key = selectMangaKey(color_sequence);
  ROS_INFO("Mission manga selected: %s (color_sequence=%s)", manga_key.c_str(), color_sequence.c_str());

  if (!missions_root_.hasMember(manga_key)) {
    ROS_WARN("No '%s' block in missions file.", manga_key.c_str());
    return result;
  }

  XmlRpc::XmlRpcValue manga_cfg = missions_root_[manga_key];

  const std::string robot_key = "robot_" + std::to_string(robot_id_);
  if (!manga_cfg.hasMember(robot_key)) {
    ROS_WARN("No section '%s' under '%s' in missions file.", robot_key.c_str(), manga_key.c_str());
    return result;
  }

  XmlRpc::XmlRpcValue robot_cfg = manga_cfg[robot_key];
  if (!robot_cfg.hasMember("targets")) {
    ROS_WARN("%s/%s has no 'targets' entry.", manga_key.c_str(), robot_key.c_str());
    return result;
  }

  XmlRpc::XmlRpcValue targets = robot_cfg["targets"];
  if (targets.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_WARN("%s.%s.targets must be an array of legs.", manga_key.c_str(), robot_key.c_str());
    return result;
  }

  for (int i = 0; i < targets.size(); ++i) {
    std::vector<int> leg_nodes = resolveMissionLeg(targets[i], color_sequence, manga_cfg);
    if (leg_nodes.empty()) {
      continue;
    }

    MissionSegment segment;
    for (size_t idx = 0; idx < leg_nodes.size(); ++idx) {
      if (leg_nodes[idx] == -1) {
        if (!segment.nodes.empty()) {
          segment.wait_after = true;
          result.push_back(segment);
          segment = MissionSegment();
        }
      } else if (leg_nodes[idx] <= -1000) {
        if (!segment.nodes.empty()) {
          segment.notify_robot_id = (-1000 - leg_nodes[idx]);
          result.push_back(segment);
          segment = MissionSegment();
        } else if (!result.empty()) {
          result.back().notify_robot_id = (-1000 - leg_nodes[idx]);
        }
      } else {
        segment.nodes.push_back(leg_nodes[idx]);
      }
    }
    if (!segment.nodes.empty()) {
      result.push_back(segment);
    }
  }

  return result;
}

std::vector<int> HardcodedIntelligentNode::resolveMissionLeg(const XmlRpc::XmlRpcValue& leg, const std::string& color_sequence,
                                                             const XmlRpc::XmlRpcValue& manga_cfg) const
{
  std::vector<int> resolved;
  if (leg.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return resolved;
  }

  std::map<char, int> local_color_claim_counter;
  for (int i = 0; i < leg.size(); ++i) {
    const XmlRpc::XmlRpcValue& item = leg[i];

    if (isWaitToken(item)) {
      resolved.push_back(-1);
      continue;
    }

    int msg_target = -2;
    if (parseMessageToken(item, msg_target)) {
      resolved.push_back(-1000 - msg_target);
      continue;
    }

    if (item.getType() == XmlRpc::XmlRpcValue::TypeString) {
      std::string token = static_cast<std::string>(item);
      char color_letter = ' ';
      bool wait_at_pick = false;
      if (parseColorWithWaitSuffix(token, color_letter, wait_at_pick)) {
        const int input_node = resolveColorToInputNode(color_letter, color_sequence, local_color_claim_counter, manga_cfg);
        if (input_node >= 0) {
          appendWarehousePickupTraversal(input_node, resolved, wait_at_pick);
        }
        continue;
      }
      if (token.size() == 1) {
        const char c = static_cast<char>(std::toupper(static_cast<unsigned char>(token[0])));
        if (c == 'R' || c == 'G' || c == 'B') {
          const int input_node = resolveColorToInputNode(c, color_sequence, local_color_claim_counter, manga_cfg);
          if (input_node >= 0) {
            appendWarehousePickupTraversal(input_node, resolved, false);
          }
          continue;
        }
      }
    }

    int node_value = -1;
    if (tryReadInt(item, node_value)) {
      resolved.push_back(node_value);
    }
  }

  collapseConsecutiveDuplicateNodes(resolved);
  return resolved;
}

int HardcodedIntelligentNode::resolveColorToInputNode(char color, const std::string& color_sequence, std::map<char, int>& local_color_claim_counter,
                                                      const XmlRpc::XmlRpcValue& manga_cfg) const
{
  const int base_rank = getColorClaimBaseRank(color, manga_cfg);
  const int local_rank = local_color_claim_counter[color];
  const int desired_occurrence = base_rank + local_rank;

  int seen = 0;
  for (size_t i = 0; i < color_sequence.size() && i < 4; ++i) {
    char c = static_cast<char>(std::toupper(color_sequence[i]));
    if (c != color) {
      continue;
    }
    if (seen != desired_occurrence) {
      seen++;
      continue;
    }
    auto it = color_input_node_by_index_.find(static_cast<int>(i));
    if (it != color_input_node_by_index_.end()) {
      local_color_claim_counter[color] = local_rank + 1;
      return it->second;
    }
  }
  ROS_WARN("No available input node for color %c with sequence '%s'.", color, color_sequence.c_str());
  return -1;
}

int HardcodedIntelligentNode::getColorClaimBaseRank(char color, const XmlRpc::XmlRpcValue& manga_cfg) const
{
  if (manga_cfg.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    return 0;
  }
  int rank = 0;
  for (auto it = manga_cfg.begin(); it != manga_cfg.end(); ++it) {
    const std::string robot_key = it->first;
    if (robot_key.rfind("robot_", 0) != 0) {
      continue;
    }
    int other_id = -1;
    try {
      other_id = std::stoi(robot_key.substr(6));
    } catch (...) {
      continue;
    }
    if (other_id >= robot_id_) {
      continue;
    }
    if (robotUsesColor(it->second, color)) {
      rank++;
    }
  }
  return rank;
}

bool HardcodedIntelligentNode::robotUsesColor(const XmlRpc::XmlRpcValue& robot_cfg, char color) const
{
  if (!robot_cfg.hasMember("targets")) {
    return false;
  }
  XmlRpc::XmlRpcValue targets = robot_cfg["targets"];
  if (targets.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return false;
  }
  for (int i = 0; i < targets.size(); ++i) {
    const XmlRpc::XmlRpcValue& leg = targets[i];
    if (leg.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      continue;
    }
    for (int j = 0; j < leg.size(); ++j) {
      const XmlRpc::XmlRpcValue& item = leg[j];
      if (item.getType() != XmlRpc::XmlRpcValue::TypeString) {
        continue;
      }
      std::string token = static_cast<std::string>(item);
      if (token.size() == 1 && static_cast<char>(std::toupper(static_cast<unsigned char>(token[0]))) == color) {
        return true;
      }
      char cw = ' ';
      bool dummy_wait = false;
      if (parseColorWithWaitSuffix(token, cw, dummy_wait) && cw == color) {
        return true;
      }
    }
  }
  return false;
}

bool HardcodedIntelligentNode::parseColorWithWaitSuffix(const std::string& token, char& color_out, bool& wait_at_pick_out) const
{
  const size_t pos = token.find('_');
  if (pos == std::string::npos || pos == 0) {
    return false;
  }
  const std::string left = token.substr(0, pos);
  std::string right = token.substr(pos + 1);
  for (char& ch : right) {
    ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
  }
  if (right != "W") {
    return false;
  }
  if (left.size() != 1) {
    return false;
  }
  const char c = static_cast<char>(std::toupper(static_cast<unsigned char>(left[0])));
  if (c != 'R' && c != 'G' && c != 'B') {
    return false;
  }
  color_out = c;
  wait_at_pick_out = true;
  return true;
}

bool HardcodedIntelligentNode::parseMessageToken(const XmlRpc::XmlRpcValue& item, int& target_robot_id) const
{
  if (item.getType() != XmlRpc::XmlRpcValue::TypeString) {
    return false;
  }
  std::string token = static_cast<std::string>(item);
  std::string upper = token;
  for (char& ch : upper) {
    ch = static_cast<char>(std::toupper(ch));
  }
  const std::string prefix = "MSG_";
  if (upper.rfind(prefix, 0) != 0) {
    return false;
  }
  std::string suffix = upper.substr(prefix.size());
  if (suffix == "ALL") {
    target_robot_id = -1;
    return true;
  }
  try {
    target_robot_id = std::stoi(suffix);
    return true;
  } catch (...) {
    return false;
  }
}

bool HardcodedIntelligentNode::isWaitToken(const XmlRpc::XmlRpcValue& item) const
{
  if (item.getType() != XmlRpc::XmlRpcValue::TypeString) {
    return false;
  }
  const std::string token = static_cast<std::string>(item);
  return token == "W" || token == "w" || token == "WAIT" || token == "wait";
}

bool HardcodedIntelligentNode::tryReadInt(const XmlRpc::XmlRpcValue& item, int& value) const
{
  if (item.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    value = static_cast<int>(item);
    return true;
  }

  if (item.getType() == XmlRpc::XmlRpcValue::TypeString) {
    std::string token = static_cast<std::string>(item);
    try {
      value = std::stoi(token);
      return true;
    } catch (...) {
      return false;
    }
  }

  return false;
}

void HardcodedIntelligentNode::appendWarehousePickupTraversal(int input_shelf_node, std::vector<int>& out, bool wait_at_pick) const
{
  // Armazém de entrada: slot 0..3; aproximação em linha: 8..11 (cf. factory graph).
  int approach = -1;
  switch (input_shelf_node) {
    case 0:
      approach = 8;
      break;
    case 1:
      approach = 9;
      break;
    case 2:
      approach = 10;
      break;
    case 3:
      approach = 11;
      break;
    default:
      ROS_WARN("appendWarehousePickupTraversal: node %d is not an input shelf (0-3); using as single waypoint.", input_shelf_node);
      out.push_back(input_shelf_node);
      return;
  }
  out.push_back(approach);
  out.push_back(input_shelf_node);
  if (wait_at_pick) {
    out.push_back(-1);
  }
  out.push_back(approach);
}

void HardcodedIntelligentNode::collapseConsecutiveDuplicateNodes(std::vector<int>& nodes) const
{
  if (nodes.empty()) {
    return;
  }
  std::vector<int> out;
  out.reserve(nodes.size());
  for (int x : nodes) {
    if (out.empty() || out.back() != x) {
      out.push_back(x);
    }
  }
  nodes.swap(out);
}

bool HardcodedIntelligentNode::validatePath(const std::vector<int>& path) const
{
  if (path.empty()) {
    return false;
  }

  if (valid_node_ids_.empty()) {
    return true;
  }

  const std::unordered_set<int> valid(valid_node_ids_.begin(), valid_node_ids_.end());
  return std::all_of(path.begin(), path.end(), [&](int node_id) {
    return valid.count(node_id) > 0;
  });
}
