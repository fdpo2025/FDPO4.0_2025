#include "custom_planner_node.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <regex>
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

/** Mission leg tokens from YAML: MSG_* encoded as -1000 - robot_id (see parseMessageToken). */
bool isMsgLegToken(int v) { return v <= -1000 && v >= -1000 - 255; }

/** Graph node id in expanded leg (excludes W=-1 and MSG tokens). Matches /planned_paths body indices. */
bool isPhysicalGraphNode(int v) { return v != -1 && !isMsgLegToken(v); }

/** 0-based index of out[out_idx] among physical nodes only (NavPlan / planned_paths order). */
uint32_t physicalNodeIndexAtOutIndex(const std::vector<int>& out, size_t out_idx) {
  uint32_t cnt = 0;
  for (size_t i = 0; i < out_idx && i < out.size(); ++i) {
    if (isPhysicalGraphNode(out[i])) ++cnt;
  }
  return cnt;
}
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
  nh_.param<std::string>("wait_state_topic", wait_state_topic_, "/pi_pico_driver/wait_state");
  nh_.param<std::string>("peer_wait_release_topic", peer_wait_release_topic_, "/pi_pico_driver/peer_wait_release");
  nh_.param<std::string>("this_current_pose_topic", this_current_pose_topic_, "/this_current_pose");
  nh_.param("stop_waiting_max_hold_s", stop_waiting_max_hold_s_, 1.0);
  if (stop_waiting_max_hold_s_ < 0.05) stop_waiting_max_hold_s_ = 0.05;
  nh_.param<std::string>("network_table_topic", network_table_topic_, "/pi_pico_driver/network_table");
  nh_.param<std::string>("stop_wait_seq_topic", stop_wait_seq_topic_, "/stop_wait_seq_send");

  mission_color_sub_ = nh_.subscribe(color_sequence_topic_, 1, &CustomPlannerNode::onMissionColorSequence, this);
  robot_identity_sub_ = nh_.subscribe("/robot_identity", 1, &CustomPlannerNode::onRobotIdentity, this);
  wait_state_sub_ = nh_.subscribe(wait_state_topic_, 20, &CustomPlannerNode::onWaitState, this);
  peer_wait_release_sub_ =
      nh_.subscribe(peer_wait_release_topic_, 50, &CustomPlannerNode::onPeerWaitRelease, this);
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
  radio_wait_target_pub_ = nh_.advertise<std_msgs::Int32>("/radio_wait_target", 10, false);
  wt_send_pub_ = nh_.advertise<std_msgs::Bool>("/wt_send", 10, false);
  target_id_send_pub_ = nh_.advertise<std_msgs::UInt32>("/target_id_send", 10, false);
  stop_waiting_send_pub_ = nh_.advertise<std_msgs::Bool>("/stop_waiting_send", 10, false);
  stop_wait_seq_pub_ = nh_.advertise<std_msgs::UInt8>(stop_wait_seq_topic_, 10, false);
  timeline_tokens_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/custom_planner/timeline_tokens", 1, true);
  timeline_cursor_pub_ = nh_.advertise<std_msgs::UInt32>("/custom_planner/timeline_cursor", 10, true);
  active_task_nav_nodes_pub_ =
      nh_.advertise<std_msgs::Int32MultiArray>("/custom_planner/active_task_nav_nodes", 1, true);
  last_executed_token_pub_ =
      nh_.advertise<std_msgs::Int32MultiArray>("/custom_planner/last_executed_token", 10, true);
  spawn_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/custom_planner/spawn_pose", 1, true);

  setState(STATE_IDLE);

  int initial_robot_id = -1;
  nh_.param("initial_robot_id", initial_robot_id, -1);
  if (initial_robot_id >= 0) {
    applyRobotIdentity(initial_robot_id);
  }

  ROS_INFO(
      "custom_planner ready: identity via /robot_identity (pi_pico_driver após INIT) ou param "
      "initial_robot_id; topics %s, %s, %s, %s (num_robots=%d); stop_waiting max_hold=%.3fs net=%s seq=%s",
      color_sequence_topic_.c_str(), wait_state_topic_.c_str(), this_current_pose_topic_.c_str(),
      peer_wait_release_topic_.c_str(), num_robots_, stop_waiting_max_hold_s_,
      network_table_topic_.c_str(), stop_wait_seq_topic_.c_str());
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

void CustomPlannerNode::onPeerWaitRelease(const std_msgs::Empty::ConstPtr& msg) {
  (void)msg;
  std::lock_guard<std::mutex> lock(mtx_);
  // Defensive: if onWaitState fired the falling edge BEFORE the matching peer_wait_release
  // (callback-ordering anomaly), an absorb window was armed there. Drop this lagging release so
  // it does not become a ghost on a future -1.
  if (extra_peer_release_drops_ > 0 && ros::Time::now() < extra_peer_release_drops_until_) {
    --extra_peer_release_drops_;
    ROS_INFO_THROTTLE(0.5,
                      "custom_planner: peer_wait_release absorbed (post-edge window, drops left=%d)",
                      extra_peer_release_drops_);
    return;
  }
  if (extra_peer_release_drops_ > 0 && ros::Time::now() >= extra_peer_release_drops_until_) {
    extra_peer_release_drops_ = 0;
  }
  // Always buffer. The matching wait_state falling edge will pop_front (current WAIT). Any
  // remaining entries are legitimate early releases for future -1 tokens.
  pending_peer_releases_.push_back(1u);
  ROS_INFO_THROTTLE(0.5, "custom_planner: buffered peer_wait_release (FIFO size=%zu, state=%s)",
                    pending_peer_releases_.size(),
                    state_ == STATE_WAITING ? "WAITING"
                        : (state_ == STATE_NAVIGATING ? "NAVIGATING" : "IDLE"));
}

void CustomPlannerNode::onWaitState(const std_msgs::Bool::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  // wait_state mirrors network "waiting" for this robot_id (CMD wt_send + radio).
  if (state_ == STATE_WAITING && wait_state_resync_armed_) {
    last_pi_wait_state_ = msg->data;
    wait_state_resync_armed_ = false;
    // Pico-side race: peer's stop_waiting reached the Pico BEFORE our wt=1 did, so the Pico's
    // is_waiting was forced to 0 even at the very first publish after we entered WAITING. There
    // is no future falling edge possible (baseline is already 0). If a peer_wait_release was
    // already buffered, consume it now and exit WAITING immediately.
    if (!msg->data && !pending_peer_releases_.empty()) {
      pending_peer_releases_.pop_front();
      setState(STATE_NAVIGATING);
      ROS_INFO("custom_planner: race-released at WAIT entry (Pico flag pre-set); FIFO=%zu",
               pending_peer_releases_.size());
      processTimelineLocked();
    }
    return;
  }
  const bool prev = last_pi_wait_state_;
  last_pi_wait_state_ = msg->data;
  if (state_ != STATE_WAITING) return;
  // Require falling edge: was waiting on the wire, peer cleared us via stop_waiting.
  if (!prev || msg->data) return;
  setState(STATE_NAVIGATING);
  // Consume the release that caused this edge. Any remaining FIFO entries are early releases
  // for future -1 tokens. If FIFO is empty, the edge fired before its matching peer_wait_release
  // arrived (callback-order anomaly) — arm an absorb window to drop the lagging duplicate.
  if (!pending_peer_releases_.empty()) {
    pending_peer_releases_.pop_front();
    ROS_INFO("custom_planner: wait_state edge consumed peer_wait_release; FIFO=%zu",
             pending_peer_releases_.size());
  } else {
    extra_peer_release_drops_ = 1;
    extra_peer_release_drops_until_ = ros::Time::now() + ros::Duration(0.30);
    ROS_WARN("custom_planner: wait_state edge with empty FIFO; absorb window armed (300 ms)");
  }
  processTimelineLocked();
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

  // plan_handler drops the first planned_paths node from NavPlan if it is a warehouse; NavPlan
  // plan_index advances one step behind mission physical-node count for that omission.
  const uint32_t consumed_count =
      plan_index + 1u + static_cast<uint32_t>(active_task_.plan_handler_skips_first_nav_waypoint);
  if (consumed_count <= consumed_nav_nodes_count_) return;
  consumed_nav_nodes_count_ = consumed_count;
  processTimelineLocked();
}

void CustomPlannerNode::startMission(const std::string& color_sequence) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (stop_waiting_pulse_active_) {
    endStopWaitingPulseLocked();
  }
  if (robot_id_ < 0) {
    pending_color_sequence_ = color_sequence;
    ROS_WARN("custom_planner: robot identity unknown, mission queued.");
    return;
  }
  last_color_sequence_ = color_sequence;
  const std::string manga_key = selectMangaKey(color_sequence);
  publishSpawnPose(manga_key);
  pending_peer_releases_.clear();
  extra_peer_release_drops_ = 0;
  // Cancel any in-flight stop_waiting pulse from the previous mission so the new mission can
  // immediately fire MSG_* tokens.
  if (stop_waiting_reset_timer_) {
    stop_waiting_reset_timer_.stop();
  }
  if (stop_waiting_pulse_active_) {
    std_msgs::Bool off;
    off.data = false;
    stop_waiting_send_pub_.publish(off);
    stop_waiting_pulse_active_ = false;
  }
  pending_tasks_.clear();
  has_active_task_ = false;
  active_task_ = MissionTask();
  last_executed_token_index_ = -1;
  last_executed_token_value_ = 0;
  publishActiveTaskDebugLocked();

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

  for (int i = 0; i < targets.size(); ++i) {
    std::vector<int> leg_nodes = resolveMissionLeg(targets[i], color_sequence, nullptr, false);
    if (leg_nodes.empty()) continue;
    MissionTask task;
    task.timeline_tokens.reserve(leg_nodes.size());
    for (int t : leg_nodes) {
      // Normalize only consecutive duplicated physical nodes in mission source.
      // Keep MSG/W semantics untouched and preserve duplicates separated by events.
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
      ROS_INFO("custom_planner: task first nav node %d is warehouse; compensating nav_plan consumed count (+1)",
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
    std::vector<uint32_t> leg_pause;
    std::vector<int> leg_nodes = resolveMissionLeg(targets[i], color_sequence, &leg_pause);
    if (leg_nodes.empty()) continue;
    // Only drop _W metadata if there is a mid-leg "W" (-1) after some graph nodes (ambiguous vs shelf pause).
    // Leading W (-1 before any physical node) does not invalidate shelf indices.
    if (!leg_pause.empty()) {
      bool seen_physical = false;
      bool mid_leg_wait = false;
      for (int v : leg_nodes) {
        if (v == -1) {
          if (seen_physical) mid_leg_wait = true;
        } else if (isPhysicalGraphNode(v) || isMsgLegToken(v)) {
          if (isPhysicalGraphNode(v)) seen_physical = true;
        }
      }
      if (mid_leg_wait) {
        ROS_WARN(
            "custom_planner: leg has pause_after_wp_index and mid-leg W (-1); dropping pause metadata for safety");
        leg_pause.clear();
      }
    }
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
    if (!seg.nodes.empty()) {
      seg.pause_after_wp_index = std::move(leg_pause);
      out.push_back(seg);
    }
  }
  return out;
}

std::vector<int> CustomPlannerNode::resolveMissionLeg(const XmlRpc::XmlRpcValue& leg,
                                                      const std::string& color_sequence,
                                                      std::vector<uint32_t>* leg_pause_out,
                                                      bool collapse_duplicates) const {
  std::vector<int> out;
  std::vector<std::pair<int, int>> shelf_pick_wait_pairs;
  // Context from original mission leg (before token expansion). Used for +100 mirror selection.
  int last_source_physical_node = -1;
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
      bool use_900 = false;
      bool end_on_900 = false;
      if (parseColorWith900Suffix(token, color_token, wait_pick, use_900, end_on_900)) {
        const int input = resolveIndexedColorNode(color_token, color_sequence);
        if (input >= 0) {
          const int approach = approachNodeForInputShelf(input);
          const int target_shelf = (last_source_physical_node == 8) ? (input + 100) : input;

          // Build the legacy traversal first: approach -> shelf -> (W) -> approach
          std::vector<int> base;
          base.reserve(wait_pick ? 4 : 3);
          appendWarehousePickupTraversal(input, target_shelf, base, wait_pick);

          // Then, if requested, add the +900 auxiliary approach before (and optionally after).
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
          // Mirror selection (+100) for any subsequent token must consider the LAST physical node
          // we appended (the approach we leave on) — not the integer literal that preceded the
          // color token. This affects e.g. ["8", "1B", "2G", ...] sequences.
          if (approach >= 0) last_source_physical_node = approach;
        }
        continue;
      }
      const int input = resolveIndexedColorNode(token, color_sequence);
      if (input >= 0) {
        const int approach = approachNodeForInputShelf(input);
        const int target_shelf = (last_source_physical_node == 8) ? (input + 100) : input;
        appendWarehousePickupTraversal(input, target_shelf, out, false);
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
          const uint32_t pause_idx = physicalNodeIndexAtOutIndex(out, j + 1);
          leg_pause_out->push_back(pause_idx);
          ROS_INFO(
              "custom_planner: pause_after_wp_index=%u (human=%u) at shelf node %d after approach %d "
              "(physical path index, not raw leg token index)",
              static_cast<unsigned>(pause_idx), static_cast<unsigned>(pause_idx + 1), shelf, ap);
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

bool CustomPlannerNode::parseColorWith900Suffix(const std::string& token, std::string& color_token_out,
                                                bool& wait_at_pick_out,
                                                bool& use_900_approach_out,
                                                bool& end_on_900_out) const {
  // Supported:
  // - "1B_900"
  // - "1B_W_900"
  // - "1B_900f" (no _W variant)
  // - "1B_W" (legacy; use_900=false)
  // - "1B" (not handled here)
  wait_at_pick_out = false;
  use_900_approach_out = false;
  end_on_900_out = false;

  std::string u;
  u.reserve(token.size());
  for (char c : token) u.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));

  auto ends_with = [&](const std::string& suf) -> bool {
    return u.size() >= suf.size() && u.compare(u.size() - suf.size(), suf.size(), suf) == 0;
  };

  if (ends_with("_W_900F")) {
    // Explicitly not supported per requirement: no wait variants for 900f.
    return false;
  }
  if (ends_with("_900F")) {
    use_900_approach_out = true;
    end_on_900_out = true;
    color_token_out = token.substr(0, token.size() - 5);  // strip "_900f"
    return !color_token_out.empty();
  }
  if (ends_with("_W_900")) {
    use_900_approach_out = true;
    wait_at_pick_out = true;
    color_token_out = token.substr(0, token.size() - 6);  // strip "_W_900"
    return !color_token_out.empty();
  }
  if (ends_with("_900")) {
    use_900_approach_out = true;
    color_token_out = token.substr(0, token.size() - 4);  // strip "_900"
    return !color_token_out.empty();
  }
  // Fall back to legacy "_W"
  if (parseColorWithWaitSuffix(token, color_token_out, wait_at_pick_out)) {
    use_900_approach_out = false;
    return true;
  }
  return false;
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

void CustomPlannerNode::appendWarehousePickupTraversal(int approach_source_shelf_node, int target_shelf_node,
                                                       std::vector<int>& out, bool wait_at_pick) const {
  const int approach = approachNodeForInputShelf(approach_source_shelf_node);
  if (approach < 0) {
    out.push_back(target_shelf_node);
    return;
  }
  out.push_back(approach);
  out.push_back(target_shelf_node);
  if (wait_at_pick) {
    // _W semantics: pause exactly after consuming the shelf node.
    out.push_back(-1);
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

  // Disable legacy intrinsic pause path entirely.
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

void CustomPlannerNode::publishRadioStopWaitingPulse(uint32_t target_robot_id) {
  if (stop_waiting_pulse_active_) {
    endStopWaitingPulseLocked();
  }

  ++stop_wait_tx_seq_;
  if (stop_wait_tx_seq_ == 0) {
    ++stop_wait_tx_seq_;
  }
  pulse_seq_ = stop_wait_tx_seq_;
  pulse_target_id_ = std::min(target_robot_id, 255u);
  stop_waiting_pulse_active_ = true;

  std_msgs::UInt32 tid;
  tid.data = pulse_target_id_;
  target_id_send_pub_.publish(tid);

  std_msgs::Bool sw;
  sw.data = true;
  stop_waiting_send_pub_.publish(sw);
  stop_waiting_pulse_active_ = true;

  std_msgs::UInt8 sq;
  sq.data = pulse_seq_;
  stop_wait_seq_pub_.publish(sq);

  stop_waiting_max_hold_timer_.stop();
  stop_waiting_max_hold_timer_ =
      nh_.createTimer(ros::Duration(stop_waiting_max_hold_s_), &CustomPlannerNode::onStopWaitingMaxHoldTimer, this,
                      true, true);
}

void CustomPlannerNode::endStopWaitingPulseLocked() {
  if (!stop_waiting_pulse_active_) {
    return;
  }
  stop_waiting_pulse_active_ = false;
  stop_waiting_max_hold_timer_.stop();
  std_msgs::Bool off;
  off.data = false;
  stop_waiting_send_pub_.publish(off);
  std_msgs::UInt8 z;
  z.data = 0;
  stop_wait_seq_pub_.publish(z);
  // Preserve MSG serialization semantics: if processTimelineLocked() had deferred on a consecutive
  // MSG_* while this pulse was active, resume now that the pulse ended.
  processTimelineLocked();
}

void CustomPlannerNode::onNetworkTable(const pi_pico_driver::RadioNetworkTable::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!stop_waiting_pulse_active_) {
    return;
  }
  if (robot_id_ < 0) {
    return;
  }
  const size_t n = msg->waiting.size();
  if (n == 0) {
    return;
  }
  if (pulse_target_id_ == 255) {
    for (size_t i = 0; i < n; ++i) {
      if (static_cast<int>(i) == robot_id_) {
        continue;
      }
      if (msg->waiting[i]) {
        return;
      }
    }
  } else {
    if (pulse_target_id_ >= n) {
      endStopWaitingPulseLocked();
      return;
    }
    if (msg->waiting[pulse_target_id_]) {
      return;
    }
  }
  CP_DBG("custom_planner: stop_waiting pulse ended (network_table release, target=%u seq=%u)",
         static_cast<unsigned>(pulse_target_id_), static_cast<unsigned>(pulse_seq_));
  endStopWaitingPulseLocked();
}

void CustomPlannerNode::onStopWaitingMaxHoldTimer(const ros::TimerEvent& ev) {
  (void)ev;
  std::lock_guard<std::mutex> lock(mtx_);
  if (!stop_waiting_pulse_active_) {
    return;
  }
  CP_DBG("custom_planner: stop_waiting pulse ended (max_hold %.3fs, target=%u seq=%u)",
         stop_waiting_max_hold_s_, static_cast<unsigned>(pulse_target_id_),
         static_cast<unsigned>(pulse_seq_));
  endStopWaitingPulseLocked();
}

void CustomPlannerNode::processTimelineLocked() {
  if (!has_active_task_) return;
  if (state_ == STATE_WAITING) return;

  while (timeline_cursor_ < active_task_.timeline_tokens.size()) {
    if (timeline_cursor_ >= active_task_.token_required_consumed_count.size()) break;
    if (consumed_nav_nodes_count_ < active_task_.token_required_consumed_count[timeline_cursor_]) break;

    const int tok = active_task_.timeline_tokens[timeline_cursor_];
    const int32_t executed_idx = static_cast<int32_t>(timeline_cursor_);
    last_executed_token_index_ = executed_idx;
    last_executed_token_value_ = static_cast<int32_t>(tok);
    if (isPhysicalGraphNode(tok)) {
      ++timeline_cursor_;
      publishActiveTaskDebugLocked();
      continue;
    }
    if (isMsgLegToken(tok)) {
      // Serialize MSG_* tokens: if a previous pulse is still in flight, defer this one. Otherwise
      // the second target_id would overwrite the first inside pi_pico_driver before the Pico had a
      // chance to TX the first on radio (Pico CMD is sent every 20 ms; pulse hold ≥ 120 ms).
      // The timer that ends the active pulse calls processTimelineLocked() to drain queued MSGs.
      if (stop_waiting_pulse_active_) {
        return;
      }
      const uint32_t target = static_cast<uint32_t>(-1000 - tok);
      publishRadioStopWaitingPulse(target);
      ++timeline_cursor_;
      publishActiveTaskDebugLocked();
      continue;
    }
    if (tok == -1) {
      if (!pending_peer_releases_.empty()) {
        pending_peer_releases_.pop_front();
        ++timeline_cursor_;
        publishActiveTaskDebugLocked();
        ROS_INFO(
            "custom_planner: timeline -1 consumed from buffered peer release (remaining FIFO=%zu)",
            pending_peer_releases_.size());
        processTimelineLocked();
        return;
      }
      setState(STATE_WAITING);
      publishRadioWakeRequest(robot_id_);
      ++timeline_cursor_;
      publishActiveTaskDebugLocked();
      return;
    }
    ++timeline_cursor_;
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
  publishActiveTaskDebugLocked();
  return true;
}

void CustomPlannerNode::setState(MissionState new_state) {
  const MissionState prev = state_;
  state_ = new_state;
  if (new_state == STATE_WAITING && prev != STATE_WAITING) {
    wait_state_resync_armed_ = true;
  }
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

