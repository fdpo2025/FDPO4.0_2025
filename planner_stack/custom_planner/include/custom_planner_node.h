#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt32MultiArray.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

class CustomPlannerNode {
 public:
  explicit CustomPlannerNode(ros::NodeHandle& nh);

 private:
  enum MissionState {
    STATE_IDLE = 0,
    STATE_NAVIGATING = 1,
    STATE_WAITING = 2
  };

  struct MissionSegment {
    std::vector<int> nodes;
    bool wait_after = false;
    int notify_robot_id = -2;  // -2: none, 255: broadcast, >=0: target robot id
    /** 0-based indices into nodes (same order as /planned_paths body): pause after waypoint consumed. */
    std::vector<uint32_t> pause_after_wp_index;
  };

  struct MissionTask {
    std::vector<int> timeline_tokens;
    std::vector<int> nav_nodes;
    std::vector<uint32_t> token_required_consumed_count;
    /**
     * When 1, plan_handler omits the first /planned_paths node from NavPlan if it is a warehouse
     * (same IDs as plan_handler_node). Timeline counts physical nodes in nav_nodes including that
     * node, so we add this to (plan_index+1) when mapping nav_plan_waypoint_consumed.
     */
    uint8_t plan_handler_skips_first_nav_waypoint = 0;
  };

  void onMissionColorSequence(const std_msgs::String::ConstPtr& msg);
  void onRobotIdentity(const std_msgs::Int32::ConstPtr& msg);
  /** Same logic as /robot_identity callback; optional param initial_robot_id at startup. */
  void applyRobotIdentity(int new_id);
  void onWaitState(const std_msgs::Bool::ConstPtr& msg);
  void onThisCurrentPose(const std_msgs::UInt32::ConstPtr& msg);
  void onNavRoutePauseRequest(const std_msgs::Bool::ConstPtr& msg);
  void onNavPlanWaypointConsumed(const std_msgs::UInt32MultiArray::ConstPtr& msg);
  void onPeerWaitRelease(const std_msgs::Empty::ConstPtr& msg);

  bool loadMissions();
  std::string selectMangaKey(const std::string& color_sequence) const;
  std::vector<MissionSegment> buildMissionSegments(const std::string& color_sequence,
                                                   const std::string& manga_key) const;
  std::vector<int> resolveMissionLeg(const XmlRpc::XmlRpcValue& leg, const std::string& color_sequence,
                                     std::vector<uint32_t>* leg_pause_out,
                                     bool collapse_duplicates = true) const;
  int resolveIndexedColorNode(const std::string& token, const std::string& color_sequence) const;
  bool parseColorWithWaitSuffix(const std::string& token, std::string& color_token_out,
                                bool& wait_at_pick_out) const;
  bool parseColorWith900Suffix(const std::string& token, std::string& color_token_out, bool& wait_at_pick_out,
                               bool& use_900_approach_out, bool& end_on_900_out) const;
  bool parseMessageToken(const XmlRpc::XmlRpcValue& item, int& target_robot_id) const;
  bool isWaitToken(const XmlRpc::XmlRpcValue& item) const;
  bool tryReadInt(const XmlRpc::XmlRpcValue& item, int& value) const;
  void appendWarehousePickupTraversal(int approach_source_shelf_node, int target_shelf_node,
                                      std::vector<int>& out, bool wait_at_pick) const;
  void collapseConsecutiveDuplicateNodes(std::vector<int>& nodes) const;

  void startMission(const std::string& color_sequence);
  void publishMissionRoute(const std::vector<int>& nav_nodes);
  void publishActiveTaskDebugLocked();
  void processTimelineLocked();
  bool loadNextTaskLocked();
  void publishRadioStopWaitingPulse(uint32_t target_robot_id);
  void setState(MissionState new_state);

  bool validatePath(const std::vector<int>& path) const;
  void loadValidNodeIds();
  void publishSpawnPose(const std::string& manga_key);
  void publishRadioWakeRequest(int wait_topic_value);
  bool tryReadDouble(const XmlRpc::XmlRpcValue& v, double& out) const;

  /** Must match is_warehouse_coordinate in plan_handler_node.cpp (first-node skip rule). */
  static bool isWarehouseCoordinate(int node_id);

  ros::NodeHandle nh_;
  ros::Subscriber mission_color_sub_;
  ros::Subscriber robot_identity_sub_;
  ros::Subscriber wait_state_sub_;
  ros::Subscriber peer_wait_release_sub_;
  ros::Subscriber this_pose_sub_;
  ros::Subscriber nav_plan_waypoint_consumed_sub_;
  ros::Publisher planned_paths_pub_;
  ros::Publisher nav_pause_after_wp_index_pub_;
  ros::Subscriber nav_route_pause_request_sub_;
  ros::Publisher mission_state_pub_;
  ros::Publisher radio_wait_target_pub_;
  ros::Publisher wt_send_pub_;
  ros::Publisher target_id_send_pub_;
  ros::Publisher stop_waiting_send_pub_;
  ros::Publisher timeline_tokens_pub_;
  ros::Publisher timeline_cursor_pub_;
  ros::Publisher active_task_nav_nodes_pub_;
  ros::Publisher last_executed_token_pub_;
  ros::Publisher spawn_pose_pub_;

  std::string color_sequence_topic_;
  std::string wait_state_topic_;
  std::string peer_wait_release_topic_;
  std::string this_current_pose_topic_;
  int num_robots_ = 2;
  int radio_stop_waiting_retries_ = 1;

  XmlRpc::XmlRpcValue missions_root_;
  std::vector<int> valid_node_ids_;
  std::unordered_map<int, int> color_input_node_by_index_;
  std::deque<MissionTask> pending_tasks_;
  MissionTask active_task_;
  bool has_active_task_ = false;
  size_t timeline_cursor_ = 0;
  int32_t last_executed_token_index_ = -1;
  int32_t last_executed_token_value_ = 0;
  uint32_t consumed_nav_nodes_count_ = 0;
  uint32_t active_nav_plan_seq_ = 0;
  bool active_nav_plan_seq_valid_ = false;
  bool mission_route_published_ = false;
  std::string last_color_sequence_;
  std::string pending_color_sequence_;
  int robot_id_ = -1;
  MissionState state_ = STATE_IDLE;
  mutable std::mutex mtx_;
  bool debug_verbose_ = false;
  ros::Timer stop_waiting_reset_timer_;
  /** True while a stop_waiting pulse is in-flight (between publish=true and timer reset to false).
   *  Used to serialize consecutive MSG_* tokens so a second target_id cannot overwrite the first
   *  before the Pico has had a chance to TX it on radio. */
  bool stop_waiting_pulse_active_ = false;
  /** Last /pi_pico_driver/wait_state from network (true = this robot waiting on radio). */
  bool last_pi_wait_state_ = false;
  /** After entering STATE_WAITING, ignore one edge cycle: re-baseline last_pi_wait_state_ so a stale
   *  True→False from before this wait cannot immediately release (fixes leading "W" skipped). */
  bool wait_state_resync_armed_ = false;
  /**
   * FIFO of peer stop_waiting events. Every peer_wait_release is buffered here. The FIFO is
   * consumed exactly once per logical release:
   *   - When processing a `-1` token with non-empty FIFO: pop_front, skip WAITING (early release).
   *   - When the wait_state falling edge fires while in STATE_WAITING: pop_front (this release is
   *     the cause of the current wait ending; popping prevents it from "ghost"-releasing the
   *     next `-1` such as an `_W`).
   * Remaining entries after a pop are legitimate early releases for future `-1` tokens.
   */
  std::deque<uint8_t> pending_peer_releases_;
  /**
   * Defensive against ROS callback reordering: if onWaitState fires the falling edge BEFORE the
   * matching onPeerWaitRelease (anomaly — pi_pico_driver normally publishes peer_wait_release
   * first), arm a short window in which the next incoming peer_wait_release is dropped instead of
   * becoming a ghost in pending_peer_releases_.
   */
  int extra_peer_release_drops_ = 0;
  ros::Time extra_peer_release_drops_until_;
};

