#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
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

  void onMissionColorSequence(const std_msgs::String::ConstPtr& msg);
  void onRobotIdentity(const std_msgs::Int32::ConstPtr& msg);
  /** Same logic as /robot_identity callback; optional param initial_robot_id at startup. */
  void applyRobotIdentity(int new_id);
  void onWaitState(const std_msgs::Bool::ConstPtr& msg);
  void onThisCurrentPose(const std_msgs::UInt32::ConstPtr& msg);
  void onNavRoutePauseRequest(const std_msgs::Bool::ConstPtr& msg);

  bool loadMissions();
  std::string selectMangaKey(const std::string& color_sequence) const;
  std::vector<MissionSegment> buildMissionSegments(const std::string& color_sequence,
                                                   const std::string& manga_key) const;
  std::vector<int> resolveMissionLeg(const XmlRpc::XmlRpcValue& leg, const std::string& color_sequence,
                                     std::vector<uint32_t>* leg_pause_out) const;
  int resolveIndexedColorNode(const std::string& token, const std::string& color_sequence) const;
  bool parseColorWithWaitSuffix(const std::string& token, std::string& color_token_out,
                                bool& wait_at_pick_out) const;
  bool parseMessageToken(const XmlRpc::XmlRpcValue& item, int& target_robot_id) const;
  bool isWaitToken(const XmlRpc::XmlRpcValue& item) const;
  bool tryReadInt(const XmlRpc::XmlRpcValue& item, int& value) const;
  void appendWarehousePickupTraversal(int input_shelf_node, std::vector<int>& out, bool wait_at_pick) const;
  void collapseConsecutiveDuplicateNodes(std::vector<int>& nodes) const;

  void startMission(const std::string& color_sequence);
  void publishCurrentSegment();
  void advanceStateMachineAfterSegment();
  void setState(MissionState new_state);

  bool validatePath(const std::vector<int>& path) const;
  void loadValidNodeIds();
  void publishSpawnPose(const std::string& manga_key);
  void publishRadioWakeRequest(int wait_topic_value);
  bool tryReadDouble(const XmlRpc::XmlRpcValue& v, double& out) const;

  ros::NodeHandle nh_;
  ros::Subscriber mission_color_sub_;
  ros::Subscriber robot_identity_sub_;
  ros::Subscriber wait_state_sub_;
  ros::Subscriber this_pose_sub_;
  ros::Publisher planned_paths_pub_;
  ros::Publisher nav_pause_after_wp_index_pub_;
  ros::Subscriber nav_route_pause_request_sub_;
  ros::Publisher mission_state_pub_;
  ros::Publisher radio_wait_target_pub_;
  ros::Publisher wt_send_pub_;
  ros::Publisher spawn_pose_pub_;

  std::string color_sequence_topic_;
  std::string wait_state_topic_;
  std::string this_current_pose_topic_;
  int num_robots_ = 2;

  XmlRpc::XmlRpcValue missions_root_;
  std::vector<int> valid_node_ids_;
  std::unordered_map<int, int> color_input_node_by_index_;
  std::deque<MissionSegment> pending_segments_;
  std::vector<int> active_segment_nodes_;
  /** True if active segment was published with non-empty pause_after_wp_index (mid-route wait, no re-publish). */
  bool active_segment_has_intrinsic_wait_ = false;
  std::string last_color_sequence_;
  std::string pending_color_sequence_;
  int robot_id_ = -1;
  MissionState state_ = STATE_IDLE;
  mutable std::mutex mtx_;
  bool debug_verbose_ = false;
  /** Last /pi_pico_driver/wait_state from network (true = this robot waiting on radio). */
  bool last_pi_wait_state_ = false;
};

