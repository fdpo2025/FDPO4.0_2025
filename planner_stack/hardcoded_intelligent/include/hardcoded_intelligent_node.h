#pragma once

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <plan_handler/CompletionFeedback.h>
#include <geometry_msgs/PoseStamped.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <deque>
#include <mutex>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>

class HardcodedIntelligentNode
{
public:
  explicit HardcodedIntelligentNode(ros::NodeHandle& nh);

private:
  enum MissionState {
    STATE_IDLE = 0,
    STATE_NAVIGATING = 1,
    STATE_WAITING = 2
  };

  struct MissionSegment {
    std::vector<int> nodes;
    bool wait_after = false;
    int notify_robot_id = -2; // -2: none, -1: broadcast, >=0: target robot id
  };

  void onColorSequence(const std_msgs::String::ConstPtr& msg);
  void onRobotIdentity(const std_msgs::Int32::ConstPtr& msg);
  void onNavigationFeedback(const plan_handler::CompletionFeedback::ConstPtr& msg);
  void onWaitRelease(const std_msgs::Bool::ConstPtr& msg);

  bool loadMissions();
  std::string selectMangaKey(const std::string& color_sequence) const;
  std::vector<MissionSegment> buildMissionSegments(const std::string& color_sequence,
                                                  const std::string& manga_key) const;
  std::vector<int> resolveMissionLeg(const XmlRpc::XmlRpcValue& leg, const std::string& color_sequence,
                                     const XmlRpc::XmlRpcValue& manga_cfg) const;
  int resolveColorToInputNode(char color, const std::string& color_sequence, std::map<char, int>& local_color_claim_counter,
                              const XmlRpc::XmlRpcValue& manga_cfg) const;
  int getColorClaimBaseRank(char color, const XmlRpc::XmlRpcValue& manga_cfg) const;
  bool robotUsesColor(const XmlRpc::XmlRpcValue& robot_cfg, char color) const;
  bool parseColorWithWaitSuffix(const std::string& token, char& color_out, bool& wait_at_pick_out) const;
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
  ros::Subscriber color_seq_sub_;
  ros::Subscriber robot_identity_sub_;
  ros::Subscriber nav_feedback_sub_;
  ros::Subscriber wait_release_sub_;
  ros::Publisher planned_paths_pub_;
  ros::Publisher mission_state_pub_;
  ros::Publisher radio_wait_target_pub_;
  ros::Publisher spawn_pose_pub_;

  XmlRpc::XmlRpcValue missions_root_;
  std::vector<int> valid_node_ids_;
  std::unordered_map<int, int> color_input_node_by_index_;
  std::deque<MissionSegment> pending_segments_;
  std::vector<int> active_segment_nodes_;
  std::string last_color_sequence_;
  std::string pending_color_sequence_;
  int active_segment_feedback_count_ = 0;
  int robot_id_ = -1;
  MissionState state_ = STATE_IDLE;
  mutable std::mutex mtx_;
  bool debug_verbose_ = false;
};
