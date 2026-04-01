#pragma once

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <plan_handler/CompletionFeedback.h>
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
  std::vector<MissionSegment> buildMissionSegments(const std::string& color_sequence) const;
  std::vector<int> resolveMissionLeg(const XmlRpc::XmlRpcValue& leg, const std::string& color_sequence) const;
  int resolveColorToInputNode(char color, const std::string& color_sequence, std::map<char, int>& local_color_claim_counter) const;
  int getColorClaimBaseRank(char color) const;
  bool robotUsesColor(const XmlRpc::XmlRpcValue& robot_cfg, char color) const;
  bool parseMessageToken(const XmlRpc::XmlRpcValue& item, int& target_robot_id) const;
  bool isWaitToken(const XmlRpc::XmlRpcValue& item) const;
  bool tryReadInt(const XmlRpc::XmlRpcValue& item, int& value) const;

  void startMission(const std::string& color_sequence);
  void publishCurrentSegment();
  void advanceStateMachineAfterSegment();
  void setState(MissionState new_state);

  bool validatePath(const std::vector<int>& path) const;
  void loadValidNodeIds();

  ros::NodeHandle nh_;
  ros::Subscriber color_seq_sub_;
  ros::Subscriber robot_identity_sub_;
  ros::Subscriber nav_feedback_sub_;
  ros::Subscriber wait_release_sub_;
  ros::Publisher planned_paths_pub_;
  ros::Publisher mission_state_pub_;
  ros::Publisher radio_wait_target_pub_;

  XmlRpc::XmlRpcValue missions_root_;
  std::vector<int> valid_node_ids_;
  std::unordered_map<int, int> color_input_node_by_index_;
  std::deque<MissionSegment> pending_segments_;
  std::vector<int> active_segment_nodes_;
  std::string last_color_sequence_;
  int active_segment_feedback_count_ = 0;
  int robot_id_ = -1;
  MissionState state_ = STATE_IDLE;
  mutable std::mutex mtx_;
};
