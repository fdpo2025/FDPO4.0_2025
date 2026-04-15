#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32MultiArray.h>

#include "planner.h"
#include "warehouse_approach_topology.h"

#include <string>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct RobotState {
    int node = 31;
    int current_node = 31;
    int last_node = -1;
    int box = EMPTY;
    int goal = -1;
    std::vector<int> path;
    std::vector<int> compact_path;
    bool busy = false;
    bool waiting_replan = false;
    std::string task_type;
    int reserved_pickup_node = -1;
    int held_published_last_node = -1;
};

class MultiPlannerNode {
public:
    explicit MultiPlannerNode(ros::NodeHandle& nh);

private:
    // Callbacks
    void sequenceCb(const std_msgs::String::ConstPtr& msg);
    void robot1NodeCb(const std_msgs::UInt32::ConstPtr& msg);
    void robot2NodeCb(const std_msgs::UInt32::ConstPtr& msg);

    // Core logic
    State buildState(const std::string& robot_id) const;
    bool planForRobot(const std::string& robot_id);
    void updateRobotPosition(const std::string& robot_id, int node);
    void goalReached(const std::string& robot_id);

    // Helpers
    std::vector<int> sequenceToBoxtypes(const std::string& seq) const;
    std::vector<int> shortestPathAvoiding(int start, int goal,
                                          const std::unordered_set<int>& blocked) const;
    std::vector<int> compactExistingPath(const std::vector<int>& path) const;
    double pathCost(const std::vector<int>& path) const;
    std::vector<int> extendPathWithPreviousNode(const std::vector<int>& path) const;
    std::vector<int> dropFirstNodeUnlessStart31(const std::vector<int>& path) const;
    std::vector<int> adaptPublishedPath(const std::vector<int>& path) const;

    // Reservation system
    bool reserveBoxAtPlanning(const std::string& robot_id, int pickup_node);
    void reservePath(const std::string& robot_id, const std::vector<int>& path, int goal);
    void releaseHeldPublishedLastNode(const std::string& robot_id);
    void holdPublishedLastNode(const std::string& robot_id, const std::vector<int>& published_path);
    bool releaseNode(const std::string& robot_id, int node);
    bool releaseNodesBefore(const std::string& robot_id, int current_node);
    bool releaseAllPathNodesExceptCurrent(const std::string& robot_id, int current_node);
    void tryReplanWaitingRobot(const std::string& freed_by);

    // Pickup priority
    std::unordered_set<int> getUnavailablePickupNodes(const std::string& robot_id) const;
    std::unordered_set<int> getExtraBlockedNodes(const std::string& robot_id) const;
    int getBoxTypeAtNode(int node) const;
    int getRobotActiveTaskBoxType(const std::string& robot_id) const;

    struct PickupPriority {
        bool active = false;
        std::string mode;  // "prefer_exact" or "avoid_exact"
        int target_box_type = EMPTY;
    };
    PickupPriority getPickupPriorityMode(const std::string& robot_id) const;
    std::vector<std::vector<int>> splitPickupCandidatesByPriority(
        const std::string& robot_id, const std::vector<int>& valid_nodes) const;

    // +100: id “espelhado” do nó base (ex. prateleira). +900: filas auxiliares no grafo — não são
    // warehouse; não aplicar id-100 (908→808 quebraria reservas / current_node).
    static int resolveWarehouseId(int id) {
        if ((id >= 908 && id <= 911) || (id >= 927 && id <= 930)) return id;
        return id >= 100 ? id - 100 : id;
    }
    bool isWarehouseNode(int id) const { return warehouse_nodes_.count(id) != 0; }
    int determineApproachSideNode(const std::vector<int>& full_path, int warehouse_node) const;

    // Output warehouse rule
    std::vector<int> applyOutputWarehouseRule(const std::vector<int>& valid_nodes) const;

    bool canPickupBoxForMachine(const std::string& robot_id, int pickup_node,
                                int source_box_type,
                                const std::vector<int>& machine_inputs,
                                const std::vector<int>& machine_outputs,
                                const std::string& label) const;

    // State publishing
    void publishStateSnapshot(const std::string& robot_id, const std::string& label,
                              const State& state);
    void publishLogicalState(const std::string& robot_id);
    void publishPath(const std::string& robot_id, const std::vector<int>& path);

    // ROS
    ros::NodeHandle& nh_;
    ros::Publisher pub_r1_;
    ros::Publisher pub_r2_;
    ros::Publisher pub_state_;
    ros::Subscriber sub_sequence_;
    ros::Subscriber sub_r1_pose_;
    ros::Subscriber sub_r2_pose_;

    // Planner core
    std::unique_ptr<Planner> planner_;

    // Robot states
    std::unordered_map<std::string, RobotState> robots_;

    // Boxes shared state
    std::vector<int> boxes_;

    // Reservation maps
    std::unordered_map<int, std::string> reserved_nodes_;
    std::unordered_map<int, std::string> reserved_goals_;

    // Special block nodes
    std::unordered_set<int> special_block_nodes_;

    // Output warehouse nodes
    std::unordered_set<int> output_nodes_;

    // All warehouse nodes (input + process + output)
    std::unordered_set<int> warehouse_nodes_;

    // Pickup plan counter
    int pickup_plan_count_;

    WarehouseApproachTopology approach_topology_;

    bool isNodeReservedOrBlockedIndirectly(int node) const;
    bool isDirect11To27GloballyForbidden() const;
    bool isDirect11To27ReservedConsecutively() const;
};
