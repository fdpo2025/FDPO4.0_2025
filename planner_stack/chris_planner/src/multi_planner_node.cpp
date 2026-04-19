#include "multi_planner_node.h"

#include <yaml-cpp/yaml.h>
#include <queue>
#include <algorithm>
#include <sstream>
#include <stdexcept>

// =====================================================================
// Constructor
// =====================================================================

MultiPlannerNode::MultiPlannerNode(ros::NodeHandle& nh)
    : nh_(nh)
    , pickup_plan_count_(0)
{
    std::string package_path = ros::package::getPath("chris_planner");
    if (package_path.empty()) {
        ROS_FATAL("chris_planner: ros::package::getPath retornou vazio (ROS_PACKAGE_PATH?)");
        throw std::runtime_error("chris_planner: package path empty");
    }
    approach_topology_.loadFromFile(package_path +
                                    "/files/inputs/warehouse_approach_topology.yaml");

    std::string graph_file = package_path + "/files/inputs/graph.yaml";
    std::string factory_file = package_path + "/files/inputs/factory_components.yaml";

    YAML::Node graph_dict = YAML::LoadFile(graph_file);
    YAML::Node factory_dict = YAML::LoadFile(factory_file);

    std::string planning_method;
    nh_.param<std::string>("planning_method", planning_method, "astar");

    planner_ = std::make_unique<Planner>(graph_dict, factory_dict, planning_method);

    // Build special_block_nodes from factory_components_dict
    for (auto it = factory_dict.begin(); it != factory_dict.end(); ++it) {
        std::string key = it->first.as<std::string>();
        const auto& value = it->second;
        if (value.IsSequence()) {
            for (const auto& item : value)
                special_block_nodes_.insert(item.as<int>());
        } else if (value.IsMap()) {
            for (auto sub = value.begin(); sub != value.end(); ++sub) {
                if (sub->second.IsSequence()) {
                    for (const auto& item : sub->second)
                        special_block_nodes_.insert(item.as<int>());
                }
            }
        }
    }

    output_nodes_ = {35, 36, 37, 38};

    // All warehouse nodes (input + machine I/O + output)
    const auto& f = planner_->factory;
    for (int n : f.input_warehouse)   warehouse_nodes_.insert(n);
    for (int n : f.machineA_inputs)   warehouse_nodes_.insert(n);
    for (int n : f.machineA_outputs)  warehouse_nodes_.insert(n);
    for (int n : f.machineB_inputs)   warehouse_nodes_.insert(n);
    for (int n : f.machineB_outputs)  warehouse_nodes_.insert(n);
    for (int n : f.output_warehouse)  warehouse_nodes_.insert(n);

    // Initialize robots
    robots_["r1"] = RobotState();
    robots_["r2"] = RobotState();

    // Publishers
    pub_r1_    = nh_.advertise<std_msgs::Int32MultiArray>("/robot1_planned_paths", 10);
    pub_r2_    = nh_.advertise<std_msgs::Int32MultiArray>("/robot2_planned_paths", 10);
    pub_state_ = nh_.advertise<std_msgs::String>("/planner/logical_state", 10);

    // Subscribers
    sub_sequence_ = nh_.subscribe("/color_sequence", 1, &MultiPlannerNode::sequenceCb, this);
    sub_r1_pose_  = nh_.subscribe("/robot1/current_pose", 10, &MultiPlannerNode::robot1NodeCb, this);
    sub_r2_pose_  = nh_.subscribe("/robot2/current_pose", 10, &MultiPlannerNode::robot2NodeCb, this);

    ROS_INFO("MultiPlannerNode initialized");
}

// =====================================================================
// State snapshot publishing
// =====================================================================

void MultiPlannerNode::publishStateSnapshot(const std::string& robot_id,
                                            const std::string& label,
                                            const State& state)
{
    std::ostringstream ss;
    ss << robot_id << " | " << label << " | logical state=("
       << state.robot_node << ", " << state.robot_box_type << ", [";
    for (int i = 0; i < static_cast<int>(state.boxes.size()); ++i) {
        if (i > 0) ss << ", ";
        ss << state.boxes[i];
    }
    ss << "])";

    std_msgs::String msg;
    msg.data = ss.str();
    pub_state_.publish(msg);
}

void MultiPlannerNode::publishLogicalState(const std::string& robot_id)
{
    State state = buildState(robot_id);
    publishStateSnapshot(robot_id, "current", state);
}

// =====================================================================
// Callbacks
// =====================================================================

void MultiPlannerNode::sequenceCb(const std_msgs::String::ConstPtr& msg)
{
    std::string seq = msg->data;
    // Trim whitespace
    while (!seq.empty() && std::isspace(seq.front())) seq.erase(seq.begin());
    while (!seq.empty() && std::isspace(seq.back())) seq.pop_back();

    ROS_INFO("Received sequence %s", seq.c_str());

    auto boxtypes = sequenceToBoxtypes(seq);
    State initial = planner_->factory.initialState(boxtypes);
    boxes_ = initial.boxes;

    reserved_nodes_.clear();
    reserved_goals_.clear();
    pickup_plan_count_ = 0;

    for (auto& [id, r] : robots_) {
        r.node = 31;
        r.current_node = 31;
        r.last_node = -1;
        r.box = EMPTY;
        r.goal = -1;
        r.path.clear();
        r.compact_path.clear();
        r.busy = false;
        r.waiting_replan = false;
        r.task_type.clear();
        r.reserved_pickup_node = -1;
        r.held_published_last_node = -1;
    }

    ROS_WARN("Initial boxes set");

    publishLogicalState("r1");
    publishLogicalState("r2");

    planForRobot("r1");
    planForRobot("r2");
}

void MultiPlannerNode::robot1NodeCb(const std_msgs::UInt32::ConstPtr& msg)
{
    updateRobotPosition("r1", static_cast<int>(msg->data));
}

void MultiPlannerNode::robot2NodeCb(const std_msgs::UInt32::ConstPtr& msg)
{
    updateRobotPosition("r2", static_cast<int>(msg->data));
}

// =====================================================================
// Build state
// =====================================================================

State MultiPlannerNode::buildState(const std::string& robot_id) const
{
    const auto& r = robots_.at(robot_id);
    State s{r.node, r.box, boxes_};
    s.recomputeHash();
    return s;
}

// =====================================================================
// Sequence parsing
// =====================================================================

std::vector<int> MultiPlannerNode::sequenceToBoxtypes(const std::string& seq) const
{
    std::vector<int> boxtypes;
    for (char c : seq) {
        char upper = std::toupper(c);
        switch (upper) {
            case 'R': boxtypes.push_back(TYPE_A); break;
            case 'G': boxtypes.push_back(TYPE_B); break;
            case 'B': boxtypes.push_back(TYPE_C); break;
            default:
                ROS_WARN("Invalid color '%c', ignored", c);
                break;
        }
    }
    return boxtypes;
}

// =====================================================================
// Plan for robot
// =====================================================================

bool MultiPlannerNode::planForRobot(const std::string& robot_id)
{
    auto& r = robots_[robot_id];
    auto formatVector = [](const std::vector<int>& values) {
        std::ostringstream ss;
        ss << "[";
        for (size_t i = 0; i < values.size(); ++i) {
            if (i > 0) ss << ", ";
            ss << values[i];
        }
        ss << "]";
        return ss.str();
    };
    auto formatSet = [](const std::unordered_set<int>& values) {
        std::vector<int> sorted(values.begin(), values.end());
        std::sort(sorted.begin(), sorted.end());
        std::ostringstream ss;
        ss << "[";
        for (size_t i = 0; i < sorted.size(); ++i) {
            if (i > 0) ss << ", ";
            ss << sorted[i];
        }
        ss << "]";
        return ss.str();
    };
    auto dumpBoxes = [this]() {
        std::ostringstream ss;
        ss << "[";
        const auto& special_nodes = planner_->factory.special_nodes;
        for (size_t i = 0; i < special_nodes.size() && i < boxes_.size(); ++i) {
            if (i > 0) ss << ", ";
            ss << special_nodes[i] << ":" << boxes_[i];
        }
        ss << "]";
        return ss.str();
    };

    if (r.busy) {
        ROS_INFO("%s is already busy", robot_id.c_str());
        return false;
    }

    State state = buildState(robot_id);
    publishStateSnapshot(robot_id, "before planning", state);

    int robot_node = r.current_node;
    ROS_INFO("[%s] planning from physical node=%d", robot_id.c_str(), robot_node);

    auto valid_nodes = planner_->factory.validDestinations(state);

    auto unavailable = getUnavailablePickupNodes(robot_id);

    std::vector<int> filtered;
    for (int n : valid_nodes) {
        if (reserved_goals_.count(n)) continue;
        if (unavailable.count(n)) continue;
        filtered.push_back(n);
    }

    // Output warehouse rule for dropoff
    if (r.box != EMPTY)
        filtered = applyOutputWarehouseRule(filtered);

    ROS_INFO("[%s][DEBUG] valid_nodes=%s unavailable_pickups=%s filtered=%s reserved_goals=%zu boxes=%s",
             robot_id.c_str(),
             formatVector(valid_nodes).c_str(),
             formatSet(unavailable).c_str(),
             formatVector(filtered).c_str(),
             reserved_goals_.size(),
             dumpBoxes().c_str());

    if (filtered.empty()) {
        ROS_WARN("No valid nodes for %s", robot_id.c_str());
        ROS_WARN("[%s][DEBUG] planning failed: no valid nodes after filtering", robot_id.c_str());
        r.waiting_replan = true;
        publishStateSnapshot(robot_id, "planning failed - no valid nodes", state);
        return false;
    }

    // Blocked nodes
    std::unordered_set<int> reserved_by_other;
    for (auto& [n, owner] : reserved_nodes_)
        if (owner != robot_id) reserved_by_other.insert(n);

    auto extra = getExtraBlockedNodes(robot_id);
    std::unordered_set<int> blocked = reserved_by_other;
    blocked.insert(extra.begin(), extra.end());

    ROS_INFO("[%s][DEBUG] reserved_by_other=%s extra_blocked=%s blocked=%s",
             robot_id.c_str(),
             formatSet(reserved_by_other).c_str(),
             formatSet(extra).c_str(),
             formatSet(blocked).c_str());

    // Candidate groups
    std::vector<std::vector<int>> candidate_groups;
    if (r.box == EMPTY)
        candidate_groups = splitPickupCandidatesByPriority(robot_id, filtered);
    else
        candidate_groups = {filtered};

    for (size_t i = 0; i < candidate_groups.size(); ++i) {
        ROS_INFO("[%s][DEBUG] candidate_group[%zu]=%s",
                 robot_id.c_str(), i, formatVector(candidate_groups[i]).c_str());
    }

    std::vector<int> best_path;
    int best_goal = -1;

    for (const auto& candidates : candidate_groups) {
        std::vector<int> local_best_path;
        int local_best_goal = -1;
        double local_best_cost = std::numeric_limits<double>::infinity();

        for (int node : candidates) {
            if (r.box == EMPTY) {
                if (!canPickupBoxForMachine(robot_id, node, TYPE_A,
                        planner_->factory.machineA_inputs,
                        planner_->factory.machineA_outputs, "red")) {
                    ROS_INFO("[%s] pickup at node %d blocked by red machine capacity", robot_id.c_str(), node);
                    continue;
                }
                if (!canPickupBoxForMachine(robot_id, node, TYPE_B,
                        planner_->factory.machineB_inputs,
                        planner_->factory.machineB_outputs, "green")) {
                    ROS_INFO("[%s] pickup at node %d blocked by green machine capacity", robot_id.c_str(), node);
                    continue;
                }
            }

            auto path = shortestPathAvoiding(robot_node, node, blocked);
            if (path.empty()) {
                ROS_INFO("[%s][DEBUG] candidate node %d rejected: no path from %d with blocked=%s",
                         robot_id.c_str(), node, robot_node, formatSet(blocked).c_str());
                continue;
            }

            double cost = pathCost(path);
            ROS_INFO("[%s][DEBUG] candidate node %d path=%s cost=%.3f",
                     robot_id.c_str(), node, formatVector(path).c_str(), cost);
            if (cost < local_best_cost) {
                local_best_cost = cost;
                local_best_path = path;
                local_best_goal = node;
            }
        }

        if (!local_best_path.empty()) {
            best_path = local_best_path;
            best_goal = local_best_goal;
            break;
        }
    }

    if (best_path.empty()) {
        ROS_WARN("No collision free path for %s", robot_id.c_str());
        ROS_WARN("[%s][DEBUG] planning failed: robot_node=%d filtered=%s blocked=%s boxes=%s",
                 robot_id.c_str(),
                 robot_node,
                 formatVector(filtered).c_str(),
                 formatSet(blocked).c_str(),
                 dumpBoxes().c_str());
        r.waiting_replan = true;
        publishStateSnapshot(robot_id, "planning failed - no path", state);
        return false;
    }

    std::string task_type = (r.box == EMPTY) ? "pickup" : "dropoff";

    if (task_type == "pickup") {
        if (!reserveBoxAtPlanning(robot_id, best_goal)) {
            ROS_WARN("[%s] Failed to reserve box at node %d", robot_id.c_str(), best_goal);
            r.waiting_replan = true;
            publishStateSnapshot(robot_id, "planning failed - reserve pickup", state);
            return false;
        }
        pickup_plan_count_++;
        ROS_INFO("[%s] pickup_plan_count = %d", robot_id.c_str(), pickup_plan_count_);
    }

    auto full_path = dropFirstNodeUnlessStart31(extendPathWithPreviousNode(best_path));
    auto compact_path = dropFirstNodeUnlessStart31(
        extendPathWithPreviousNode(compactExistingPath(best_path)));

    if (isWarehouseNode(best_goal)) {
        int side_node = determineApproachSideNode(best_path, best_goal);
        compact_path[compact_path.size() - 2] = side_node;
        ROS_INFO("[%s] Warehouse %d approach-side resolved to %d",
                 robot_id.c_str(), best_goal, side_node);
    }

    compact_path = adaptPublishedPath(compact_path);

    releaseHeldPublishedLastNode(robot_id);
    reservePath(robot_id, full_path, best_goal);
    publishPath(robot_id, compact_path);
    holdPublishedLastNode(robot_id, compact_path);

    r.path = full_path;
    r.compact_path = compact_path;
    r.goal = best_goal;
    r.busy = true;
    r.waiting_replan = false;
    r.task_type = task_type;

    ROS_INFO("%s planned task_type=%s", robot_id.c_str(), task_type.c_str());

    publishLogicalState(robot_id);
    return true;
}

// =====================================================================
// Reservation
// =====================================================================

bool MultiPlannerNode::reserveBoxAtPlanning(const std::string& robot_id, int pickup_node)
{
    auto& r = robots_[robot_id];
    auto it = planner_->factory.index_of.find(pickup_node);
    if (it == planner_->factory.index_of.end()) {
        ROS_ERROR("[%s] pickup node %d is not a special node", robot_id.c_str(), pickup_node);
        return false;
    }

    int i_pickup = it->second;
    int box_type = boxes_[i_pickup];
    if (box_type == EMPTY) {
        ROS_ERROR("[%s] no box to reserve at node %d", robot_id.c_str(), pickup_node);
        return false;
    }

    ROS_WARN("[%s] reserving pickup node %d (no state change yet)", robot_id.c_str(), pickup_node);
    r.reserved_pickup_node = pickup_node;
    return true;
}

void MultiPlannerNode::reservePath(const std::string& robot_id,
                                   const std::vector<int>& path, int goal)
{
    for (int n : path) {
        if (n == 31) continue;
        reserved_nodes_[n] = robot_id;
    }
    reserved_goals_[goal] = robot_id;
}

void MultiPlannerNode::releaseHeldPublishedLastNode(const std::string& robot_id)
{
    auto& r = robots_[robot_id];
    if (r.held_published_last_node < 0) return;

    auto it = reserved_nodes_.find(r.held_published_last_node);
    if (it != reserved_nodes_.end() && it->second == robot_id)
        reserved_nodes_.erase(it);

    r.held_published_last_node = -1;
}

void MultiPlannerNode::holdPublishedLastNode(
    const std::string& robot_id, const std::vector<int>& published_path)
{
    auto& r = robots_[robot_id];
    if (published_path.empty()) {
        r.held_published_last_node = -1;
        return;
    }

    int held_node = resolveWarehouseId(published_path.back());
    reserved_nodes_[held_node] = robot_id;
    r.held_published_last_node = held_node;
}

void MultiPlannerNode::publishPath(const std::string& robot_id,
                                   const std::vector<int>& path)
{
    std_msgs::Int32MultiArray msg;
    msg.data = path;
    if (robot_id == "r1") pub_r1_.publish(msg);
    else                  pub_r2_.publish(msg);
}

bool MultiPlannerNode::releaseNode(const std::string& robot_id, int node)
{
    auto it = reserved_nodes_.find(node);
    if (it != reserved_nodes_.end() && it->second == robot_id) {
        reserved_nodes_.erase(it);
        return true;
    }
    return false;
}

// =====================================================================
// Position update
// =====================================================================

void MultiPlannerNode::updateRobotPosition(const std::string& robot_id, int node)
{
    int resolved = resolveWarehouseId(node);

    auto& r = robots_[robot_id];
    if (resolved == r.current_node) return;

    r.last_node = r.current_node;
    r.current_node = resolved;

    bool released = releaseNodesBefore(robot_id, resolved);
    if (released) tryReplanWaitingRobot(robot_id);

    publishLogicalState(robot_id);

    if (r.goal >= 0 && resolved == r.goal) {
        ROS_WARN("[%s] GOAL DETECTED at node %d (raw CP=%d)", robot_id.c_str(), resolved, node);
        goalReached(robot_id);
    }
}

// =====================================================================
// Goal reached
// =====================================================================

void MultiPlannerNode::goalReached(const std::string& robot_id)
{
    auto& r = robots_[robot_id];
    int goal = r.goal;
    std::string task_type = r.task_type;
    int published_last_node = r.held_published_last_node;

    releaseAllPathNodesExceptCurrent(robot_id, goal);
    if (published_last_node >= 0)
        reserved_nodes_[published_last_node] = robot_id;

    ROS_INFO("%s reached goal %d with task_type=%s", robot_id.c_str(), goal, task_type.c_str());

    if (task_type == "pickup" || task_type == "dropoff") {
        State state{r.node, r.box, boxes_};
        state.recomputeHash();
        publishStateSnapshot(robot_id, "before " + task_type + " update", state);
        ROS_WARN("[%s] BEFORE %s update: goal=%d", robot_id.c_str(), task_type.c_str(), goal);

        State new_state = planner_->factory.updateState(state, goal);
        publishStateSnapshot(robot_id, "after " + task_type + " update", new_state);

        r.node = new_state.robot_node;
        r.box = new_state.robot_box_type;
        boxes_ = new_state.boxes;
        r.reserved_pickup_node = -1;

        ROS_WARN("[%s] AFTER %s update: node=%d box=%d",
                 robot_id.c_str(), task_type.c_str(), r.node, r.box);
    }

    r.busy = false;
    r.goal = -1;
    r.path.clear();
    r.compact_path.clear();
    r.task_type.clear();

    publishLogicalState(robot_id);

    planForRobot(robot_id);
    tryReplanWaitingRobot(robot_id);
}

// =====================================================================
// Path helpers
// =====================================================================

std::vector<int> MultiPlannerNode::shortestPathAvoiding(
    int start, int goal, const std::unordered_set<int>& blocked_nodes) const
{
    std::unordered_set<int> blocked = blocked_nodes;
    blocked.erase(start);
    blocked.erase(goal);

    const bool forbid_11_27 = isDirect11To27GloballyForbidden();

    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> prev;
    dist[start] = 0.0;

    using Pair = std::pair<double, int>;
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> pq;
    pq.push({0.0, start});

    while (!pq.empty()) {
        auto [curr_dist, u] = pq.top();
        pq.pop();

        auto dit = dist.find(u);
        if (dit != dist.end() && curr_dist > dit->second) continue;
        if (u == goal) break;

        for (auto& [v, w] : planner_->factory.graph.neighbors(u)) {
            if (blocked.count(v)) continue;

            // Regra global:
            // se 12 ou 26 estiverem reservados, proibir apenas a transição direta 11 <-> 27
            if (forbid_11_27 &&
                ((u == 11 && v == 27) || (u == 27 && v == 11))) {
                ROS_INFO("Globally forbidding direct transition %d -> %d because 12/26 are reserved", u, v);
                continue;
            }

            double new_dist = curr_dist + w;
            auto vit = dist.find(v);
            if (vit == dist.end() || new_dist < vit->second) {
                dist[v] = new_dist;
                prev[v] = u;
                pq.push({new_dist, v});
            }
        }
    }

    if (!dist.count(goal)) return {};

    std::vector<int> path;
    int node = goal;
    while (node != start) {
        path.push_back(node);
        node = prev[node];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<int> MultiPlannerNode::compactExistingPath(const std::vector<int>& path) const
{
    if (path.empty()) return {};
    if (path.size() <= 2) return path;

    auto colinear = [&](int idx1, int idx2, int idx3) -> bool {
        auto p1 = planner_->factory.points_map.at(idx1);
        auto p2 = planner_->factory.points_map.at(idx2);
        auto p3 = planner_->factory.points_map.at(idx3);
        double ax = p3.first  - p1.first;
        double ay = p3.second - p1.second;
        double bx = p2.first  - p1.first;
        double by = p2.second - p1.second;
        double denom = std::sqrt(ax * ax + ay * ay);
        if (denom == 0.0) return true;
        return std::abs(ax * by - ay * bx) / denom < 0.001;
    };

    const auto& special_set = planner_->factory.special_set;

    std::vector<int> compact = {path[0]};
    int i = 1;

    if (special_set.count(path[0]) && path.size() > 1) {
        compact.push_back(path[1]);
        i = 2;
    }

    while (i < static_cast<int>(path.size()) - 2) {
        int last_idx = compact.back();
        if (colinear(last_idx, path[i], path[i + 1]))
            ++i;
        else {
            compact.push_back(path[i]);
            ++i;
        }
    }

    for (int j = i; j < static_cast<int>(path.size()); ++j)
        compact.push_back(path[j]);

    return compact;
}

double MultiPlannerNode::pathCost(const std::vector<int>& path) const
{
    if (path.size() < 2) return 0.0;
    double total = 0.0;
    for (int i = 0; i < static_cast<int>(path.size()) - 1; ++i)
        total += planner_->factory.graph.distance(path[i], path[i + 1]);
    return total;
}

std::vector<int> MultiPlannerNode::extendPathWithPreviousNode(
    const std::vector<int>& path) const
{
    if (path.size() < 2) return path;
    auto extended = path;
    extended.push_back(path[path.size() - 2]);
    return extended;
}

std::vector<int> MultiPlannerNode::dropFirstNodeUnlessStart31(
    const std::vector<int>& path) const
{
    if (path.size() <= 1) return path;
    if (path.front() == 31) return path;

    return std::vector<int>(path.begin() + 1, path.end());
}

std::vector<int> MultiPlannerNode::adaptPublishedPath(
    const std::vector<int>& path) const
{
    auto adapted = path;
    int last_special_base_node = -1;
    if (!adapted.empty()) {
        int first_base_node = resolveWarehouseId(adapted.front());
        if (planner_->factory.special_set.count(first_base_node))
            last_special_base_node = first_base_node;
    }
    for (int i = 1; i < static_cast<int>(adapted.size()); ++i) {
        if (adapted[i - 1] == 7 && adapted[i] == 30)
            adapted[i] = 130;
        if (adapted[i - 1] == 30 && adapted[i] == 138)
            adapted[i - 1] = 230;

        int current_base_node = resolveWarehouseId(adapted[i]);
        if (adapted[i] == 8 &&
            (last_special_base_node == 17 || last_special_base_node == 117)) {
            adapted[i] = 208;
            current_base_node = 8;
        }

        if (planner_->factory.special_set.count(current_base_node))
            last_special_base_node = current_base_node;
    }
    return adapted;
}

// =====================================================================
// Warehouse approach-side determination
// =====================================================================

int MultiPlannerNode::determineApproachSideNode(
    const std::vector<int>& full_path, int warehouse_node) const
{
    int wh_idx = -1;
    for (int i = static_cast<int>(full_path.size()) - 1; i >= 0; --i) {
        if (full_path[i] == warehouse_node) { wh_idx = i; break; }
    }
    if (wh_idx < 2) return warehouse_node + 100;

    int neighbor = full_path[wh_idx - 1];
    int pred = full_path[wh_idx - 2];
    bool right = approach_topology_.isRightApproach(warehouse_node, neighbor, pred);
    return right ? warehouse_node : warehouse_node + 100;
}

// =====================================================================
// Replan waiting robot
// =====================================================================

void MultiPlannerNode::tryReplanWaitingRobot(const std::string& freed_by)
{
    std::string other = (freed_by == "r1") ? "r2" : "r1";
    auto& r = robots_[other];
    if (r.busy) return;
    if (r.goal >= 0) return;
    if (!r.waiting_replan) return;

    ROS_INFO("Trying replanning for waiting robot %s", other.c_str());
    planForRobot(other);
}

// =====================================================================
// Release nodes
// =====================================================================

bool MultiPlannerNode::releaseAllPathNodesExceptCurrent(
    const std::string& robot_id, int current_node)
{
    auto& r = robots_[robot_id];
    bool released_any = false;

    if (r.path.empty()) return false;

    auto git = reserved_goals_.find(current_node);
    if (git != reserved_goals_.end() && git->second == robot_id) {
        reserved_goals_.erase(git);
        ROS_INFO("[%s] released current reserved goal %d on goal arrival", robot_id.c_str(), current_node);
    }

    for (int n : r.path) {
        if (n == current_node) continue;

        auto nit = reserved_nodes_.find(n);
        if (nit != reserved_nodes_.end() && nit->second == robot_id) {
            reserved_nodes_.erase(nit);
            released_any = true;
            ROS_INFO("[%s] released path node %d on goal arrival", robot_id.c_str(), n);
        }

        auto git2 = reserved_goals_.find(n);
        if (git2 != reserved_goals_.end() && git2->second == robot_id) {
            reserved_goals_.erase(git2);
            ROS_INFO("[%s] released reserved goal %d on goal arrival", robot_id.c_str(), n);
        }
    }
    return released_any;
}

bool MultiPlannerNode::releaseNodesBefore(const std::string& robot_id, int current_node)
{
    auto& r = robots_[robot_id];
    if (r.path.empty()) return false;

    auto it = std::find(r.path.begin(), r.path.end(), current_node);
    if (it == r.path.end()) {
        ROS_WARN("[%s] current_node %d not in reserved path", robot_id.c_str(), current_node);
        return false;
    }

    bool released_any = false;
    for (auto jt = r.path.begin(); jt != it; ++jt) {
        int n = *jt;
        auto nit = reserved_nodes_.find(n);
        if (nit != reserved_nodes_.end() && nit->second == robot_id) {
            reserved_nodes_.erase(nit);
            released_any = true;
            ROS_INFO("[%s] released past node %d", robot_id.c_str(), n);
        }
        auto git = reserved_goals_.find(n);
        if (git != reserved_goals_.end() && git->second == robot_id) {
            reserved_goals_.erase(git);
            ROS_INFO("[%s] released past reserved goal %d", robot_id.c_str(), n);
        }
    }
    return released_any;
}

// =====================================================================
// Unavailable pickup nodes
// =====================================================================

std::unordered_set<int> MultiPlannerNode::getUnavailablePickupNodes(
    const std::string& robot_id) const
{
    std::unordered_set<int> blocked;
    for (auto& [other_id, other] : robots_) {
        if (other_id == robot_id) continue;
        if (other.box != EMPTY) blocked.insert(other.node);
        if (other.goal >= 0 && other.task_type == "pickup")
            blocked.insert(other.goal);
        if (other.reserved_pickup_node >= 0)
            blocked.insert(other.reserved_pickup_node);
    }
    return blocked;
}

// =====================================================================
// Extra blocked nodes
// =====================================================================

std::unordered_set<int> MultiPlannerNode::getExtraBlockedNodes(
    const std::string& robot_id) const
{
    std::unordered_set<int> blocked;
    if (isNodeReservedOrBlockedIndirectly(30)) {
        blocked.insert(29);
        ROS_INFO("[%s] global block: node 29 blocked because node 30 is reserved or indirectly blocked",
                 robot_id.c_str());
    }

    // Regra global:
    // se algum caminho reservado estiver a usar 11 <-> 27 consecutivamente,
    // então 12 e 26 ficam bloqueados para qualquer novo planeamento.
    if (isDirect11To27ReservedConsecutively()) {
        blocked.insert(12);
        blocked.insert(26);
        ROS_INFO("[%s] global block: nodes 12 and 26 blocked because some reserved path uses 11<->27 consecutively",
                 robot_id.c_str());
    }

    return blocked;
}

// =====================================================================
// Output warehouse rule
// =====================================================================

std::vector<int> MultiPlannerNode::applyOutputWarehouseRule(
    const std::vector<int>& valid_nodes) const
{
    std::vector<int> output_candidates;
    for (int n : valid_nodes)
        if (output_nodes_.count(n)) output_candidates.push_back(n);

    if (output_candidates.empty()) return valid_nodes;

    std::unordered_set<int> committed_outputs;
    for (const auto& [other_id, other] : robots_) {
        if (other.goal >= 0 && other.task_type == "dropoff" &&
            output_nodes_.count(other.goal)) {
            committed_outputs.insert(other.goal);
        }
    }

    for (int node : output_nodes_) {
        auto it = planner_->factory.index_of.find(node);
        if (it != planner_->factory.index_of.end() && boxes_[it->second] != EMPTY)
            committed_outputs.insert(node);
    }

    auto restrict_to_output = [&](int target_output, const char* reason) {
        if (std::find(output_candidates.begin(), output_candidates.end(), target_output) ==
            output_candidates.end()) {
            ROS_INFO("[OUTPUT_RULE] target output %d unavailable for %s, keeping all valid outputs",
                     target_output, reason);
            return valid_nodes;
        }
        std::vector<int> filtered;
        for (int n : valid_nodes) {
            if (!output_nodes_.count(n) || n == target_output)
                filtered.push_back(n);
        }
        ROS_INFO("[OUTPUT_RULE] forcing output node %d for %s", target_output, reason);
        return filtered;
    };

    if (committed_outputs.empty())
        return restrict_to_output(36, "first blue box");

    if (committed_outputs.size() == 1 && committed_outputs.count(36))
        return restrict_to_output(37, "second blue after first blue went to 36");

    ROS_INFO("[OUTPUT_RULE] no special blue-output rule applies, keeping all valid outputs");
    return valid_nodes;
}

// =====================================================================
// Pickup priority
// =====================================================================

int MultiPlannerNode::getBoxTypeAtNode(int node) const
{
    auto it = planner_->factory.index_of.find(node);
    if (it == planner_->factory.index_of.end()) return -99;
    return boxes_[it->second];
}

int MultiPlannerNode::getRobotActiveTaskBoxType(const std::string& robot_id) const
{
    const auto& r = robots_.at(robot_id);
    if (r.box != EMPTY) return r.box;
    if (r.reserved_pickup_node >= 0)
        return getBoxTypeAtNode(r.reserved_pickup_node);
    return -99;
}

MultiPlannerNode::PickupPriority
MultiPlannerNode::getPickupPriorityMode(const std::string& robot_id) const
{
    std::string other = (robot_id == "r1") ? "r2" : "r1";
    int other_color = getRobotActiveTaskBoxType(other);

    if (pickup_plan_count_ == 1 && other_color >= 0) {
        if (other_color == TYPE_B) {
            ROS_INFO("[%s] 2nd pickup: first is GREEN, prioritizing NON-GREEN", robot_id.c_str());
            return {true, "avoid_exact", TYPE_B};
        } else {
            ROS_INFO("[%s] 2nd pickup: first is NON-GREEN, prioritizing GREEN", robot_id.c_str());
            return {true, "prefer_exact", TYPE_B};
        }
    }

    if (other_color >= 0) {
        ROS_INFO("[%s] other robot color is %d, avoiding same", robot_id.c_str(), other_color);
        return {true, "avoid_exact", other_color};
    }

    return {};
}

std::vector<std::vector<int>> MultiPlannerNode::splitPickupCandidatesByPriority(
    const std::string& robot_id, const std::vector<int>& valid_nodes) const
{
    auto mode = getPickupPriorityMode(robot_id);
    if (!mode.active) return {valid_nodes};

    std::vector<int> exact, other, unknown;
    for (int node : valid_nodes) {
        int bt = getBoxTypeAtNode(node);
        if (bt == -99)                    unknown.push_back(node);
        else if (bt == mode.target_box_type) exact.push_back(node);
        else                              other.push_back(node);
    }

    if (mode.mode == "prefer_exact") {
        std::vector<int> fallback;
        fallback.insert(fallback.end(), other.begin(), other.end());
        fallback.insert(fallback.end(), unknown.begin(), unknown.end());
        if (!exact.empty()) {
            if (fallback.empty()) return {exact};
            return {exact, fallback};
        }
        return {valid_nodes};
    }

    if (mode.mode == "avoid_exact") {
        std::vector<int> preferred;
        preferred.insert(preferred.end(), other.begin(), other.end());
        preferred.insert(preferred.end(), unknown.begin(), unknown.end());
        if (!preferred.empty()) {
            if (exact.empty()) return {preferred};
            return {preferred, exact};
        }
        return {valid_nodes};
    }

    return {valid_nodes};
}

// =====================================================================
// Machine capacity check
// =====================================================================

bool MultiPlannerNode::canPickupBoxForMachine(
    const std::string& robot_id, int pickup_node,
    int source_box_type,
    const std::vector<int>& machine_inputs,
    const std::vector<int>& machine_outputs,
    const std::string& label) const
{
    auto it = planner_->factory.index_of.find(pickup_node);
    if (it == planner_->factory.index_of.end()) return false;

    int box_type = boxes_[it->second];
    if (box_type != source_box_type) return true;

    int available_lines = 0;
    for (int k = 0; k < static_cast<int>(machine_inputs.size()); ++k) {
        int i_input  = planner_->factory.index_of.at(machine_inputs[k]);
        int i_output = planner_->factory.index_of.at(machine_outputs[k]);
        if (boxes_[i_input] == EMPTY && boxes_[i_output] == EMPTY)
            ++available_lines;
    }

    int same_in_transit = 0;
    for (auto& [other_id, other] : robots_) {
        if (other_id == robot_id) continue;
        if (other.box == source_box_type) {
            ++same_in_transit;
            continue;
        }
        if (other.reserved_pickup_node >= 0) {
            auto pit = planner_->factory.index_of.find(other.reserved_pickup_node);
            if (pit != planner_->factory.index_of.end()) {
                if (boxes_[pit->second] == source_box_type)
                    ++same_in_transit;
            }
        }
    }

    ROS_INFO("[%s] %s pickup check at node %d: available_lines=%d, same_in_transit=%d",
             robot_id.c_str(), label.c_str(), pickup_node, available_lines, same_in_transit);

    return available_lines > same_in_transit;
}

bool MultiPlannerNode::isDirect11To27ReservedConsecutively() const
{
    for (const auto& [robot_id, r] : robots_) {
        for (int i = 0; i < static_cast<int>(r.path.size()) - 1; ++i) {
            int a = r.path[i];
            int b = r.path[i + 1];

            if ((a == 11 && b == 27) || (a == 27 && b == 11)) {
                return true;
            }
        }
    }
    return false;
}

bool MultiPlannerNode::isNodeReservedOrBlockedIndirectly(int node) const
{
    if (reserved_nodes_.count(node)) return true;

    if (node == 12 && reserved_nodes_.count(13)) return true;
    if (node == 26 && reserved_nodes_.count(25)) return true;
    if (node == 29 && reserved_nodes_.count(30)) return true;

    return false;
}

bool MultiPlannerNode::isDirect11To27GloballyForbidden() const
{
    // Se 12 ou 26 estiverem reservados por qualquer robô,
    // então o troço direto 11 <-> 27 fica proibido globalmente.
    if (isNodeReservedOrBlockedIndirectly(12) ||
        isNodeReservedOrBlockedIndirectly(26)) {
        ROS_INFO("Global rule active: direct edge 11<->27 forbidden because 12/26 are reserved or indirectly blocked by 13/25");
        return true;
    }

    return false;
}
