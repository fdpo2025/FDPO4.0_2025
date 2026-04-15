#include "plan_handler_node.h"
#include <algorithm>
#include <xmlrpcpp/XmlRpcValue.h>
#include <cmath>

#include <std_msgs/UInt32.h>

namespace {
bool isPlannedPathMsgSentinel(int32_t v) {
  return v <= -900000000 && v >= -900000000 - 255;
}
uint32_t decodeMsgTargetRobotId(int32_t v) {
  const long long decoded = -(static_cast<long long>(v) + 900000000LL);
  if (decoded < 0 || decoded > 255) return 255u;
  return static_cast<uint32_t>(decoded);
}
}  // namespace

PlanHandlerNode::PlanHandlerNode(ros::NodeHandle& nh_) : nh(nh_)
{
    nh.param("planned_paths_topic", planned_paths_topic, std::string("/planned_paths"));
    nh.param("queue_size", queue_size, 100);

    ros::NodeHandle nh_fl("follow_line/plan_handler");
    nh_fl.param("vel_lin_nom_warehouse_process", fl_.vel_lin_nom_warehouse_process, 0.025);
    nh_fl.param("vel_lin_nom_warehouse_pick", fl_.vel_lin_nom_warehouse_pick, 0.06);
    nh_fl.param("vel_lin_nom_warehouse_other", fl_.vel_lin_nom_warehouse_other, 0.06);
    nh_fl.param("line_switch_before_pick", fl_.line_switch_before_pick, 0.95);
    nh_fl.param("line_switch_plan_stack_before_pick", fl_.line_switch_plan_stack_before_pick, 1.0);
    nh_fl.param("line_switch_drop_process", fl_.line_switch_drop_process, 0.95);
    nh_fl.param("line_switch_drop_other", fl_.line_switch_drop_other, 0.60);
    nh_fl.param("line_switch_after_warehouse_process", fl_.line_switch_after_warehouse_process, 1.0);
    nh_fl.param("line_switch_after_warehouse", fl_.line_switch_after_warehouse, 0.7);
    nh_fl.param("line_switch_normal", fl_.line_switch_normal, 0.75);
    {
        double legacy_after = 0.1;
        nh_fl.param("vel_lin_nom_after_warehouse", legacy_after, 0.1);
        nh_fl.param("vel_lin_nom_backwards_after_warehouse", fl_.vel_lin_nom_backwards_after_warehouse, legacy_after);
    }
    nh_fl.param("vel_lin_nom_normal", fl_.vel_lin_nom_normal, -1.0);

    ROS_INFO("PlanHandlerNode: follow_line params from /follow_line/plan_handler (follow_line_parameters.yaml)");

    // ========================================================================
    // EXTRAIR COORDENADAS DO YAML (formato mapa: id -> [x, y])
    // ========================================================================
    XmlRpc::XmlRpcValue coords_map;
    if (nh.getParam("factory_coords", coords_map)) {
        if (coords_map.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            for (auto it = coords_map.begin(); it != coords_map.end(); ++it) {
                int id = std::stoi(it->first);

                if (it->second.getType() != XmlRpc::XmlRpcValue::TypeArray || it->second.size() < 2) {
                    ROS_WARN("PlanHandlerNode: Invalid coordinate entry for node %d", id);
                    continue;
                }

                Pose p;
                p.x = static_cast<double>(it->second[0]);
                p.y = static_cast<double>(it->second[1]);
                factory_coordinates[id] = p;
            }

            ROS_INFO("PlanHandlerNode: Loaded %zu coordinates from YAML", factory_coordinates.size());
        } else {
            ROS_ERROR("PlanHandlerNode: Parameter 'factory_coords' is not a struct/map");
        }
    } else {
        ROS_ERROR("PlanHandlerNode: Failed to get 'factory_coords' from parameter server!");
    }

    // ========================================================================
    // WAREHOUSE COORDINATES: hardcoded (dps mudar para extrair de parametros)
    // ========================================================================
    warehouse_coordinates.resize(16);

    // Input
    warehouse_coordinates[0] = factory_coordinates[0];
    warehouse_coordinates[1] = factory_coordinates[1];
    warehouse_coordinates[2] = factory_coordinates[2];
    warehouse_coordinates[3] = factory_coordinates[3];

    // Process
    warehouse_coordinates[4]  = factory_coordinates[13];
    warehouse_coordinates[5]  = factory_coordinates[14];
    warehouse_coordinates[6]  = factory_coordinates[17];
    warehouse_coordinates[7]  = factory_coordinates[18];
    warehouse_coordinates[8]  = factory_coordinates[20];
    warehouse_coordinates[9]  = factory_coordinates[21];
    warehouse_coordinates[10] = factory_coordinates[24];
    warehouse_coordinates[11] = factory_coordinates[25];

    // Output
    warehouse_coordinates[12] = factory_coordinates[35];
    warehouse_coordinates[13] = factory_coordinates[36];
    warehouse_coordinates[14] = factory_coordinates[37];
    warehouse_coordinates[15] = factory_coordinates[38];

    // ========================================================================
    // WAREHOUSE FLAGS
    // ========================================================================
    for (int id : {0, 1, 2, 3}) {
        is_warehouse_coordinate.insert(id);
        is_input_warehouse.insert(id);
    }

    for (int id : {13, 14, 17, 18, 20, 21, 24, 25}) {
        is_warehouse_coordinate.insert(id);
        is_process_warehouse.insert(id);
    }

    for (int id : {35, 36, 37, 38}) {
        is_warehouse_coordinate.insert(id);
        is_output_warehouse.insert(id);
    }

    // State variables
    fe_warehouse_coordinate = false;
    has_box = false;
    is_last_warehouse = false;
    is_current_warehouse = false;
    was_last_warehouse_process = false;
    last_pick_box_state = false;

    // ROS subscribers e publishers
    plannedPathsSub = nh.subscribe(planned_paths_topic, queue_size, &PlanHandlerNode::plannedPathsCallback, this);
    std::string consumed_topic;
    nh.param<std::string>("nav_plan_waypoint_consumed_topic", consumed_topic,
                          std::string("/nav_plan_waypoint_consumed"));
    navPlanWaypointConsumedSub_ =
        nh.subscribe(consumed_topic, 50, &PlanHandlerNode::navPlanWaypointConsumedCallback, this);
    navCompletionFeedbackSub = nh.subscribe("/nav_completion_feedback", 10, &PlanHandlerNode::navCompletionFeedbackCallback, this);
    navPlanPub = nh.advertise<plan_handler::NavPlan>("/nav_plan", 10);
    pickBoxPub = nh.advertise<std_msgs::Bool>("/pick_box", 10);
    targetIdSendPub = nh.advertise<std_msgs::UInt32>("/target_id_send", 10);
    stopWaitingSendPub = nh.advertise<std_msgs::Bool>("/stop_waiting_send", 10);

    ROS_INFO("PlanHandlerNode subscribing to: %s (queue_size: %d)", planned_paths_topic.c_str(), queue_size);
    ROS_INFO("PlanHandlerNode subscribing to: %s (deferred MSG via nav consumption)", consumed_topic.c_str());
    ROS_INFO("PlanHandlerNode subscribing to: /nav_completion_feedback");
    ROS_INFO("PlanHandlerNode publishing to: /nav_plan");
    ROS_INFO("PlanHandlerNode publishing to: /pick_box");
    ROS_INFO("PlanHandlerNode publishing to: /target_id_send and /stop_waiting_send (custom_planner MSG_* sentinels)");
    ROS_INFO("PlanHandlerNode instance created");
}

void PlanHandlerNode::publishRadioStopWaitingPulse(uint32_t target_robot_id) {
    std_msgs::UInt32 tid;
    tid.data = std::min(target_robot_id, 255u);
    targetIdSendPub.publish(tid);
    std_msgs::Bool sw;
    sw.data = true;
    stopWaitingSendPub.publish(sw);
    ROS_INFO("PlanHandlerNode: MSG sentinel → /target_id_send=%u, /stop_waiting_send=true", tid.data);

    if (stopWaitingResetTimerValid) {
        stopWaitingResetTimer.stop();
    }
    stopWaitingResetTimer =
        nh.createTimer(ros::Duration(0.12),
                       [this](const ros::TimerEvent&) {
                           std_msgs::Bool off;
                           off.data = false;
                           stopWaitingSendPub.publish(off);
                           stopWaitingResetTimer.stop();
                       },
                       true, true);
    stopWaitingResetTimerValid = true;
}

void PlanHandlerNode::navPlanWaypointConsumedCallback(const std_msgs::UInt32MultiArray::ConstPtr& msg) {
    if (msg->data.size() < 2) return;
    const uint32_t seq = msg->data[0];
    const int consumed_index = static_cast<int>(msg->data[1]);
    for (auto it = pending_msg_pulses_.begin(); it != pending_msg_pulses_.end();) {
        if (it->nav_plan_seq == seq && it->trigger_plan_index == consumed_index) {
            publishRadioStopWaitingPulse(it->target_robot_id);
            it = pending_msg_pulses_.erase(it);
        } else {
            ++it;
        }
    }
}

void PlanHandlerNode::plannedPathsCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    // Do not clear pending_msg_pulses_ here: a new /planned_paths may arrive (e.g. next segment
    // when this_current_pose hits the last node) before navigation_controller publishes consumption
    // of the previous NavPlan's last plan_index — that would drop the deferred MSG pulse.

    std::vector<ControllerPoint> control_points;
    bool is_first_node = true;

    std::vector<uint32_t> msg_targets;
    size_t idx = 0;
    while (idx < msg->data.size() && isPlannedPathMsgSentinel(msg->data[idx])) {
        msg_targets.push_back(decodeMsgTargetRobotId(msg->data[idx]));
        ++idx;
    }

    for (; idx < msg->data.size(); ++idx) {
        const int32_t value = msg->data[idx];

        auto coord_it = factory_coordinates.find(value);
        if (coord_it == factory_coordinates.end()) {
            ROS_WARN("PlanHandlerNode: Unknown node ID %d, skipping", value);
            continue;
        }

        int resolved_id = (value >= 100) ? value - 100 : value;

        ControllerPoint point;

        // =========================
        // NODE_ID
        // =========================
        point.node_id = value;

        point.x = coord_it->second.x;
        point.y = coord_it->second.y;

        is_last_warehouse = is_current_warehouse;
        is_current_warehouse = is_warehouse_coordinate.count(resolved_id);
        fe_warehouse_coordinate = is_last_warehouse && !is_current_warehouse;

        ROS_INFO("PlanHandlerNode: Processing node %d (resolved %d) - is_first=%d, is_warehouse=%d, is_input=%d, is_output=%d, has_box=%d",
                 value, resolved_id, is_first_node ? 1 : 0, is_current_warehouse ? 1 : 0,
                 is_input_warehouse.count(resolved_id) ? 1 : 0,
                 is_output_warehouse.count(resolved_id) ? 1 : 0,
                 has_box ? 1 : 0);

        if (is_current_warehouse) {
            point.is_warehouse = true;
            point.is_process_warehouse = is_process_warehouse.count(resolved_id);
            was_last_warehouse_process = is_process_warehouse.count(resolved_id);
            point.line_switch_ratio = 1.0;
            point.backwards = false;

            // Se é o primeiro nó do caminho e é warehouse, não adicionar
            if (is_first_node) {
                ROS_INFO("PlanHandlerNode: First node is warehouse (ID %d), skipping entirely (not added to plan_stack)", value);
                is_first_node = false;
                continue;
            }
            // Input warehouses: sempre pick
            else if (is_input_warehouse.count(resolved_id)) {
                point.pick_box = true;
                has_box = true;
                point.should_pub = true;
                ROS_INFO("PlanHandlerNode: Input warehouse (ID %d) -> PICK", value);
            }
            // Output warehouses: sempre drop
            else if (is_output_warehouse.count(resolved_id)) {
                point.pick_box = false;
                has_box = false;
                point.should_pub = true;
                ROS_INFO("PlanHandlerNode: Output warehouse (ID %d) -> DROP", value);
            }
            // Process warehouses: toggle
            else {
                point.pick_box = !has_box;
                has_box = !has_box;
                point.should_pub = true;
                ROS_INFO("PlanHandlerNode: Process warehouse (ID %d) -> %s", value, point.pick_box ? "PICK" : "DROP");
            }

            if (point.pick_box) {
                if (!control_points.empty()) {
                    control_points.back().line_switch_ratio = fl_.line_switch_before_pick;
                    ROS_INFO("PlanHandlerNode: Set line_switch_ratio for previous point (before pick warehouse)");
                }
                if (!plan_stack.empty()) {
                    plan_stack.back().line_switch_ratio = fl_.line_switch_plan_stack_before_pick;
                }
            } else {
                double drop_switch_ratio = is_process_warehouse.count(resolved_id)
                                               ? fl_.line_switch_drop_process
                                               : fl_.line_switch_drop_other;

                if (!control_points.empty()) {
                    control_points.back().line_switch_ratio = drop_switch_ratio;
                    ROS_INFO("PlanHandlerNode: Set line_switch_ratio=%.2f for previous point (before drop warehouse, process=%d)",
                             drop_switch_ratio, is_process_warehouse.count(resolved_id) ? 1 : 0);
                }
                if (!plan_stack.empty()) {
                    plan_stack.back().line_switch_ratio = drop_switch_ratio;
                }
            }

            if (point.pick_box) {
                point.vel_lin_nom = is_process_warehouse.count(resolved_id)
                                        ? fl_.vel_lin_nom_warehouse_process
                                        : fl_.vel_lin_nom_warehouse_pick;
            } else {
                point.vel_lin_nom = is_process_warehouse.count(resolved_id)
                                        ? fl_.vel_lin_nom_warehouse_process
                                        : fl_.vel_lin_nom_warehouse_other;
            }

        } else {
            point.is_warehouse = false;
            point.is_process_warehouse = false;

            if (fe_warehouse_coordinate) {
                point.line_switch_ratio = was_last_warehouse_process
                                              ? fl_.line_switch_after_warehouse_process
                                              : fl_.line_switch_after_warehouse;
            } else {
                point.line_switch_ratio = fl_.line_switch_normal;
            }

            point.backwards = fe_warehouse_coordinate;
            point.vel_lin_nom = fe_warehouse_coordinate
                                    ? fl_.vel_lin_nom_backwards_after_warehouse
                                    : fl_.vel_lin_nom_normal;
            point.should_pub = false;
        }

        control_points.push_back(point);
        plan_stack.push_back(point);

        ROS_INFO("PlanHandlerNode: Added point - node_id=%d, x=%.3f, y=%.3f, line_switch_ratio=%.2f, vel_lin_nom=%.2f, backwards=%d",
                 point.node_id, point.x, point.y, point.line_switch_ratio, point.vel_lin_nom, point.backwards ? 1 : 0);

        is_first_node = false;
    }

    ROS_INFO("PlanHandlerNode: Received new path with %zu waypoints. Stack size: %zu", control_points.size(), plan_stack.size());

    if (!msg_targets.empty() && control_points.empty()) {
        ROS_WARN(
            "PlanHandlerNode: MSG sentinel(s) on /planned_paths but no NavPlan points after processing; "
            "deferred radio pulse(s) skipped");
    }

    if (!control_points.empty()) {
        plan_handler::NavPlan nav_plan;
        nav_plan.header.stamp = ros::Time::now();
        nav_plan.header.frame_id = "map";
        const uint32_t seq = ++nav_plan_publish_seq_;
        nav_plan.header.seq = static_cast<uint32_t>(seq);

        const int trigger_plan_index = static_cast<int>(control_points.size()) - 1;
        for (uint32_t tid : msg_targets) {
            PendingMsgPulse p;
            p.target_robot_id = tid;
            p.trigger_plan_index = trigger_plan_index;
            p.nav_plan_seq = seq;
            pending_msg_pulses_.push_back(p);
            ROS_INFO(
                "PlanHandlerNode: deferred MSG pulse target_robot=%u until navigation consumes plan_index=%d "
                "(0-based, %zu NavPlan points, nav_plan seq=%u)",
                tid, trigger_plan_index, control_points.size(), seq);
        }

        nav_plan.points.resize(control_points.size());

        for (size_t i = 0; i < control_points.size(); ++i) {
            const ControllerPoint& cp = control_points[i];

            nav_plan.points[i].x = cp.x;
            nav_plan.points[i].y = cp.y;
            nav_plan.points[i].line_switch_ratio = cp.line_switch_ratio;
            nav_plan.points[i].vel_lin_nom = cp.vel_lin_nom;
            nav_plan.points[i].backwards = cp.backwards;
            nav_plan.points[i].pick_box = cp.pick_box;
            nav_plan.points[i].is_warehouse = cp.is_warehouse;
            nav_plan.points[i].is_process_warehouse = cp.is_process_warehouse;
            nav_plan.points[i].node_id = cp.node_id;
        }

        navPlanPub.publish(nav_plan);
        ROS_INFO("PlanHandlerNode: Published %zu control points to /nav_plan", control_points.size());
    }
}

void PlanHandlerNode::navCompletionFeedbackCallback(const plan_handler::CompletionFeedback::ConstPtr& msg)
{
    ROS_INFO("PlanHandlerNode: Received completion feedback for point (%.3f, %.3f)", msg->x, msg->y);

    const double tolerance = 0.01; // 1 cm

    bool found = false;
    for (auto it = plan_stack.begin(); it != plan_stack.end(); ++it) {
        double dx = it->x - msg->x;
        double dy = it->y - msg->y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < tolerance) {
            ControllerPoint removed_point = *it;
            plan_stack.erase(it);

            // Publicar sempre em /pick_box ao consumir um warehouse (pick ou drop), para o tópico
            // acompanhar a ação corrente (ex.: drop process após pick input sem depender de "mudança" vs last).
            if (removed_point.should_pub) {
                std_msgs::Bool pick_box_msg;
                pick_box_msg.data = removed_point.pick_box;
                pickBoxPub.publish(pick_box_msg);
                last_pick_box_state = removed_point.pick_box;

                ROS_INFO("PlanHandlerNode: Removed point node_id=%d (%.3f, %.3f) from plan_stack. Published pick_box=%d. Remaining points: %zu",
                         removed_point.node_id, removed_point.x, removed_point.y,
                         pick_box_msg.data ? 1 : 0, plan_stack.size());
            } else {
                ROS_INFO("PlanHandlerNode: Removed point node_id=%d (%.3f, %.3f) from plan_stack (should_pub=0). Remaining points: %zu",
                         removed_point.node_id, removed_point.x, removed_point.y, plan_stack.size());
            }

            found = true;
            break;
        }
    }

    if (!found) {
        ROS_WARN("PlanHandlerNode: Could not find point (%.3f, %.3f) in plan_stack to remove", msg->x, msg->y);
    }
}