#include "plan_handler_node.h"
#include <algorithm>
#include <xmlrpcpp/XmlRpcValue.h>

PlanHandlerNode::PlanHandlerNode(ros::NodeHandle& nh_): nh(nh_)
{
    nh.param("planned_paths_topic", planned_paths_topic, std::string("/planned_paths"));
    nh.param("queue_size", queue_size, 100); 

    // ========================================================================
    // EXTRAIR COORDENADAS DO YAML (formato mapa: id -> [x, y])
    // ========================================================================
    XmlRpc::XmlRpcValue coords_map;
    if (nh.getParam("factory_coords", coords_map)) {
        if (coords_map.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            for (auto it = coords_map.begin(); it != coords_map.end(); ++it) {
                int id = std::stoi(it->first);
                Pose p;
                p.x = static_cast<double>(it->second[0]);
                p.y = static_cast<double>(it->second[1]);
                factory_coordinates[id] = p;
            }
            ROS_INFO("PlanHandlerNode: Loaded %zu coordinates from YAML", factory_coordinates.size());
        }
    } else {
        ROS_ERROR("PlanHandlerNode: Failed to get 'factory_coords' from parameter server!");
    }

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
    fe_warehouse_coordinate = has_box = is_last_warehouse = is_current_warehouse = false;
    last_pick_box_state = false;  // Estado inicial do pick_box

    // ROS subscribers e publishers
    plannedPathsSub = nh.subscribe(planned_paths_topic, queue_size, &PlanHandlerNode::plannedPathsCallback, this);
    navCompletionFeedbackSub = nh.subscribe("/nav_completion_feedback", 10, &PlanHandlerNode::navCompletionFeedbackCallback, this);
    navPlanPub = nh.advertise<plan_handler::NavPlan>("/nav_plan", 10);
    pickBoxPub = nh.advertise<std_msgs::Bool>("/pick_box", 10);
    
    ROS_INFO("PlanHandlerNode subscribing to: %s (queue_size: %d)", planned_paths_topic.c_str(), queue_size);
    ROS_INFO("PlanHandlerNode subscribing to: /nav_completion_feedback");
    ROS_INFO("PlanHandlerNode publishing to: /nav_plan");
    ROS_INFO("PlanHandlerNode publishing to: /pick_box");
    ROS_INFO("PlanHandlerNode instance created");
}

void PlanHandlerNode::plannedPathsCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    std::vector<ControllerPoint> control_points;
    bool is_first_node = true;  // Flag para identificar o primeiro nó do caminho

    for (const auto& value : msg->data) {

        auto coord_it = factory_coordinates.find(value);
        if (coord_it == factory_coordinates.end()) {
            ROS_WARN("PlanHandlerNode: Unknown node ID %d, skipping", value);
            continue;
        }

        int resolved_id = (value >= 100) ? value - 100 : value;

        ControllerPoint point;

        point.node_id = value;
        point.x = coord_it->second.x;
        point.y = coord_it->second.y;

        is_last_warehouse = is_current_warehouse;
        is_current_warehouse = is_warehouse_coordinate.count(resolved_id);
        fe_warehouse_coordinate = is_last_warehouse && !is_current_warehouse;

        ROS_INFO("PlanHandlerNode: Processing node %d (resolved %d) - is_first=%d, is_warehouse=%d, is_input=%d, is_output=%d, has_box=%d",
                value, resolved_id, is_first_node ? 1 : 0, is_current_warehouse ? 1 : 0, 
                is_input_warehouse.count(resolved_id) ? 1 : 0, is_output_warehouse.count(resolved_id) ? 1 : 0, has_box ? 1 : 0);

        if(is_current_warehouse) {
            point.is_warehouse = true;
            was_last_warehouse_process = is_process_warehouse.count(resolved_id);
            point.line_switch_ratio = 1.0;
            point.backwards = false;
            point.vel_lin_nom = is_process_warehouse.count(resolved_id) ? 0.025 : 0.06;

            if (is_first_node) {
                ROS_INFO("PlanHandlerNode: First node is warehouse (ID %d), skipping entirely (not added to plan_stack)", value);
                is_first_node = false;
                continue;
            } 
            else if (is_input_warehouse.count(resolved_id)) {
                point.pick_box = true;
                has_box = true;
                point.should_pub = true;
                ROS_INFO("PlanHandlerNode: Input warehouse (ID %d) -> PICK", value);
            }
            else if (is_output_warehouse.count(resolved_id)) {
                point.pick_box = false;
                has_box = false;
                point.should_pub = true;
                ROS_INFO("PlanHandlerNode: Output warehouse (ID %d) -> DROP", value);
            }
            // Process warehouses: toggle (comportamento original)
            else {
                point.pick_box = !has_box; 
                has_box = !has_box;
                point.should_pub = true;
                ROS_INFO("PlanHandlerNode: Process warehouse (ID %d) -> %s", value, point.pick_box ? "PICK" : "DROP");
            }
            
            // Se é warehouse de pick (pick_box = true), garantir que o ponto anterior tenha line_switch_ratio = 1.0
            // (para completar 100% da linha antes de chegar à warehouse de pick)
            if (point.pick_box) {
                if (!control_points.empty()) {
                    control_points.back().line_switch_ratio = 0.95;
                    ROS_INFO("PlanHandlerNode: Set line_switch_ratio=1.0 for previous point (before pick warehouse)");
                }
                // Também atualizar no plan_stack se não estiver vazio
                if (!plan_stack.empty()) {
                    plan_stack.back().line_switch_ratio = 1.0;
                }
            } else {
                // Se é warehouse de drop (pick_box = false)
                // Para warehouses de process: usar line_switch_ratio = 0.95
                // Para outras warehouses: usar line_switch_ratio = 0.60
                double drop_switch_ratio = is_process_warehouse.count(resolved_id) ? 0.95 : 0.60;
                
                if (!control_points.empty()) {
                    control_points.back().line_switch_ratio = drop_switch_ratio;
                    ROS_INFO("PlanHandlerNode: Set line_switch_ratio=%.2f for previous point (before drop warehouse, process=%d)", 
                            drop_switch_ratio, is_process_warehouse.count(resolved_id) ? 1 : 0);
                }
                // Também atualizar no plan_stack se não estiver vazio
                if (!plan_stack.empty()) {
                    plan_stack.back().line_switch_ratio = drop_switch_ratio;
                }
            }

        } else {
            point.is_warehouse = false;

            //point.line_switch_ratio = 0.8 * fe_warehouse_coordinate + 0.75 * !fe_warehouse_coordinate; // complete line if backwards
            
            if (fe_warehouse_coordinate) {
                point.line_switch_ratio = was_last_warehouse_process ? 1.0 : 0.7;
            } else {
                point.line_switch_ratio = 0.75;
            }            

            point.backwards = fe_warehouse_coordinate;  // backwards ao sair de qualquer warehouse (pick ou drop)
            point.vel_lin_nom = 0.1 * fe_warehouse_coordinate + 0.3 * !fe_warehouse_coordinate;
            point.should_pub = false;       
        }

        // add current new path
        control_points.push_back(point);
        // add to total path
        plan_stack.push_back(point);
        
        ROS_INFO("PlanHandlerNode: Added point - node_id=%d, x=%.3f, y=%.3f, line_switch_ratio=%.2f, vel_lin_nom=%.2f, backwards=%d",
                point.node_id, point.x, point.y, point.line_switch_ratio, point.vel_lin_nom, point.backwards ? 1 : 0);
        
        // Após processar o primeiro nó, marcar como false
        is_first_node = false;
    }

    ROS_INFO("PlanHandlerNode: Received new path with %zu waypoints. Stack size: %zu", control_points.size(), plan_stack.size());

    // Publish
    if (!control_points.empty()) {
        plan_handler::NavPlan nav_plan;
        nav_plan.header.stamp = ros::Time::now();
        nav_plan.header.frame_id = "map"; 
        
        nav_plan.points.resize(control_points.size());
        
        for (size_t i = 0; i < control_points.size(); ++i) {
            const ControllerPoint& cp = control_points[i];
            
            // Copiar todos os campos diretamente
            nav_plan.points[i].x = cp.x;
            nav_plan.points[i].y = cp.y;
            nav_plan.points[i].line_switch_ratio = cp.line_switch_ratio;
            nav_plan.points[i].vel_lin_nom = cp.vel_lin_nom;
            nav_plan.points[i].backwards = cp.backwards;
            nav_plan.points[i].pick_box = cp.pick_box;
            nav_plan.points[i].is_warehouse = cp.is_warehouse;

            // =========================
            // NODE_ID
            // =========================
            nav_plan.points[i].node_id = cp.node_id;
        }
        
        navPlanPub.publish(nav_plan);
        ROS_INFO("PlanHandlerNode: Published %zu control points to /nav_plan", control_points.size());
    }
}

void PlanHandlerNode::navCompletionFeedbackCallback(const plan_handler::CompletionFeedback::ConstPtr& msg)
{
    ROS_INFO("PlanHandlerNode: Received completion feedback for point (%.3f, %.3f)", msg->x, msg->y);
    
    // Tolerância para comparação de coordenadas (devido a erros de ponto flutuante)
    const double tolerance = 0.01; // 1cm
    
    // Procurar e remover o primeiro ponto no plan_stack com essas coordenadas
    bool found = false;
    for (auto it = plan_stack.begin(); it != plan_stack.end(); ++it) {
        double dx = it->x - msg->x;
        double dy = it->y - msg->y;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        if (dist < tolerance) {
            // Encontrou o ponto - remover e publicar pick_box apenas se o estado mudou
            ControllerPoint removed_point = *it;
            plan_stack.erase(it);
            
            // Publicar no tópico /pick_box apenas se o estado mudou
            if (removed_point.pick_box != last_pick_box_state && removed_point.should_pub) {
                std_msgs::Bool pick_box_msg;
                pick_box_msg.data = removed_point.pick_box;
                pickBoxPub.publish(pick_box_msg);
                last_pick_box_state = removed_point.pick_box;
                
                ROS_INFO("PlanHandlerNode: Removed point node_id=%d (%.3f, %.3f) from plan_stack. Published pick_box=%d (state changed). Remaining points: %zu", 
                        removed_point.node_id, removed_point.x, removed_point.y, pick_box_msg.data ? 1 : 0, plan_stack.size());
            } else {
                ROS_INFO("PlanHandlerNode: Removed point node_id=%d (%.3f, %.3f) from plan_stack. pick_box=%d (no state change). Remaining points: %zu", 
                        removed_point.node_id, removed_point.x, removed_point.y, removed_point.pick_box ? 1 : 0, plan_stack.size());
            }
            
            found = true;
            break;
        }
    }
    
    if (!found) {
        ROS_WARN("PlanHandlerNode: Could not find point (%.3f, %.3f) in plan_stack to remove", msg->x, msg->y);
    }
}