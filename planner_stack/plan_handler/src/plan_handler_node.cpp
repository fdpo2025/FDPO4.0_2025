#include "plan_handler_node.h"
#include <algorithm>
#include <xmlrpcpp/XmlRpcValue.h>

PlanHandlerNode::PlanHandlerNode(ros::NodeHandle& nh_): nh(nh_)
{
    nh.param("planned_paths_topic", planned_paths_topic, std::string("/planned_paths"));
    nh.param("queue_size", queue_size, 100); 

    // Inicializar vetores
    factory_coordinates.resize(39);
    warehouse_coordinates.resize(16);
    is_warehouse_coordinate.resize(39, false);
    is_process_warehouse.resize(39, false);
    is_input_warehouse.resize(39, false);
    is_output_warehouse.resize(39, false);

    // ========================================================================
    // EXTRAIR COORDENADAS DO YAML
    // ========================================================================
    XmlRpc::XmlRpcValue coords_list;
    if (nh.getParam("factory_coords", coords_list)) {
        if (coords_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < coords_list.size() && i < 39; ++i) {
                factory_coordinates[i].x = static_cast<double>(coords_list[i][0]);
                factory_coordinates[i].y = static_cast<double>(coords_list[i][1]);
            }
            ROS_INFO("PlanHandlerNode: Successfully loaded %d coordinates from YAML", coords_list.size());
        }
    } else {
        ROS_ERROR("PlanHandlerNode: Failed to get 'factory_coords' from parameter server!");
    }

    // ========================================================================
    // WAREHOUSE COORDINATES: hardcoded (dps mudar para extrair de parametrois)
    // ========================================================================
    // Input 
    warehouse_coordinates[0] = factory_coordinates[0];  // -0.695, 0.46
    warehouse_coordinates[1] = factory_coordinates[1];  // -0.545, 0.46
    warehouse_coordinates[2] = factory_coordinates[2];  // -0.395, 0.46
    warehouse_coordinates[3] = factory_coordinates[3];  // -0.245, 0.46
    
    // Process
    warehouse_coordinates[4] = factory_coordinates[13];  // 0.227, 0.15
    warehouse_coordinates[5] = factory_coordinates[14];  // 0.468, 0.15
    warehouse_coordinates[6] = factory_coordinates[17];  // -0.468, 0.0
    warehouse_coordinates[7] = factory_coordinates[18];  // -0.227, 0.0
    warehouse_coordinates[8] = factory_coordinates[20];  // 0.227, 0.0
    warehouse_coordinates[9] = factory_coordinates[21];  // 0.468, 0.0
    warehouse_coordinates[10] = factory_coordinates[24]; // -0.468, -0.15
    warehouse_coordinates[11] = factory_coordinates[25]; // -0.227, -0.15
    
    // Output
    warehouse_coordinates[12] = factory_coordinates[35]; // 0.245, -0.46
    warehouse_coordinates[13] = factory_coordinates[36]; // 0.395, -0.46
    warehouse_coordinates[14] = factory_coordinates[37]; // 0.545, -0.46
    warehouse_coordinates[15] = factory_coordinates[38]; // 0.695, -0.46

    // ========================================================================
    // IS_WAREHOUSE_COORDINATE
    // ========================================================================
    // Input warehouses (IDs 0-3) - SEMPRE pick
    is_warehouse_coordinate[0] = true;
    is_warehouse_coordinate[1] = true;
    is_warehouse_coordinate[2] = true;
    is_warehouse_coordinate[3] = true;
    is_input_warehouse[0] = true;
    is_input_warehouse[1] = true;
    is_input_warehouse[2] = true;
    is_input_warehouse[3] = true;
    
    // Process warehouses (IDs 13,14,17,18,20,21,24,25)
    is_warehouse_coordinate[13] = true;
    is_warehouse_coordinate[14] = true;
    is_warehouse_coordinate[17] = true;
    is_warehouse_coordinate[18] = true;
    is_warehouse_coordinate[20] = true;
    is_warehouse_coordinate[21] = true;
    is_warehouse_coordinate[24] = true;
    is_warehouse_coordinate[25] = true;
    
    // Marcar warehouses de process
    is_process_warehouse[13] = true;
    is_process_warehouse[14] = true;
    is_process_warehouse[17] = true;
    is_process_warehouse[18] = true;
    is_process_warehouse[20] = true;
    is_process_warehouse[21] = true;
    is_process_warehouse[24] = true;
    is_process_warehouse[25] = true;
    
    // Output warehouses (IDs 35-38) - SEMPRE drop
    is_warehouse_coordinate[35] = true;
    is_warehouse_coordinate[36] = true;
    is_warehouse_coordinate[37] = true;
    is_warehouse_coordinate[38] = true;
    is_output_warehouse[35] = true;
    is_output_warehouse[36] = true;
    is_output_warehouse[37] = true;
    is_output_warehouse[38] = true;

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
    
        if (value < 0 || value >= static_cast<int>(factory_coordinates.size())) {
            ROS_WARN("PlanHandlerNode: Invalid node ID %d (valid range: 0-%zu), skipping", value, factory_coordinates.size() - 1);
            continue;
        }

        ControllerPoint point;

        point.x = factory_coordinates[value].x;
        point.y = factory_coordinates[value].y;

        is_last_warehouse = is_current_warehouse;
        is_current_warehouse = is_warehouse_coordinate[value];
        fe_warehouse_coordinate = is_last_warehouse && !is_current_warehouse;

        // DEBUG: Mostrar info de cada nó
        ROS_INFO("PlanHandlerNode: Processing node %d - is_first=%d, is_warehouse=%d, is_input=%d, is_output=%d, has_box=%d",
                value, is_first_node ? 1 : 0, is_current_warehouse ? 1 : 0, 
                is_input_warehouse[value] ? 1 : 0, is_output_warehouse[value] ? 1 : 0, has_box ? 1 : 0);

        if(is_current_warehouse) {
            was_last_warehouse_process = is_process_warehouse[value];
            point.line_switch_ratio = 1.0;
            point.backwards = false;
            // Velocidade nominal: 0.03 m/s para warehouses de process, 0.05 m/s para outras
            point.vel_lin_nom = is_process_warehouse[value] ? 0.025 : 0.06;

            // Se é o primeiro nó do caminho E é uma warehouse, NÃO fazer toggle do has_box
            // (o robô já está nessa posição, é apenas o ponto de partida)
            // IMPORTANTE: Também não adicionar ao plan_stack para evitar conflitos com caminhos futuros
            if (is_first_node) {
                ROS_INFO("PlanHandlerNode: First node is warehouse (ID %d), skipping entirely (not added to plan_stack)", value);
                is_first_node = false;
                continue;  // Salta este ponto completamente - não adiciona a control_points nem plan_stack
            } 
            // Input warehouses: SEMPRE pick (has_box = true)
            else if (is_input_warehouse[value]) {
                point.pick_box = true;
                has_box = true;
                point.should_pub = true;
                ROS_INFO("PlanHandlerNode: Input warehouse (ID %d) -> PICK", value);
            }
            // Output warehouses: SEMPRE drop (has_box = false)
            else if (is_output_warehouse[value]) {
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
                double drop_switch_ratio = is_process_warehouse[value] ? 0.95 : 0.60;
                
                if (!control_points.empty()) {
                    control_points.back().line_switch_ratio = drop_switch_ratio;
                    ROS_INFO("PlanHandlerNode: Set line_switch_ratio=%.2f for previous point (before drop warehouse, process=%d)", 
                            drop_switch_ratio, is_process_warehouse[value] ? 1 : 0);
                }
                // Também atualizar no plan_stack se não estiver vazio
                if (!plan_stack.empty()) {
                    plan_stack.back().line_switch_ratio = drop_switch_ratio;
                }
            }

        } else {

            //point.line_switch_ratio = 0.8 * fe_warehouse_coordinate + 0.75 * !fe_warehouse_coordinate; // complete line if backwards
            
            if (fe_warehouse_coordinate) {


                point.line_switch_ratio = was_last_warehouse_process ? 1.0 : 0.7;


            } else {


                point.line_switch_ratio = 0.75;


            }            


        point.backwards = false;  // nunca backwards: ao sair de warehouse sai de frente
            point.vel_lin_nom = 0.3;  // velocidade normal (antes 0.1 ao sair, 0.3 forward)   
            point.should_pub = false;       

        }

        // add current new path
        control_points.push_back(point);
        // add to total path
        plan_stack.push_back(point);
        
        ROS_INFO("PlanHandlerNode: Added point - x=%.3f, y=%.3f, line_switch_ratio=%.2f, vel_lin_nom=%.2f, backwards=%d",
                point.x, point.y, point.line_switch_ratio, point.vel_lin_nom, point.backwards ? 1 : 0);
        
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
                
                ROS_INFO("PlanHandlerNode: Removed point (%.3f, %.3f) from plan_stack. Published pick_box=%d (state changed). Remaining points: %zu", 
                        removed_point.x, removed_point.y, pick_box_msg.data ? 1 : 0, plan_stack.size());
            } else {
                ROS_INFO("PlanHandlerNode: Removed point (%.3f, %.3f) from plan_stack. pick_box=%d (no state change). Remaining points: %zu", 
                        removed_point.x, removed_point.y, removed_point.pick_box ? 1 : 0, plan_stack.size());
            }
            
            found = true;
            break;
        }
    }
    
    if (!found) {
        ROS_WARN("PlanHandlerNode: Could not find point (%.3f, %.3f) in plan_stack to remove", msg->x, msg->y);
    }
}
