#include "plan_handler_node.h"
#include <algorithm>

PlanHandlerNode::PlanHandlerNode(ros::NodeHandle& nh_): nh(nh_)
{
    nh.param("planned_paths_topic", planned_paths_topic, std::string("/planned_paths"));
    nh.param("queue_size", queue_size, 100); 

    // Inicializar vetores
    factory_coordinates.resize(39);
    warehouse_coordinates.resize(16);
    is_warehouse_coordinate.resize(39, false);

    // ========================================================================
    // FACTORY COORDINATES: hardcoded (dps mudar para extrair de parametrois)
    // ========================================================================
    factory_coordinates[0] = {-0.695, 0.46};
    factory_coordinates[1] = {-0.545, 0.46};
    factory_coordinates[2] = {-0.395, 0.46};
    factory_coordinates[3] = {-0.245, 0.46};
    factory_coordinates[4] = {0.0, 0.355};
    factory_coordinates[5] = {0.227, 0.355};
    factory_coordinates[6] = {0.468, 0.355};
    factory_coordinates[7] = {0.695, 0.355};
    factory_coordinates[8] = {-0.695, 0.233};
    factory_coordinates[9] = {-0.545, 0.233};
    factory_coordinates[10] = {-0.395, 0.233};
    factory_coordinates[11] = {-0.245, 0.233};
    factory_coordinates[12] = {0.0, 0.15};
    factory_coordinates[13] = {0.227, 0.15};  
    factory_coordinates[14] = {0.468, 0.15};   
    factory_coordinates[15] = {0.695, 0.15};
    factory_coordinates[16] = {-0.695, 0.0};
    factory_coordinates[17] = {-0.468, 0.0};  
    factory_coordinates[18] = {-0.227, 0.0};  
    factory_coordinates[19] = {0.0, 0.0};
    factory_coordinates[20] = {0.227, 0.0};    
    factory_coordinates[21] = {0.468, 0.0};  
    factory_coordinates[22] = {0.695, 0.0};
    factory_coordinates[23] = {-0.695, -0.15};
    factory_coordinates[24] = {-0.468, -0.15};
    factory_coordinates[25] = {-0.227, -0.15};
    factory_coordinates[26] = {0.0, -0.15};
    factory_coordinates[27] = {0.245, -0.233};
    factory_coordinates[28] = {0.395, -0.233};
    factory_coordinates[29] = {0.545, -0.233};
    factory_coordinates[30] = {0.695, -0.233};
    factory_coordinates[31] = {-0.695, -0.355};
    factory_coordinates[32] = {-0.468, -0.355};
    factory_coordinates[33] = {-0.227, -0.355};
    factory_coordinates[34] = {0.0, -0.355};
    factory_coordinates[35] = {0.245, -0.46};
    factory_coordinates[36] = {0.395, -0.46};
    factory_coordinates[37] = {0.545, -0.46};
    factory_coordinates[38] = {0.695, -0.46};

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
    // Input warehouses (IDs 0-3)
    is_warehouse_coordinate[0] = true;
    is_warehouse_coordinate[1] = true;
    is_warehouse_coordinate[2] = true;
    is_warehouse_coordinate[3] = true;
    
    // Process warehouses (IDs 13,14,17,18,20,21,24,25)
    is_warehouse_coordinate[13] = true;
    is_warehouse_coordinate[14] = true;
    is_warehouse_coordinate[17] = true;
    is_warehouse_coordinate[18] = true;
    is_warehouse_coordinate[20] = true;
    is_warehouse_coordinate[21] = true;
    is_warehouse_coordinate[24] = true;
    is_warehouse_coordinate[25] = true;
    
    // Output warehouses (IDs 35-38)
    is_warehouse_coordinate[35] = true;
    is_warehouse_coordinate[36] = true;
    is_warehouse_coordinate[37] = true;
    is_warehouse_coordinate[38] = true;

    // State variables
    fe_warehouse_coordinate = has_box = is_last_warehouse = is_current_warehouse = false;

    // ROS subscribers e publishers
    plannedPathsSub = nh.subscribe(planned_paths_topic, queue_size, &PlanHandlerNode::plannedPathsCallback, this);
    navPlanPub = nh.advertise<plan_handler::NavPlan>("/nav_plan", 10);
    
    ROS_INFO("PlanHandlerNode subscribing to: %s (queue_size: %d)", planned_paths_topic.c_str(), queue_size);
    ROS_INFO("PlanHandlerNode publishing to: /nav_plan");
    ROS_INFO("PlanHandlerNode instance created");
}

void PlanHandlerNode::plannedPathsCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    std::vector<ControllerPoint> control_points;

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

        if(is_current_warehouse) {

            point.line_switch_ratio = 1.0;
            point.backwards = false;
            point.vel_lin_nom = 0.1;

            pick_box = !has_box; 
            has_box  = !has_box;

        } else {

            point.line_switch_ratio = 0.75;
            point.backwards = fe_warehouse_coordinate; // go backwards if its returning from a warehouse
            point.vel_lin_nom = 0.2;            

        }

        // add current new path
        control_points.push_back(point);
        // add to total path
        plan_stack.push_back(point);
        
        ROS_INFO("PlanHandlerNode: Added point - x=%.3f, y=%.3f, line_switch_ratio=%.2f, vel_lin_nom=%.2f, backwards=%d",
                 point.x, point.y, point.line_switch_ratio, point.vel_lin_nom, point.backwards ? 1 : 0);
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
        }
        
        navPlanPub.publish(nav_plan);
        ROS_INFO("PlanHandlerNode: Published %zu control points to /nav_plan", control_points.size());
    }
}
