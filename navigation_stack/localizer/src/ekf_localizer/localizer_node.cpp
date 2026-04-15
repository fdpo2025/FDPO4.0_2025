#include "ekf_localizer/localizer_node.h"

LocalizerNode::LocalizerNode(ros::NodeHandle& nh) : nh(nh), tf_buffer(ros::Duration(10.0))  {

    loadBeaconsFromParams();
    loadEKFParams();

    // Pos. inicial (configurável via parâmetros)
    double init_x, init_y, init_theta;
    nh.param("ekf_params/initial_pose/x", init_x, 0.0);
    nh.param("ekf_params/initial_pose/y", init_y, 0.0);
    nh.param("ekf_params/initial_pose/theta", init_theta, 0.0);
    
    X_state(0) = init_x;
    X_state(1) = init_y;
    X_state(2) = init_theta;
    
    ROS_INFO("[LocalizerNode] Initial pose: x=%.3f, y=%.3f, theta=%.3f rad (%.1f°)", 
             init_x, init_y, init_theta, init_theta * 180.0 / M_PI);

    nh.param("ekf_params/odom_skip_count", odom_skip_count_, 50);
    odom_skip_remaining_ = odom_skip_count_;
    ROS_INFO("[LocalizerNode] Will skip first %d /odom messages to let Pico stabilize", odom_skip_count_);

    odometry_sub = nh.subscribe("/odom", 10, &LocalizerNode::ekf_predict, this);
    beacon_sub = nh.subscribe("/beacon_estimation", 10, &LocalizerNode::ekf_update, this);
    pose_pub = nh.advertise<nav_msgs::Odometry>("/odometry/filtered", 10);

    tf_listener = std::make_unique<tf2_ros::TransformListener>(tf_buffer);
}

void LocalizerNode::loadBeaconsFromParams() {

    beacons.clear();

    XmlRpc::XmlRpcValue beaconsParams;

    if(nh.getParam("/beacon_detector_node/beacons", beaconsParams)) {

        for(int beacon_id = 0; beacon_id < static_cast<int>(beaconsParams.size()); beacon_id++) {

            Beacon beacon_temp;
            
            beacon_temp.name = static_cast<std::string>(beaconsParams[beacon_id]["name"]);
            beacon_temp.pose.x = static_cast<double>(beaconsParams[beacon_id]["x"]);
            beacon_temp.pose.y = static_cast<double>(beaconsParams[beacon_id]["y"]);

            beacons.insert({beacon_temp.name, beacon_temp});
        }

    };

    ROS_INFO("[LocalizerNode] Loaded %zu beacons:", beacons.size());
    for (const auto& kv : beacons) {
        const auto& name = kv.first;
        const auto& b    = kv.second;
        ROS_INFO("  - %s: x=%.3f, y=%.3f", name.c_str(), b.pose.x, b.pose.y);
    }

}

void LocalizerNode::loadEKFParams() {
    
    // Initial state covariance matrix P parameters
    double p_xx, p_yy, p_theta;
    nh.param("ekf_params/initial_covariance/position_x", p_xx, 0.5);
    nh.param("ekf_params/initial_covariance/position_y", p_yy, 0.5);
    nh.param("ekf_params/initial_covariance/orientation", p_theta, 0.5);
    
    P.setZero();
    P(0,0) = p_xx;
    P(1,1) = p_yy;
    P(2,2) = p_theta;
    
    // Process covariance matrix Q parameters
    // Q is 2x2 for [v, w] input noise: [σ_v², 0; 0, σ_w²]
    double q_vv, q_ww;
    nh.param("ekf_params/process_covariance/velocity_linear", q_vv, 0.0005);
    nh.param("ekf_params/process_covariance/velocity_angular", q_ww, 0.0005);
    
    Q.setZero();
    Q(0,0) = q_vv;  // Linear velocity noise variance
    Q(1,1) = q_ww;  // Angular velocity noise variance
    
    ROS_INFO("[LocalizerNode] EKF Parameters loaded:");
    ROS_INFO("  - Initial covariance P: [%.6f, %.6f, %.6f]", p_xx, p_yy, p_theta);
    ROS_INFO("  - Process covariance Q: [%.6f, %.6f] (v, w)", q_vv, q_ww);
}

double LocalizerNode::normalizeAngle(double theta) {

    while(theta > M_PI) theta -= 2.0 * M_PI;
    while (theta <= -M_PI) theta += 2.0*M_PI;

    return theta;
}


void LocalizerNode::ekf_predict(const nav_msgs::Odometry::ConstPtr& msg) {

    if (odom_skip_remaining_ > 0) {
        --odom_skip_remaining_;
        if (odom_skip_remaining_ == 0) {
            ROS_INFO("[LocalizerNode] Skipped %d /odom msgs — EKF starting now", odom_skip_count_);
        }
        return;
    }

    if (odom_stamp.isZero()) {
        odom_stamp = msg->header.stamp;
        last_state_stamp_ = msg->header.stamp;
        publishMapToOdomTF_();
        publishLogPose();
        return;
    }
    const ros::Time last_stamp = odom_stamp;
    odom_stamp = msg->header.stamp;
    last_state_stamp_ = msg->header.stamp;
    dt = (odom_stamp - last_stamp).toSec();

    // Estimated Velocities
    v_e = msg->twist.twist.linear.x;
    w_e = msg->twist.twist.angular.z;

    // State propagation
    double theta_e = X_state(2);
    double theta_mid = theta_e + w_e * dt * 0.5;
    X_state(0) += v_e * std::cos(theta_mid) * dt;
    X_state(1) += v_e * std::sin(theta_mid) * dt;
    X_state(2) = normalizeAngle(theta_e + w_e * dt);

    // Gradients calculation
    grad_f_X <<
    1, 0, -v_e * dt * std::sin(theta_mid),
    0, 1, v_e * dt * std::cos(theta_mid),
    0, 0, 1;

    grad_f_U <<
    std::cos(theta_mid), -0.5*v_e*dt*std::sin(theta_mid),
    std::sin(theta_mid), 0.5*v_e*dt*std::cos(theta_mid),
    0, 1;

    // Model Covariance Propagation
    P = grad_f_X * P * grad_f_X.transpose() + grad_f_U * Q * grad_f_U.transpose();
  
    publishMapToOdomTF_();
    publishLogPose();
}

void LocalizerNode::ekf_update(const localizer::BeaconMatch::ConstPtr& msg) {

    if (odom_skip_remaining_ > 0 || odom_stamp.isZero()) return;

    last_state_stamp_ = msg->header.stamp;

    const std::vector<localizer::Cluster>& clusters = msg->clusters;
    if(clusters.empty()) return;

    for(const auto& beacon_measured: clusters) {

        Eigen::Matrix<double, 2, 3> grad_h_X;

        Eigen::Matrix<double, 2, 1> Z_measured;
        Eigen::Matrix<double, 2, 1> Z_estimated;
        Eigen::Matrix<double, 2, 1> Z_diff;

        Eigen::Matrix<double, 2, 2> R;
        Eigen::Matrix<double, 2, 2> S;
        Eigen::Matrix<double, 3, 2> K;


        // Real Measures
        double dist_measured = std::hypot(beacon_measured.centroid.x, beacon_measured.centroid.y);
        double theta_measured = normalizeAngle(std::atan2(beacon_measured.centroid.y ,beacon_measured.centroid.x));

        Z_measured(0) = dist_measured;
        Z_measured(1) = theta_measured;

        // Estimated Measures
        // Check if beacon exists in map
        auto beacon_it = beacons.find(beacon_measured.beacon_match_name);
        if (beacon_it == beacons.end()) {
            ROS_WARN_THROTTLE(1.0, "[EKF] Beacon '%s' not found in map, skipping update", 
                             beacon_measured.beacon_match_name.c_str());
            continue;
        }
        const Beacon& beacon_fixed = beacon_it->second;
        double x_e = X_state(0), y_e = X_state(1), theta_e = X_state(2);

        double dist_estimated = std::hypot(beacon_fixed.pose.x - x_e, beacon_fixed.pose.y - y_e);
        double theta_estimated = normalizeAngle(std::atan2(beacon_fixed.pose.y - y_e, beacon_fixed.pose.x - x_e) - theta_e);
        if (dist_estimated < 1e-12) dist_estimated = 1e-12; 

        Z_estimated(0) = dist_estimated;
        Z_estimated(1) = theta_estimated;

        // Measured Covariance (calibrated from LiDAR measurements)
        // Values from calibration: sigma_r = 0.008024 m, sigma_theta = 0.051428 rad
        double sigma_r  = 0.008024;  // Calibrated: 0.008024 m
        double sigma_th = 0.051428;  // Calibrated: 0.051428 rad (2.95 degrees)

        R(0,0) = sigma_r * sigma_r; R(0,1) = R(1,0) = 0; R(1,1) = sigma_th * sigma_th;

        // Grad_h_X calculation
        grad_h_X <<
        -(beacon_fixed.pose.x - x_e) / dist_estimated, -(beacon_fixed.pose.y - y_e) / dist_estimated, 0,
        (beacon_fixed.pose.y - y_e)/(std::pow(dist_estimated, 2)), -(beacon_fixed.pose.x - x_e)/(std::pow(dist_estimated, 2)), -1;  

        // S matrix
        S = grad_h_X * P * grad_h_X.transpose() + R;

        // Kalman Gain
        K = P * grad_h_X.transpose() * S.ldlt().solve(Eigen::Matrix2d::Identity()); // ldlt().solve(Eigen::Matrix2d::Identity() -> .inverse()

        // Model Covariance Propagation
        Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
        P = (I3 - K * grad_h_X) * P;

        // State Update
        Z_diff = Z_measured - Z_estimated;
        Z_diff(1) = normalizeAngle(Z_diff(1));
        X_state += K * (Z_diff);
        X_state(2) = normalizeAngle(X_state(2));
    }

}

void LocalizerNode::publishLogPose() {

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = last_state_stamp_;
    odom_msg.header.frame_id = "map";      
    odom_msg.child_frame_id = "base_link"; 

    odom_msg.pose.pose.position.x = X_state(0);
    odom_msg.pose.pose.position.y = X_state(1);
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0,0,X_state(2));
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    for (int i=0;i<36;i++) odom_msg.pose.covariance[i] = 0.0;
    odom_msg.pose.covariance[0] = P(0,0);  
    odom_msg.pose.covariance[7] = P(1,1);   
    odom_msg.pose.covariance[35]= P(2,2);

    pose_pub.publish(odom_msg);

}

void LocalizerNode::publishMapToOdomTF_() {

    const ros::Time stamp = last_state_stamp_;
    if (stamp.isZero()) return;

    tf2::Transform t_map_base;
    t_map_base.setOrigin(tf2::Vector3(X_state(0), X_state(1), 0.0));
    tf2::Quaternion q_map_base; q_map_base.setRPY(0, 0, X_state(2));
    t_map_base.setRotation(q_map_base);

    geometry_msgs::TransformStamped T_odom_base_msg;
    try {
        if (tf_buffer.canTransform("odom", "base_link", stamp, ros::Duration(0.0))) {
            T_odom_base_msg = tf_buffer.lookupTransform("odom", "base_link", stamp, ros::Duration(0.0));
        } else if (tf_buffer.canTransform("odom", "base_link", ros::Time(0), ros::Duration(0.0))) {
            T_odom_base_msg = tf_buffer.lookupTransform("odom", "base_link", ros::Time(0), ros::Duration(0.0));
        } else {
            ROS_WARN_THROTTLE(1.0, "TF odom->base_link not available yet");
            return;
        }
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "TF lookup odom->base_link falhou: %s", ex.what());
        return;
    }

    tf2::Transform t_odom_base;
    tf2::fromMsg(T_odom_base_msg.transform, t_odom_base);

    tf2::Transform t_map_odom = t_map_base * t_odom_base.inverse();

    geometry_msgs::TransformStamped T_map_odom_msg;
    T_map_odom_msg.header.stamp = stamp;
    T_map_odom_msg.header.frame_id = "map";
    T_map_odom_msg.child_frame_id  = "odom";
    T_map_odom_msg.transform = tf2::toMsg(t_map_odom);
    tf_broadcaster.sendTransform(T_map_odom_msg);

}