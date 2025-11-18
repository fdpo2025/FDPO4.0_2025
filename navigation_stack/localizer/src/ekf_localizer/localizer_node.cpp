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
    double q_xx, q_yy;
    nh.param("ekf_params/process_covariance/position_x", q_xx, 0.0005);
    nh.param("ekf_params/process_covariance/position_y", q_yy, 0.0005);
    
    Q.setZero();
    Q(0,0) = q_xx;
    Q(1,1) = q_yy;
    
    ROS_INFO("[LocalizerNode] EKF Parameters loaded:");
    ROS_INFO("  - Initial covariance P: [%.6f, %.6f, %.6f]", p_xx, p_yy, p_theta);
    ROS_INFO("  - Process covariance Q: [%.6f, %.6f]", q_xx, q_yy);
}

double LocalizerNode::normalizeAngle(double theta) {

    while(theta > M_PI) theta -= 2.0 * M_PI;
    while (theta <= -M_PI) theta += 2.0*M_PI;

    return theta;
}


void LocalizerNode::ekf_predict(const nav_msgs::Odometry::ConstPtr& msg) {
    
    // Dt update
    if (odom_stamp.isZero()) {     // 1st message   
        odom_stamp = msg->header.stamp;
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
        Beacon beacon_fixed = beacons[beacon_measured.beacon_match_name];
        double x_e = X_state(0), y_e = X_state(1), theta_e = X_state(2);

        double dist_estimated = std::hypot(beacon_fixed.pose.x - x_e, beacon_fixed.pose.y - y_e);
        double theta_estimated = normalizeAngle(std::atan2(beacon_fixed.pose.y - y_e, beacon_fixed.pose.x - x_e) - theta_e);
        if (dist_estimated < 1e-12) dist_estimated = 1e-12; 

        Z_estimated(0) = dist_estimated;
        Z_estimated(1) = theta_estimated;

        // Measured Covariance
        // double sigma_r  = 0.005;  
        // double sigma_th = 1.5;
        double sigma_r  = 0.03 + 0.02 * dist_estimated;  
        double sigma_th = 1.5 * M_PI/180.0;

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
        T_odom_base_msg = tf_buffer.lookupTransform("odom", "base_link", stamp, ros::Duration(0.05));
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