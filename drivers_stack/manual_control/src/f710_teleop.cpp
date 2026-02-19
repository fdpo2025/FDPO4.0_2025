#include "f710_teleop.h"

F710Teleop::F710Teleop(ros::NodeHandle& nh) : nh_(nh) {
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    pick_pub_ = nh_.advertise<std_msgs::Bool>("/pick_box", 10);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &F710Teleop::joyCallback, this);

    nh_.param("max_linear_vel", max_v_, 0.5);
    nh_.param("max_angular_vel", max_w_, 1.5);
}

F710Teleop::~F710Teleop() {}

void F710Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    geometry_msgs::Twist twist;

    // 1. Deadzone manual e processamento de velocidades
    double raw_linear = (std::abs(joy->axes[1]) > 0.05) ? joy->axes[1] : 0.0;
    double raw_angular = (std::abs(joy->axes[3]) > 0.05) ? joy->axes[3] : 0.0;

    double linear_val = raw_linear * std::abs(raw_linear);
    double angular_val = raw_angular * std::abs(raw_angular);

    if (std::abs(linear_val) > 0.0) {
        twist.linear.x = linear_val * max_v_ * (1.0 - std::abs(angular_val) * 0.3);
    } else {
        twist.linear.x = linear_val * max_v_;
    }
    twist.angular.z = angular_val * max_w_;
    vel_pub_.publish(twist);

    // 2. Lógica de Toggle do Íman (Rising Edge no Botão 'A' / Index 0)
    bool current_button_state = (joy->buttons[0] == 1);

    // Deteta a transição de "não pressionado" para "pressionado"
    if (current_button_state && !last_button_state_) {
        magnet_state_ = !magnet_state_; // Inverte o estado
        
        std_msgs::Bool pick_msg;
        pick_msg.data = magnet_state_;
        pick_pub_.publish(pick_msg);
        
        ROS_INFO("Estado do Iman (pick_box) alterado para: %s", magnet_state_ ? "TRUE" : "FALSE");
    }

    // Guarda o estado para a próxima comparação
    last_button_state_ = current_button_state;
}