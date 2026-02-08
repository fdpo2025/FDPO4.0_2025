#include "f710_teleop.h"

F710Teleop::F710Teleop(ros::NodeHandle& nh) : nh_(nh) {
    // Publicadores para os tópicos que o pico_driver_node subscreve
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    pick_pub_ = nh_.advertise<std_msgs::Bool>("/pick_box", 10);

    // Subscrição ao nó do joystick
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &F710Teleop::joyCallback, this);

    // Parâmetros configuráveis
    nh_.param("max_linear_vel", max_v_, 0.5);
    nh_.param("max_angular_vel", max_w_, 1.5);
}

F710Teleop::~F710Teleop() {}

void F710Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    geometry_msgs::Twist twist;

    // 1. Deadzone manual: evita que o robô ande sozinho se o analógico não voltar ao zero perfeito
    double raw_linear = (std::abs(joy->axes[1]) > 0.05) ? joy->axes[1] : 0.0;
    double raw_angular = (std::abs(joy->axes[3]) > 0.05) ? joy->axes[3] : 0.0;

    // 2. Curva Exponencial: dá mais precisão em velocidades baixas e força total só no fim do curso
    // Isso torna o comando muito mais "intuitivo"
    double linear_val = raw_linear * std::abs(raw_linear);
    double angular_val = raw_angular * std::abs(raw_angular);

    // 3. Limitação Dinâmica: Se estiveres no máximo de linear, reduzimos um pouco a linear 
    // para dar "espaço" aos motores para rodarem (angular)
    if (std::abs(linear_val) > 0.0) {
        twist.linear.x = linear_val * max_v_ * (1.0 - std::abs(angular_val) * 0.3);
    } else {
        twist.linear.x = linear_val * max_v_;
    }

    twist.angular.z = angular_val * max_w_;
    
    vel_pub_.publish(twist);

    //
    std_msgs::Bool pick_msg;
    pick_msg.data = (joy->buttons[0] == 1);
    pick_pub_.publish(pick_msg);
}