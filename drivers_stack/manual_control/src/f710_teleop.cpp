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

    // Analógico Esquerdo (Eixo 1) -> Velocidade Linear
    twist.linear.x = max_v_ * joy->axes[1];
    // Analógico Direito (Eixo 3) -> Velocidade Angular
    twist.angular.z = max_w_ * joy->axes[3];
    
    vel_pub_.publish(twist);

    // Botão 'A' (Index 0) -> Controlo do pick_box
    std_msgs::Bool pick_msg;
    pick_msg.data = (joy->buttons[0] == 1);
    pick_pub_.publish(pick_msg);
}