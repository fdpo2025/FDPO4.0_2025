#ifndef F710_TELEOP_H
#define F710_TELEOP_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

class F710Teleop {
public:
    F710Teleop(ros::NodeHandle& nh);
    ~F710Teleop();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Publisher pick_pub_;
    ros::Subscriber joy_sub_;

    double max_v_, max_w_;
};

#endif