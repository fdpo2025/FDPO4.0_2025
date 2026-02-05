#include <ros/ros.h>
#include "wifi_driver_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wifi_driver_node");
  ros::NodeHandle nh("~");

  WifiDriverNode node(nh);
  ROS_INFO("Wifi driver a correr...");

  ros::spin();
  return 0;
}
