#pragma once
#include <ros/ros.h>

class PathPlanner
{
public:
  PathPlanner(ros::NodeHandle& nh);
  void run();

private:
  ros::NodeHandle nh_;
};
