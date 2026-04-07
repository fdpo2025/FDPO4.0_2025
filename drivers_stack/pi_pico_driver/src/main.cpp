#include "pico_driver_node.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "pico_driver_node");
  // Private namespace so nh.param() reads /pico_driver_node/* from the launch file.
  // Topics use leading "/" and stay global.
  ros::NodeHandle nh("~");

  PiPicoDriver driver(nh);  

  ros::spin(); 

  return 0;
}