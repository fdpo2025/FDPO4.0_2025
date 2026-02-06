#include "f710_teleop.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "f710_teleop_node");
    ros::NodeHandle nh;

    F710Teleop teleop(nh);

    ros::spin();

    return 0;
}