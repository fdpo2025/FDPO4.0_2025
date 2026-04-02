#include "comunicacoes_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "comunicacoes");
    ros::NodeHandle nh("~");

    int robot_id = 1;
    nh.param("robot_id", robot_id, 1);

    ComunicacoesNode node(nh, robot_id);

    ros::spin();
    return 0;
}
