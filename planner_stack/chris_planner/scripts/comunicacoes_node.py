#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt32, Int32MultiArray


ROBOT_ID = 1   # meter 1 ou 2


class ComunicacoesNode:

    def __init__(self):
        rospy.init_node("comunicacoes")

        self.robot_id = ROBOT_ID
        # self.robot_id = rospy.get_param("~robot_id", 1)

        # -------------------------------------------------
        # Publishers comuns
        # -------------------------------------------------

        # caminho local para o robô
        self.pub_planned_paths = rospy.Publisher(
            "/planned_paths", Int32MultiArray, queue_size=10
        )

        # poses/posições globais para o planner
        self.pub_robot1_pose = rospy.Publisher(
            "/robot1/current_pose", UInt32, queue_size=10
        )
        self.pub_robot2_pose = rospy.Publisher(
            "/robot2/current_pose", UInt32, queue_size=10
        )

        # interface com o driver
        self.pub_cp_send = rospy.Publisher(
            "/cp_send", UInt32, queue_size=10
        )
        self.pub_path_send = rospy.Publisher(
            "/path_send", Int32MultiArray, queue_size=10
        )

        # -------------------------------------------------
        # Modo robô 1
        # -------------------------------------------------
        if self.robot_id == 1:
            rospy.loginfo("Comunicacoes: modo ROBOT_ID = 1")

            rospy.Subscriber(
                "/robot1_planned_paths",
                Int32MultiArray,
                self.robot1_planned_path_cb
            )

            rospy.Subscriber(
                "/robot2_planned_paths",
                Int32MultiArray,
                self.robot2_planned_path_cb
            )

            rospy.Subscriber(
                "/this_current_pose",
                UInt32,
                self.this_current_pose_robot1_cb
            )

            rospy.Subscriber(
                "/cp_rcv",
                UInt32,
                self.cp_rcv_cb
            )

        # -------------------------------------------------
        # Modo robô 2
        # -------------------------------------------------
        elif self.robot_id == 2:
            rospy.loginfo("Comunicacoes: modo ROBOT_ID = 2")

            rospy.Subscriber(
                "/this_current_pose",
                UInt32,
                self.this_current_pose_robot2_cb
            )

            rospy.Subscriber(
                "/path_rcv",
                Int32MultiArray,
                self.path_rcv_cb
            )

        else:
            rospy.logerr("ROBOT_ID tem de ser 1 ou 2")
            rospy.signal_shutdown("ROBOT_ID inválido")

    # =====================================================
    # ROBÔ 1
    # =====================================================

    def robot1_planned_path_cb(self, msg):
        """
        Recebe /robot1_planned_paths e publica diretamente em /planned_paths
        """
        rospy.loginfo(f"[R1] Caminho local recebido: {list(msg.data)}")
        self.pub_planned_paths.publish(msg)

    def robot2_planned_path_cb(self, msg):
        """
        Recebe /robot2_planned_paths e publica em /path_send
        """
        rospy.loginfo(f"[R1] Caminho para robô 2 recebido: {list(msg.data)}")
        self.send_path_serial(msg)

    def this_current_pose_robot1_cb(self, msg):
        """
        Recebe /this_current_pose e publica diretamente em /robot1/current_pose
        """
        rospy.loginfo(f"[R1] Posição local recebida: {msg.data}")
        self.pub_robot1_pose.publish(msg)

    def cp_rcv_cb(self, msg):
        """
        No robô 1:
        recebe em /cp_rcv a posição do robô 2 e publica em /robot2/current_pose
        """
        pose_msg = self.read_pose_from_serial(msg)

        if pose_msg is not None:
            rospy.loginfo(f"[R1] Posição recebida do robô 2: {pose_msg.data}")
            self.pub_robot2_pose.publish(pose_msg)

    # =====================================================
    # ROBÔ 2
    # =====================================================

    def this_current_pose_robot2_cb(self, msg):
        """
        No robô 2:
        recebe /this_current_pose e envia para /cp_send
        """
        rospy.loginfo(f"[R2] Posição local para enviar: {msg.data}")
        self.send_pose_serial(msg)

    def path_rcv_cb(self, msg):
        """
        No robô 2:
        recebe /path_rcv e publica em /planned_paths
        """
        path_msg = self.read_path_from_serial(msg)

        if path_msg is not None:
            rospy.loginfo(f"[R2] Caminho recebido: {list(path_msg.data)}")
            self.pub_planned_paths.publish(path_msg)

    # =====================================================
    # FUNÇÕES DE ENVIO/RECEÇÃO VIA TÓPICOS
    # =====================================================

    def send_path_serial(self, msg):
        """
        Em vez de serial direta, publica em /path_send
        """
        rospy.loginfo(f"[TOPIC] /path_send <- {list(msg.data)}")
        self.pub_path_send.publish(msg)

    def send_pose_serial(self, msg):
        """
        Em vez de serial direta, publica em /cp_send
        """
        rospy.loginfo(f"[TOPIC] /cp_send <- {msg.data}")
        self.pub_cp_send.publish(msg)

    def read_pose_from_serial(self, msg):
        """
        Em vez de ler da serial, recebe a mensagem do subscriber /cp_rcv
        """
        return msg

    def read_path_from_serial(self, msg):
        """
        Em vez de ler da serial, recebe a mensagem do subscriber /path_rcv
        """
        return msg


if __name__ == "__main__":
    node = ComunicacoesNode()
    rospy.spin()