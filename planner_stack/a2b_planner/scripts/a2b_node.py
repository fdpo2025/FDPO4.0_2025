#!/usr/bin/env python3

import rospy
import sys
import os
import json
from std_msgs.msg import String, Int32MultiArray

# Configuração de caminhos para importar os módulos locais
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

# Importar os módulos reutilizados
try:
    from modules import factory
    from modules import yaml_utils
except ImportError as e:
    rospy.logerr(f"Falha ao importar modulos: {e}")
    sys.exit(1)

class A2BPlannerNode:
    def __init__(self):
        rospy.init_node('a2b_planner_node', anonymous=True)
        
        # --- 1. Carregar Configurações ---
        # Usa rospack para encontrar o caminho dos ficheiros config
        import rospkg
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('a2b_planner')
        
        graph_file = os.path.join(pkg_path, 'config', 'graph.yaml')
        components_file = os.path.join(pkg_path, 'config', 'factory_components.yaml')
        
        rospy.loginfo(f"Carregando grafo de: {graph_file}")
        
        try:
            self.graph_dict = yaml_utils.load_file(graph_file)
            self.components_dict = yaml_utils.load_file(components_file)
            
            # Inicializa a Fábrica (que inicializa o Grafo internamente)
            self.factory = factory.FactoryModel(self.graph_dict, self.components_dict)
            rospy.loginfo("Modelo da Fabrica e Grafo carregados com sucesso.")
            
        except Exception as e:
            rospy.logerr(f"Erro ao carregar configs: {e}")
            sys.exit(1)

        # --- 2. Estado do Robô ---
        # Define onde o robo comeca (baseado no YAML ou hardcoded se preferires)
        self.current_robot_node = self.factory.robot_start_node 
        rospy.loginfo(f"Robo inicializado no no: {self.current_robot_node}")

        # --- 3. Comunicação ROS ---
        # Subscriber: Recebe ordens da UI (Node-RED)
        self.mission_sub = rospy.Subscriber('/mission_order', String, self.handle_mission)
        
        # Publisher: Envia a lista de nós para o controlador mover o robô
        self.path_pub = rospy.Publisher('/planned_paths', Int32MultiArray, queue_size=10)

    def handle_mission(self, msg):
        """
        Callback quando chega uma ordem da UI.
        Espera JSON: {"pickup_node": 0, "dropoff_node": 35}
        """
        try:
            # Log para saberes exatamente o que chegou
            rospy.loginfo(f"Recebido do Node-RED: {msg.data}")

            # 1. Parse do JSON
            data = json.loads(msg.data)
            
            # 2. Extrair dados com segurança
            # Tenta buscar 'pickup_node' (novo padrão). Se falhar, tenta 'pickup' ou 'target_node'
            p_raw = data.get('pickup_node') 
            
            # Se a UI não mandou pickup_node, tenta chaves antigas por segurança
            if p_raw is None:
                p_raw = data.get('target_node')

            d_raw = data.get('dropoff_node')

            # 3. Validação antes de converter
            if p_raw is None or d_raw is None:
                rospy.logerr(f"ERRO: Dados incompletos. Recebi: {data}")
                return

            # Converter para Inteiros
            pickup = int(p_raw)
            dropoff = int(d_raw)
            
            rospy.loginfo(f"Nova Missao validada: Ir de {self.current_robot_node} -> Pegar em {pickup} -> Largar em {dropoff}")

            # 4. Calcular Rota 1: Onde estou -> Onde está a caixa
            # Usa shortest_path_compact para incluir coordenada adjacente ao sair de warehouses
            path_to_pickup = self.factory.shortest_path_compact(self.current_robot_node, pickup)
            
            # 5. Calcular Rota 2: Onde está a caixa -> Onde entregar
            path_to_dropoff = self.factory.shortest_path_compact(pickup, dropoff)

            # 6. Colar as rotas (Stitching)
            full_path = path_to_pickup + path_to_dropoff[1:]

            rospy.loginfo(f"Trajetoria calculada ({len(full_path)} nos): {full_path}")

            # 7. Publicar
            path_msg = Int32MultiArray()
            path_msg.data = full_path
            self.path_pub.publish(path_msg)
            
            # 8. Atualizar estado interno
            self.current_robot_node = dropoff

        except ValueError as e:
            rospy.logerr(f"Erro de valor (IDs invalidos?): {e}")
        except Exception as e:
            rospy.logerr(f"Erro critico ao processar missao: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = A2BPlannerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
