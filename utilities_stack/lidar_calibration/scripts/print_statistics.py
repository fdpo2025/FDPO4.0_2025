#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script para imprimir estatísticas das medições recolhidas
"""

import rospy
from lidar_calibration.srv import GetStatistics
import math
import os
from datetime import datetime

def print_statistics():
    rospy.init_node('print_statistics', anonymous=True)
    
    rospy.wait_for_service('calculate_statistics')
    get_stats = rospy.ServiceProxy('calculate_statistics', GetStatistics)
    
    try:
        resp = get_stats()
        
        if not resp.success:
            print("Erro: %s" % resp.message)
            return
        
        # Obter caminho do pacote para guardar ficheiro
        package_path = rospy.get_param('~package_path', None)
        if package_path is None:
            import rospkg
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('lidar_calibration')
        
        output_dir = os.path.join(package_path, 'data')
        os.makedirs(output_dir, exist_ok=True)
        
        # Criar nome do ficheiro com timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = os.path.join(output_dir, 'statistics_' + timestamp + '.txt')
        yaml_output_file = os.path.join(output_dir, 'statistics_' + timestamp + '.yaml')
        
        # Preparar conteúdo para guardar
        output_lines = []
        yaml_content = {}
        
        output_lines.append("="*80)
        output_lines.append("ESTATÍSTICAS DAS MEDIÇÕES DE BEACONS")
        output_lines.append("="*80)
        output_lines.append("")
        output_lines.append("{:<20} {:>12} {:>12} {:>15} {:>15} {:>10}".format(
            "Beacon", "Média r (m)", "Média θ (rad)", "Var r (m²)", 
            "Var θ (rad²)", "Amostras"))
        output_lines.append("-"*80)
        
        for i in range(len(resp.beacon_names)):
            name = resp.beacon_names[i]
            mean_r = resp.mean_r[i]
            mean_theta = resp.mean_theta[i]
            var_r = resp.var_r[i]
            var_theta = resp.var_theta[i]
            samples = resp.num_samples[i]
            
            # Calcular desvio padrão
            std_r = math.sqrt(var_r)
            std_theta = math.sqrt(var_theta)
            
            line = "{:<20} {:>12.4f} {:>12.4f} {:>15.9f} {:>15.9f} {:>10}".format(
                name, mean_r, mean_theta, var_r, var_theta, samples)
            output_lines.append(line)
            
            # Guardar em formato YAML também
            yaml_content[name] = {
                'mean_r': mean_r,
                'mean_theta': mean_theta,
                'var_r': var_r,
                'var_theta': var_theta,
                'std_r': std_r,
                'std_theta': std_theta,
                'num_samples': samples
            }
        
        output_lines.append("")
        output_lines.append("="*80)
        output_lines.append("VALORES RECOMENDADOS PARA MATRIZ R:")
        output_lines.append("="*80)
        output_lines.append("")
        output_lines.append("Para cada beacon, usar:")
        output_lines.append("  sigma_r = sqrt(var_r)")
        output_lines.append("  sigma_theta = sqrt(var_theta)")
        output_lines.append("")
        output_lines.append("Ou usar valores médios de todos os beacons:")
        
        if len(resp.var_r) > 0:
            avg_var_r = sum(resp.var_r) / len(resp.var_r)
            avg_var_theta = sum(resp.var_theta) / len(resp.var_theta)
            avg_std_r = math.sqrt(avg_var_r)
            avg_std_theta = math.sqrt(avg_var_theta)
            
            output_lines.append("  sigma_r médio = %.6f m" % avg_std_r)
            output_lines.append("  sigma_theta médio = %.6f rad (%.2f graus)" % (
                avg_std_theta, avg_std_theta * 180.0 / math.pi))
            output_lines.append("")
            output_lines.append("Para usar no código EKF:")
            output_lines.append("  double sigma_r = %.6f;" % avg_std_r)
            output_lines.append("  double sigma_th = %.6f;" % avg_std_theta)
            
            # Adicionar valores médios ao YAML
            yaml_content['_averages'] = {
                'avg_var_r': avg_var_r,
                'avg_var_theta': avg_var_theta,
                'avg_std_r': avg_std_r,
                'avg_std_theta': avg_std_theta,
                'recommended_sigma_r': avg_std_r,
                'recommended_sigma_theta': avg_std_theta
            }
        
        output_lines.append("")
        output_lines.append("="*80)
        
        # Imprimir no terminal
        for line in output_lines:
            print(line)
        
        # Guardar em ficheiro texto
        with open(output_file, 'w') as f:
            f.write('\n'.join(output_lines))
        print("\n[INFO] Estatísticas guardadas em: %s" % output_file)
        
        # Guardar em YAML
        try:
            import yaml
            with open(yaml_output_file, 'w') as f:
                yaml.dump(yaml_content, f, default_flow_style=False, sort_keys=False)
            print("[INFO] Estatísticas guardadas em YAML: %s" % yaml_output_file)
        except ImportError:
            print("[WARN] PyYAML não instalado. YAML não foi guardado.")
        
    except rospy.ServiceException as e:
        print("Erro ao chamar serviço: %s" % e)

if __name__ == '__main__':
    print_statistics()


