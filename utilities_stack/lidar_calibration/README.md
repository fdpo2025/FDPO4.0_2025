# Lidar Calibration - Calibração de Covariância de Medições

Este pacote permite recolher medições de beacons do LiDAR enquanto o robô está parado em diferentes posições, e calcular estatísticas (média e variância) para calibrar a matriz R do EKF.

## Como Funciona

1. O robô move-se para diferentes posições do mapa (usando `cmd_vel` ou `gotoxy`)
2. Em cada posição, o robô para e inicia a recolha de medições
3. As medições são guardadas num ficheiro YAML (organizadas por iteração/paragem)
4. No final, calcula-se a média de todas as medições para cada beacon
5. Calculam-se os resíduos (cada medição - média)
6. Calcula-se a variância a partir do somatório dos resíduos ao quadrado

## Uso

### 1. Iniciar o nó de recolha

```bash
roslaunch lidar_calibration run_measurement_collector.launch
```

### 2. Processo de Recolha de Dados

Para cada posição do mapa onde quer recolher dados:

```bash
# 1. Mover robô para posição e pará-lo
# (usar cmd_vel ou gotoxy)

# 2. Iniciar recolha de medições
rosservice call /start_collection

# 3. Esperar alguns segundos (recomendado: 20-30 segundos)
# O nó está a recolher medições automaticamente

# 4. Parar recolha (guarda automaticamente em YAML)
rosservice call /stop_collection

# 5. Mover para próxima posição e repetir (voltar ao passo 2)
```

**Nota:** Cada vez que chama `/start_collection`, inicia uma nova iteração. Ao chamar `/stop_collection`, os dados são guardados no ficheiro YAML.

### 3. Calcular Estatísticas Finais

Depois de recolher dados em várias posições:

```bash
# Iniciar nó de cálculo de estatísticas
roslaunch lidar_calibration run_statistics_calculator.launch

# Opção 1: Usar script Python (recomendado)
rosrun lidar_calibration print_statistics.py

# Opção 2: Chamar serviço diretamente
rosservice call /calculate_statistics
```

## Serviços Disponíveis

### Nó de Recolha (`measurement_collector_node`):
- `/start_collection`: Inicia a recolha de medições (nova iteração)
- `/stop_collection`: Para a recolha e guarda em YAML

### Nó de Estatísticas (`statistics_calculator_node`):
- `/calculate_statistics`: Calcula estatísticas finais (média, resíduos, variância)

## Ficheiro YAML

Os dados são guardados em:
```
~/.ros/.../lidar_calibration/data/measurements.yaml
```

Estrutura:
```yaml
iterations:
  - iteration_id: 1
    beacons:
      beacon1:
        r: [0.5, 0.51, 0.49, ...]
        theta: [0.1, 0.11, 0.09, ...]
        num_samples: 100
      beacon2:
        ...
  - iteration_id: 2
    beacons:
      ...
```

## Interpretação dos Resultados

O script `print_statistics.py` mostra:
- **Média r**: Distância média medida até cada beacon (de todas as iterações)
- **Média θ**: Ângulo médio medido até cada beacon
- **Var r**: Variância da distância calculada a partir dos resíduos
- **Var θ**: Variância do ângulo calculada a partir dos resíduos
- **Amostras**: Número total de medições recolhidas

### Cálculo da Variância

Para cada beacon:
1. Média: `μ = (1/N) * Σ(medicao_i)`
2. Resíduos: `residual_i = medicao_i - μ`
3. Variância: `σ² = (1/N) * Σ(residual_i²)`

### Valores Recomendados para Matriz R

Para usar no código EKF (`localizer_node.cpp`), use:

```cpp
double sigma_r = sqrt(var_r);      // Desvio padrão da distância
double sigma_th = sqrt(var_theta);  // Desvio padrão do ângulo
```

Pode usar valores médios de todos os beacons ou valores específicos por beacon.

## Exemplo de Workflow Completo

```bash
# Terminal 1: Iniciar nó de recolha
roslaunch lidar_calibration run_measurement_collector.launch

# Terminal 2: Recolher dados
# Posição 1
rosservice call /start_collection
sleep 20  # Esperar 20 segundos
rosservice call /stop_collection

# Posição 2
rosservice call /start_collection
sleep 20
rosservice call /stop_collection

# ... repetir para mais posições ...

# Terminal 3: Calcular estatísticas
roslaunch lidar_calibration run_statistics_calculator.launch
rosrun lidar_calibration print_statistics.py
```

## Notas

- Recolha pelo menos 50-100 medições por posição para estatísticas confiáveis
- Quanto mais posições diferentes, melhor a calibração
- Certifique-se de que o robô está realmente parado durante a recolha
- Os dados são guardados persistentemente em YAML (não se perdem ao reiniciar)
- Cada chamada a `/start_collection` cria uma nova iteração


