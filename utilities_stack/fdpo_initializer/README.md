# fdpo_initializer

Nó ROS1 (Python) que actua como **supervisor** do stack FDPO na Raspberry Pi 4.

- Lê um **switch em GPIO** (ou, em bancada, um tópico `std_msgs/Bool`).
- Conta cliques curtos; após `inactivity_ms` sem novo clique, decide o modo.
- Arranca o `roslaunch` correspondente como **processo filho** numa
  *process group* separada.
- Em **long press** (`long_press_ms`, default 5 s), encerra o stack em
  cascata (SIGINT → SIGTERM → SIGKILL) e volta ao início, pronto para
  novo modo.

O `initializer_node` **não** pertence ao stack que supervisiona: vive no
`bootstrap.launch` (nível A) e nunca é o alvo do shutdown; o
`wake_up_fdpo*.launch` é sempre o nível B, filho do supervisor.

## Layout

```
utilities_stack/fdpo_initializer/
├─ scripts/initializer_node.py     # entry point (rosrun / roslaunch)
├─ modules/
│  ├─ gpio_button.py               # GPIO (gpiozero) / sim (ROS topic)
│  ├─ stack_supervisor.py          # Popen + process group + SIG escalation
│  └─ state_machine.py             # IDLE / COUNTING / RUNNING / STOPPING
├─ config/
│  ├─ initializer_params.yaml      # perfil real (Pi)
│  └─ initializer_params.sim.yaml  # perfil bancada (sem GPIO)
└─ launch/run_initializer.launch
```

## Mapeamento cliques → modo (defaults)

| Cliques | Launch                                  | Argumentos principais                         |
|--------:|-----------------------------------------|-----------------------------------------------|
| 1       | `conf/wake_up_fdpo.launch`              | `planner_type:=chris`                         |
| 2       | `conf/wake_up_fdpo.launch`              | `planner_type:=multi`                         |
| 3       | `conf/wake_up_fdpo_hardcoded.launch`    | `pico_num_robots:=3`                          |
| 4       | `conf/wake_up_fdpo_hardcoded.launch`    | `pico_num_robots:=4`                          |

Todos os modos passam `robot_id` / `pico_robot_id` a partir da variável
de ambiente **`FDPO_ROBOT_ID`** (definida em `/etc/fdpo/env`, por Pi).

## Teste em bancada (sem GPIO)

```bash
roscore &
roslaunch fdpo_initializer run_initializer.launch \
    params_file:=$(rospack find fdpo_initializer)/config/initializer_params.sim.yaml

# Noutra shell, simular dois cliques curtos e esperar a janela de inactividade:
rostopic pub -1 /initializer/sim_button std_msgs/Bool "data: true"
rostopic pub -1 /initializer/sim_button std_msgs/Bool "data: false"
rostopic pub -1 /initializer/sim_button std_msgs/Bool "data: true"
rostopic pub -1 /initializer/sim_button std_msgs/Bool "data: false"

# Ver o estado actual:
rostopic echo /initializer/status

# Simular long press (ficar "premido" >= 5 s):
rostopic pub -1 /initializer/sim_button std_msgs/Bool "data: true"
sleep 6
rostopic pub -1 /initializer/sim_button std_msgs/Bool "data: false"
```

## Instalação no robô real

Ver `deploy/README.md` na raiz do repositório.

## Parâmetros (resumo)

| Parâmetro          | Default | Descrição                                    |
|--------------------|---------|----------------------------------------------|
| `backend`          | gpiozero | `gpiozero` (real) ou `sim` (tópico ROS)      |
| `gpio_pin`         | 17      | Pino BCM ligado ao switch                    |
| `active_low`       | true    | Switch liga a GND quando premido             |
| `debounce_ms`      | 30      | Debounce software                            |
| `inactivity_ms`    | 1000    | Janela após último clique para decidir modo  |
| `long_press_ms`    | 5000    | Tempo para disparar "held" (reset do stack)  |
| `sigint_timeout_s` | 15.0    | Espera pós-SIGINT antes de escalar para TERM |
| `sigterm_timeout_s`| 5.0     | Espera pós-SIGTERM antes de SIGKILL          |
| `source_setup`     | ""      | `devel/setup.bash` a dar source no filho     |
| `click_to_mode`    | —       | Mapa N→`{launch_pkg, launch_file, args}`     |

## Dependências de sistema

- Python 3 com `rospy`.
- `gpiozero` (e `lgpio`/`RPi.GPIO` conforme a Pi):
  ```bash
  sudo apt install python3-gpiozero python3-lgpio
  ```
  Se indisponível, o nó faz fallback automático para o backend `sim`.
