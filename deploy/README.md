# Deploy (instalação manual por robô)

Esta pasta contém ficheiros que **não** pertencem ao workspace ROS e que
têm de ser copiados manualmente para cada Raspberry Pi 4 antes de o
sistema arrancar sozinho ao ligar.

```
deploy/
├─ systemd/
│  ├─ fdpo-roscore.service       # serviço que sobe o roscore (opcional, mas
│  │                             # recomendado para um roscore estável)
│  ├─ fdpo-bootstrap.service     # serviço que sobe o initializer_node
│  ├─ fdpo-env                   # ficheiro com FDPO_ROBOT_ID (por robô)
│  └─ install_systemd.sh         # script de instalação/activação
└─ README.md                     # este ficheiro
```

## Pré-requisitos

- Ubuntu 20.04 + ROS Noetic instalado em `/opt/ros/noetic`.
- Workspace compilado em `/home/user/catkin_ws_fdpo` (ajustar nos `.service`
  se o caminho for outro).
- Utilizador que vai correr o ROS pertence aos grupos `gpio` e `dialout`.
- `source /opt/ros/noetic/setup.bash` e
  `source ~/catkin_ws_fdpo/devel/setup.bash` a funcionar na shell do utilizador.

## Instalação

1. Definir o ID do robô **em cada Pi** (valor único por robô, 0-based):

   ```bash
   sudo install -d -m 0755 /etc/fdpo
   echo "FDPO_ROBOT_ID=0" | sudo tee /etc/fdpo/env > /dev/null   # <-- trocar 0 pelo ID desta Pi
   ```

   (O ficheiro `deploy/systemd/fdpo-env` deste repo é apenas um
   template; o ficheiro real fica em `/etc/fdpo/env`.)

2. Instalar as units (requer root):

   ```bash
   cd ~/catkin_ws_fdpo/src/fdpo-ros-stack/deploy/systemd
   sudo ./install_systemd.sh
   ```

3. Verificar:

   ```bash
   systemctl status fdpo-roscore.service
   systemctl status fdpo-bootstrap.service
   journalctl -u fdpo-bootstrap.service -f
   ```

## Variáveis a rever antes de `install_systemd.sh`

Editar os `.service` se algum destes não corresponder à Pi:

- `User=` e `Group=` — utilizador que corre o ROS (por defeito `user`).
- `WorkingDirectory=` — raiz do workspace catkin.
- `EnvironmentFile=` — caminho para `/etc/fdpo/env` (FDPO_ROBOT_ID).
- Caminho para `setup.bash` dentro do `ExecStart` (devel do workspace).

O `fdpo-bootstrap.service` usa `roslaunch --wait` e `ROS_MASTER_URI=http://localhost:11311`
para **não** arrancar um segundo `roscore` se o `fdpo-roscore` ainda estiver a
inicializar (evita “auto-starting new master” e conflitos na porta 11311).

O bootstrap usa **`Wants=`** (e não `Requires=`) em relação ao `fdpo-roscore` e ao
`pigpiod`: com `Requires=`, um `systemctl restart fdpo-roscore` faz o systemd
**parar** o bootstrap durante o restart do master, o que interrompe o Python a
meio do import (`KeyboardInterrupt` em `numpy`, `rosmaster`, etc.).

## Desinstalação

```bash
sudo systemctl disable --now fdpo-bootstrap.service fdpo-roscore.service
sudo rm -f /etc/systemd/system/fdpo-bootstrap.service
sudo rm -f /etc/systemd/system/fdpo-roscore.service
sudo systemctl daemon-reload
```
