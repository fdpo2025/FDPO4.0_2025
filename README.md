# Software Stack for the Robot Factory 4.0 Competition @ National Robotics Festival 2026

ROS1 (Noetic) stack for local navigation with HLDS HLS‑LFCD2 LiDAR, beacon‑based localization (with EKF), navigation controller, and **Stage** simulation. The **`conf/`** package acts as the *single source of truth* for launch wrappers and parameter configuration.

> Context: developed by **Electrical and Computer Engineering students at FEUP (Faculty of Engineering, University of Porto)** for the **Robot Factory 4.0** competition at the **National Robotics Festival 2026**.

---

## TL;DR (Quick Start)

### Build

```bash
cd ~/catkin_ws_fdpo
catkin_make
source devel/setup.bash
```

### Simulation (Stage)

Runs the Stage world, including navigation + HMI via `conf/`.

```bash
roslaunch simulation_stage run_sim_stage.launch
```

### Real Robot (Full Bring‑up)

Brings up drivers, navigation, and HMI through `conf/`.

```bash
roslaunch conf wake_up_fdpo.launch
```

#### LiDAR Driver Selection

The system supports two LiDAR drivers. By default, it uses the YDLidar X4 driver:

```bash
# YDLidar X4 (default)
roslaunch conf wake_up_fdpo.launch

# HLS-LFCD2
roslaunch conf wake_up_fdpo.launch lidar_driver:=hlds

# Com RViz (opcional)
roslaunch conf wake_up_fdpo.launch use_rviz:=true

# Com logs de debug Pi4-Pico (para diagnóstico)
roslaunch conf wake_up_fdpo.launch debug_pico_comm:=true

# Combinar várias opções
roslaunch conf wake_up_fdpo.launch lidar_driver:=hlds debug_pico_comm:=true use_rviz:=true
```

**Alternar logs Pi4-Pico:**
```bash
# Ativar logs
rosparam set /pico_driver_node/debug_comm true

# Desativar logs
rosparam set /pico_driver_node/debug_comm false
```

#### Manual Control (Teleop)

Control the robot manually using keyboard or a graphical interface:

**Option 1: Keyboard Control (Terminal)**
```bash
# Instalar o pacote (se necessário)
sudo apt install ros-noetic-teleop-twist-keyboard

# Lançar o controlo por teclado
roslaunch conf launch/hmi/teleop_keyboard.launch
```

**Comandos do teclado:**
- `i`: avançar
- `,`: recuar
- `j`: rodar esquerda
- `l`: rodar direita
- `u`: avançar + rodar esquerda
- `o`: avançar + rodar direita
- `k`: parar
- `q/z`: aumentar/diminuir velocidade linear
- `w/x`: aumentar/diminuir velocidade angular

**Option 2: Graphical Interface (RQT)**
```bash
# Lançar interface gráfica com sliders
roslaunch conf launch/hmi/teleop_rqt_steering.launch
```

>  **Important:** run `source devel/setup.bash` in **every terminal** where you run ROS nodes.

---

## Requirements

* **OS:** Ubuntu 20.04 LTS (recommended for ROS Noetic)
* **ROS:** ROS1 Noetic (with `roscpp`, `std_msgs`, `geometry_msgs`, `nav_msgs`)
* **Dependencies:**
  * `tf2`, `tf2_ros`, `tf2_geometry_msgs`
  * `laser_geometry`
  * `message_generation`, `message_runtime`
  * `dynamic_reconfigure`
  * Visualization: `rviz`, `visualization_msgs`
* **Simulation:** Stage (`stage_ros`)
* **Hardware (real robot):**
  * LiDAR (choose one):
    * HLDS HLS‑LFCD2 (LDS‑01/02) — *driver included*
    * YDLidar X4 — *driver included via 5dpo_driver_laser_2d*
  * **Raspberry Pi 4** running ROS and this stack (high‑level)
  * **Raspberry Pi Pico** handling actuator control (low‑level)

> Tip: install missing packages via `apt`:
> ```bash
> sudo apt install ros-noetic-tf2-ros ros-noetic-laser-geometry \
>   ros-noetic-dynamic-reconfigure ros-noetic-stage-ros
> ```

---

## Repository Structure

```
FDPO4.0_2025/
├─ conf/                             # Centralized launch wrappers & parameters
│  └─ launch/wake_up_fdpo.launch     # Full bring-up of the real robot
├─ drivers_stack/                    # Hardware drivers
│  ├─ lidar_drivers/                 # LiDAR drivers
│  │  ├─ hls_lfcd_lds_driver/        # HLDS HLS-LFCD2 driver (LDS-01/02)
│  │  └─ ydlidar_x4_drivers/         # YDLidar X4 driver (5dpo_driver_laser_2d)
│  └─ pi_pico_driver/                # Raspberry Pi Pico communication driver
├─ navigation_stack/                 # Localization and navigation controller
│  ├─ localizer/                     # Beacon-based localization with EKF
│  │  ├─ msg/                        # Custom message definitions
│  │  ├─ launch/                     # Launch files for detector and EKF
│  │  ├─ src/                        # Source code (detector + EKF)
│  │  └─ include/                    # Headers
│  └─ navigation_controller/         # Path following and FSM control
│     ├─ cfg/Navigation.cfg          # Dynamic reconfigure parameters
│     ├─ srv/NavigationControl.srv   # Service for controller commands
│     ├─ launch/                     # Controller launch files
│     └─ src/                        # FSM and controller implementation
├─ simulation_stage/                 # Stage-based simulation environment
│  ├─ worlds/                        # Factory floor world file(s)
│  ├─ models/                        # Robot and environment models
│  ├─ launch/                        # Simulation launch files
│  └─ hmi/                           # RViz configs for simulation
├─ utilities_stack/                  # Utility tools
│  ├─ path_log/                      # Odometry-to-path logging node
│  └─ pointcloud_converter/          # PointCloud to PointCloud2 converter
├─ CMakeLists.txt                    # Top-level build system
└─ README.md                         # Project documentation
```

---

## System Overview (Data Flow)

**HLDS LiDAR** → **Beacon Detector (DBSCAN + matching)** → **EKF Localizer** → `odom_filtered` → **Navigation Controller (FSM + gains)** → `cmd_vel` → robot base.

High‑level logic runs on the Raspberry Pi 4 (ROS), while the Raspberry Pi Pico handles motor and actuator control.

Common frames: `map`, `odom`, `base_link`. The controller uses TF to convert goals/paths from `map` → `odom`.

---

## Work in Progress (WIP)

This project is still under active development:

* A high‑level planner for trajectory generation is missing (to be integrated).
* A driver node for communication between the Raspberry Pi 4 and the Raspberry Pi Pico is planned.

---

## Package‑by‑Package Details

### 1) `conf/`

Centralized launch wrappers and parameter files. Instead of launching packages individually, `conf/` provides unified entry points:

* `wake_up_fdpo.launch`: full system bring‑up (drivers + localization + controller + HMI).
* Other YAMLs and launch files unify parameter namespaces.

**Role:** ensures consistency between real robot and simulation.

---

### 2) `drivers_stack/lidar_drivers`

The system supports two LiDAR drivers that can be selected at launch time:

#### a) `ydlidar_x4_drivers` (5dpo_driver_laser_2d) — **default**

YDLidar X4 driver. Handles serial communication via `/dev/ttyUSB0` (USB), publishes `sensor_msgs/LaserScan`.

**Serial:** `/dev/ttyUSB0` @ 128000 baud  
**Outputs:** `laser_scan_point_cloud` (sensor_msgs/PointCloud)

**Note:** The YDLidar driver publishes PointCloud format, which is processed directly by the beacon detector (no conversion needed).

#### b) `hls_lfcd_lds_driver`

HLDS HLS-LFCD2 driver (LDS‑01/02). Handles serial communication via `/dev/ttyAMA0` (Raspberry Pi GPIO UART), publishes `sensor_msgs/LaserScan`.

**Serial:** `/dev/ttyAMA0` @ 230400 baud  
**Outputs:** `scan` (sensor_msgs/LaserScan) → remapped to `/base_scan`

**Selection:** Use `lidar_driver:=hlds` argument to switch from default YDLidar X4 to HLS-LFCD2 (see Quick Start section).

**Importance:** entry point for environment perception.

---

### 3) `drivers_stack/pi_pico_driver`

Driver node for communication between the Raspberry Pi 4 (high-level ROS control) and the Raspberry Pi Pico (low-level actuator control).

**Inputs:** `/cmd_vel` (geometry_msgs/Twist)  
**Outputs:** `/odom` (nav_msgs/Odometry)

---

### 4) `navigation_stack/localizer`

The localizer stack provides LiDAR‑based beacon detection and EKF fusion. It is composed of two nodes: **`beacon_detector_node`** and **`localizer_node` (EKF)**, and three custom messages.

#### a) Custom Messages

* **`localizer/Pose`**: `(float32 x, float32 y)` in the node's frame.
* **`localizer/Cluster`**: `string beacon_match_name`, `Pose[] points`, `Pose centroid`, `uint32 num_points`.
* **`localizer/BeaconMatch`**: `std_msgs/Header header`, `Cluster[] clusters`.

#### b) `beacon_detector_node`

**Input flexibility:** Supports two input types (configured automatically based on LiDAR driver):
* **LaserScan mode** (`input_topic_type: laser_scan`): subscribes to `/base_scan` (`sensor_msgs/LaserScan`) — used with HLS-LFCD2
* **PointCloud mode** (`input_topic_type: point_cloud`): subscribes to `laser_scan_point_cloud` (`sensor_msgs/PointCloud`) — used with YDLidar X4

* **Publish:**
  * `beacon_estimation` (`localizer/BeaconMatch`) — beacon detection and matching results.
  * `dbscan_markers` (`visualization_msgs/MarkerArray`) — raw clustered points + centroids.
  * `beacons_map_markers` (`visualization_msgs/MarkerArray`, latched) — fixed beacons in robot frame.

**Processing pipeline:**
1. Convert input data → 2D points (from LaserScan or PointCloud).  
2. **DBSCAN** clustering with `eps` and `minPoints`.  
3. **Nearest‑neighbour matching** between cluster centroids and beacon map.  
4. **Centroid bias compensation** (see equations below).  

---

### Centroid Correction — Equations

(1)  

$$
s = \frac{L}{2R}, \quad \alpha = \arcsin(\mathrm{clip}(s, -1, 1)), \quad L = |p_1 - p_n|
$$

(2)  

$$
\bar{d}_c = \frac{1}{2}\left(|p_c - p_1| + |p_c - p_n|\right)
$$

(3)  

$$
\mathrm{rad} = R^2 \cos^2 \alpha - \left(R^2 - \bar{d}_c^2\right)
$$

(4)  

$$
\Delta r = R \cos \alpha + \sqrt{\max(\mathrm{rad}, 0)}
$$

With $p_{meas}$ being the measured beacon position and  
$\hat{u}_r$ the unit radial vector:

(5)  

$$
p_{\text{corr}} = p_{\text{meas}} + \Delta r \cdot \hat{u}_r, \quad 
\hat{u}_r = \frac{p_{\text{meas}}}{|p_{\text{meas}}|}
$$

**Notes on centroid correction:**  
* Equation (1)–(2): compute arc span and raw centroid distances.  
* Equation (3)–(4): correct radial bias assuming cylindrical beacons of radius *R*.  
* Equation (5): apply correction along the radial unit vector.  
* Parameters `eps`, `minPoints`, `max_match_dist`, and beacon `radius` strongly affect matching performance.  
* Works best when beacons are observed as partial arcs in the LiDAR scan.  
* Incorrect radius values or large occlusions can lead to over- or under-correction.  

---

#### c) `localizer_node` (EKF)

* **Subscribe:**
  * `/odom` (`nav_msgs/Odometry`) → prediction step.
  * `/beacon_estimation` (`localizer/BeaconMatch`) → update step.  
* **State:** $X = [x, y, \theta]$, with covariance $P \in \mathbb{R}^{3 \times 3}$.  
* **Prediction:** motion model with noise.  
* **Measurement model:**

$$
h(X) =
\begin{bmatrix}
r \\
\beta
\end{bmatrix},
\quad
r = \sqrt{(x_b - x)^2 + (y_b - y)^2},
\quad
\beta = \mathrm{atan2}(y_b - y,\, x_b - x) - \theta
$$



* **Main EKF Equations:**

Prediction:  

$$
X_k^- = f(X_{k-1}, u_k)
$$

Covariance prediction:

$$
P_k^- = F_k P_{k-1} F_k^T + Q_k
$$


Kalman gain:  

$$
K_k = P_k^- H_k^T (H_k P_k^- H_k^T + R)^{-1}
$$


State update:  

$$
X_k = X_k^- + K_k (z_k - h(X_k^-))
$$


Covariance update:  

$$
P_k = (I - K_k H_k) P_k^-
$$

* **Outputs:** `odom_filtered` and TF `map → odom`.

---

### 5) `navigation_stack/navigation_controller`

Implements two FSM layers and exposes a service API for control.

#### a) Data structures

* **WayPoint:** `{id, pose{x,y,θ}, align, backwards}`  
* **Dynamic Reconfigure:** `v_nom, w_nom, kp_linear, kp_angular, arrive_radius, yaw_tol`  

#### b) High‑level FSM

Service: `~/control` (`NavigationControl.srv`).  
`command ∈ {start, pause, unpause, stop}`.

#### c) Low‑level FSM (GoToXYθ)

States: `idle → driveToGoal → turnToFinalYaw → done`.



#### Control Laws

Errors:  

$$
e_p =
\begin{bmatrix}
x_d - x \\\\
y_d - y
\end{bmatrix},
\quad
e_\theta = \mathrm{wrap}(\theta_d - \theta)
$$

Angular velocity:  

$$
\omega = \mathrm{clip}(k_\theta e_\theta, -\omega_{nom}, +\omega_{nom})
$$

Linear velocity:  

$$
v = \sigma \cdot \min(k_p \, |e_p|, v_{nom}), \quad \sigma \in \{ +1, -1 \}
$$

Yaw gate (optional):  

$$
g(e_\theta) = \max(0, 1 - |e_\theta| / \theta_{gate})
$$

Arrival conditions:  

$$
|p_d - p_c| \le r_{arrive}, \quad |\theta_d - \theta_c| \le \theta_{tol}
$$

---

### 6) `simulation_stage`

* `factory_floor.world`: Stage world map  
* `run_sim_stage.launch`: launches Stage + stack  

---

### 7) `utilities_stack`

#### a) `path_log`

Node `odoms_to_paths_node` converts odometry into `nav_msgs/Path` for RViz visualization.

#### b) `pointcloud_converter`

*(Currently unused - kept for potential future use)*

Simple Python node that converts `sensor_msgs/PointCloud` (v1) to `sensor_msgs/PointCloud2`. 

**Note:** The beacon detector now accepts PointCloud directly, so this converter is not needed in the current pipeline.

---

## Tutorials

A) **Full Simulation**  
```bash
roslaunch simulation_stage run_sim_stage.launch
```

B) **Beacon Localization Only**  
```bash
roslaunch localizer run_beacon_detector.launch
```

C) **EKF Localizer Only**  
```bash
roslaunch localizer run_ekf_localizer.launch
```

D) **Navigation Controller Only**  
```bash
roslaunch navigation_controller run_navigation_controller.launch
```

E) **Real Robot Bring‑up**  
```bash
# With YDLidar X4 (default, without RViz)
roslaunch conf wake_up_fdpo.launch

# With HLS-LFCD2
roslaunch conf wake_up_fdpo.launch lidar_driver:=hlds

# Enable RViz for visualization
roslaunch conf wake_up_fdpo.launch use_rviz:=true

# Combine options
roslaunch conf wake_up_fdpo.launch lidar_driver:=hlds use_rviz:=true
```

---

## Coordinate Frames and Transforms

### Frame Hierarchy

The system uses the standard ROS navigation frame structure:

```
map (global, fixed)
  └─ odom (locally consistent, drifts over time)
      └─ base_link (robot center)
          └─ laser (LiDAR sensor)
```

### Frame Descriptions

| Frame | Description | Published By |
|-------|-------------|--------------|
| **`map`** | Global fixed reference frame | EKF localizer (via TF `map→odom`) |
| **`odom`** | Odometry frame (locally consistent, accumulates drift) | Pi Pico Driver |
| **`base_link`** | Robot's base center | Pi Pico Driver (via TF `odom→base_link`) |
| **`laser`** | LiDAR sensor frame | Static TF publisher (via TF `base_link→laser`) |

### Initial Robot Pose in Map Frame

Configure the robot's starting position in `conf/launch/navigation/localizer/ekf_params.yaml`:

```yaml
ekf_params:
  initial_pose:
    x: 0.0        # meters
    y: 0.0        # meters
    theta: 0.0    # radians (0 = facing +X, π/2 = facing +Y)
```
### LiDAR Frame Transform (base_link → laser)

The LiDAR position relative to the robot is configured in `conf/launch/hardware/lidar_selector.launch`:

```xml
<arg name="laser_xyz" default="0 0 0.15"/>   <!-- X Y Z in meters -->
<arg name="laser_rpy" default="0 0 0"/>      <!-- Roll Pitch Yaw in radians -->
```


### Verify Transforms

```bash
# Check TF tree structure
rosrun tf view_frames
# Generates frames.pdf with visual TF tree

# Check specific transform
rosrun tf tf_echo base_link laser
rosrun tf tf_echo map base_link

# Monitor all transforms
rosrun rqt_tf_tree rqt_tf_tree
```

---

## Remote Visualization with RViz

The system runs on a **Raspberry Pi 4 without display**. To visualize all topics (beacons, clusters, laser scans, paths), use **RViz on your PC**.

### Setup

**On your PC (with GUI):**

```bash
# Set ROS Master URI to point to the Raspberry Pi 4
export ROS_MASTER_URI=http://10.242.202.243:11311
export ROS_IP=$(hostname -I | awk '{print $1}')

# Launch RViz
rviz
```

### Beacon Detector Visualization Topics

The `beacon_detector_node` publishes the following **latched** topics for visualization:

| Topic | Type | Description |
|-------|------|-------------|
| `/dbscan_markers` | `MarkerArray` | Raw clustered points + centroids (red/green) |
| `/beacons_map_markers` | `MarkerArray` | Fixed beacon positions in map frame |
| `/beacon_estimation` | `localizer/BeaconMatch` | Matched beacons data |

### RViz Configuration

**Add these displays in RViz:**

1. **MarkerArray** → Topic: `/dbscan_markers`
   - Shows DBSCAN clustering results in real-time

2. **MarkerArray** → Topic: `/beacons_map_markers`
   - Shows known beacon positions (latched, appears immediately)

3. **MarkerArray** → Topic: `/viz_mux/markers`
   - Shows all beacon associations

4. **Path** → Topics:
   - `/odom/path` — Odometry path
   - `/odometry/filtered/path` — EKF filtered path

5. **TF** → Show all transforms

6. **Map** → Topic: `/map` (if map_server is running)

### Check Topics Availability

```bash
# List all visualization topics
rostopic list | grep -E "markers|path|scan"

# Check if topics are being published
rostopic hz /dbscan_markers
rostopic hz /beacons_map_markers

# Echo a topic to verify data
rostopic echo /dbscan_markers -n 1
```

### Troubleshooting Remote Visualization

**Topics not appearing in RViz:**
```bash
# On Pi4: Check if topics are being published
rostopic list

# On PC: Verify ROS connectivity
rostopic list
# Should show the same topics as Pi4

# Test network connection
ping 10.242.202.243
```

**Markers not showing:**
- Markers are **latched** now, so they persist even if RViz connects later
- Make sure the **Fixed Frame** in RViz is set to `map` or `base_link`
- Check if beacon_detector is receiving LiDAR data: `rostopic hz /laser_scan_point_cloud`

**Quick Verification on Pi4:**
```bash
# 1. Verify beacon_detector is subscribed to correct topic
rosnode info /beacon_detector_node
# Expected for YDLidar: /laser_scan_point_cloud [sensor_msgs/PointCloud]
# Expected for HLS-LFCD2: /base_scan [sensor_msgs/LaserScan]

# 2. Check YDLidar is publishing data
rostopic hz /laser_scan_point_cloud
rostopic echo /laser_scan_point_cloud -n 1

# 3. Verify beacon_detector is processing and publishing
rostopic list | grep -E "dbscan|beacons|estimation"
rostopic hz /beacon_estimation

# 4. Verify markers are available (latched, should return immediately)
rostopic echo /dbscan_markers -n 1
rostopic echo /beacons_map_markers -n 1

# 5. If markers are empty, check beacon_detector logs
rosnode list
rosnode info /beacon_detector_node
```

**Important:** After recompiling, always run:
```bash
source devel/setup.bash
```
Before launching the system!

---

## Debugging & Troubleshooting

* `rqt_graph`, `rqt_reconfigure`, `rostopic echo`  
* **Problems:** missing `source`, Stage paths, LiDAR serial, TF conflicts  

---

## License

Not defined yet — confirm before external use.

---

## Credits

Developed by **Electrical and Computer Engineering students at FEUP students at FEUP** for **Robot Factory 4.0** (National Robotics Festival 2025):

* *Afonso Mateus*  
* *Christian Geyer*  
* *Daniel Silva*  
* *Pedro Lopes*  
