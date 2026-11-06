# Rovert-O System

Welcome to the central repository for **Rovert-O**, a semi-autonomous subterranean exploration and 2D mapping rover designed for the **EDSE** course.

---

## 1. Project Overview (Rovert-O System)

The mission of **Rovert-O** is to autonomously traverse and map an analog Martian lava tube environment. Because subterranean caves block GPS signals and make continuous high-bandwidth Wi-Fi teleoperation unreliable, the rover operates **semi-autonomously**:

1. An operator at the Ground Station sends a high-level distance command (e.g., *"Advance 5.0 meters into the tunnel"*).
2. The rover drives forward, uses its 2D LiDAR (`LD19`) to detect and dodge rocks or cave walls on its own, and realigns itself with the tunnel heading after clearing each obstacle.
3. As it moves, it builds a 2D occupancy grid map of the cave using Simultaneous Localization and Mapping (`slam_toolbox`).
4. Periodically—and automatically upon reaching its target distance—the rover compresses the 2D map into tiny binary packets (`<= 205 bytes` each) and beams them back to the Ground Station over **ESP-NOW** so the team can reconstruct and view the cave map without needing a Wi-Fi network.

### Target Software Stack
* **Operating System:** Ubuntu 24.04 LTS (Noble Numbat)
* **Middleware:** ROS 2 Jazzy Jalisco
* **Physics & Sensor Simulator:** Gazebo Harmonic (`ros_gz`)
* **Microcontroller Framework:** PlatformIO (Arduino / ESP32)

---

## 2. System Architecture & Division of Labor

To keep the system deterministic, lightweight, and immune to Linux CPU spikes on the Raspberry Pi 4, we split responsibilities across three hardware components:

1. **Ground Station (Laptop + Ground ESP32):** Sends high-level distance goals (e.g., *"Advance X meters"*) via ESP-NOW and runs the Python decoder (`gs_map_decoder.py`) to stitch incoming radio packets into a 2D PNG map of the cave.
2. **Onboard ESP32 (Real-Time Motor & Encoder Controller):** Handles hard real-time tasks—reading quadrature wheel encoders, driving the H-bridge motor controllers, managing the ESP-NOW radio link, and executing the local motion state machine (`STANDBY`, `EXECUTING`, `AVOIDING`, `GOAL_REACHED`).
3. **Raspberry Pi 4 Model B (ROS 2 Companion Computer):** Runs Ubuntu 24.04 and ROS 2 Jazzy. It reads the `LD19` 2D LiDAR over USB, evaluates immediate obstacle hazards (`lidar_guard_node`), builds the 2D map (`slam_toolbox`), compresses the map (`map_compressor_node`), and communicates with the onboard ESP32 over a `115200` baud binary UART link (`esp32_bridge_node`).

### Why Decouple Motor Control from ROS 2 Mapping?
Running full path-planning stacks like `Nav2` on a Raspberry Pi 4 alongside SLAM can cause CPU throttling and delayed motor commands. Instead, the **ESP32 owns the wheels** and tracks longitudinal progress, while the **Raspberry Pi acts as the "eyes" and "cartographer"**—telling the ESP32 whether the path ahead is clear or which way to turn to avoid an obstacle.

### System Data-Flow Diagram

```text
+-----------------------------------------------------------------------+
|                           GROUND STATION                              |
|  [Operator CLI / GUI]  <-- (USB Serial) -->  [Ground Station ESP32]   |
|   (gs_map_decoder.py)                                   ^             |
+---------------------------------------------------------|-------------+
                                                          |
                                               ESP-NOW Wireless Link
                                           (Commands Up / Map Chunks Down)
                                                          |
+---------------------------------------------------------v-------------+
|                         ONBOARD ESP32 (Firmware)                      |
|  - Executes "Advance D meters" state machine                          |
|  - Drives 4x skid-steer motors & counts wheel encoder ticks           |
+-----------------------------------------------------------------------+
                  |                                       ^
     Packet 1 (12B @ 20-50Hz)                Packet 2 (4B Obstacle State)
   [left_ticks, right_ticks, status]         Packet 3 (<=205B Map Chunks)
                  |           115200 Baud UART            |
+-----------------v---------------------------------------|-------------+
|                    RASPBERRY PI 4B (ROS 2 Jazzy)        |             |
|                                                         |             |
|  [LD19 LiDAR] --> /scan --> [lidar_guard_node] --> /obstacle_state    |
|                     |                                   ^             |
|                     v                                   |             |
|  [esp32_bridge_node] ---> /odom & /tf ---> [slam_toolbox]             |
|          ^                                      |                     |
|          |                                      v                     |
|   /compressed_map_chunk <----------- [map_compressor_node] <--- /map  |
+-----------------------------------------------------------------------+
```

### Coordinate Frames (TF Tree)
```text
map -> odom -> base_footprint -> base_link -> laser_frame (+ 4x wheel_links)
```
* `map -> odom`: Published by `slam_toolbox` (corrects odometry drift using LiDAR scan-matching).
* `odom -> base_footprint`: Published by Gazebo's `DiffDrive` plugin in simulation, or by `esp32_bridge_node` on the physical rover.
* `base_footprint -> base_link -> laser_frame`: Published by `robot_state_publisher` from the URDF model.

---

## 3. Repository Structure

```text
rovert-o-system/
├── README.md                          # Project documentation
├── docs/
│   └── ARCHITECTURE.md                # Master specification (TF tree, UART packet bytes, topics)
├── ground_station_gui/
│   └── gs_map_decoder.py              # Standalone library & CLI tool to decode Packet 3 into PNG maps
├── firmware/
│   ├── esp32_onboard/                 # PlatformIO project for the rover's onboard ESP32
│   └── esp32_ground_station/          # PlatformIO project for the Ground Station ESP32 transceiver
└── roverto_ws/                        # ROS 2 Jazzy workspace
    └── src/
        ├── roverto_description/       # Robot 3D model (URDF/Xacro) & unified calibration YAML
        │   ├── config/
        │   │   └── rover_calibration.yaml
        │   └── urdf/
        │       └── roverto.urdf.xacro
        ├── roverto_gazebo/            # Gazebo Harmonic simulation world & ROS-Gazebo topic bridge
        │   ├── config/
        │   │   └── gazebo_bridge.yaml
        │   ├── launch/
        │   │   └── roverto_gazebo.launch.xml
        │   └── worlds/
        │       └── tunnel.sdf         # 10m x 1.5m corridor with wall ribs & 2 cylindrical obstacles
        ├── roverto_navigation/        # 2D SLAM configuration (slam_toolbox, no Nav2)
        │   ├── config/
        │   │   └── mapper_params_online_async.yaml
        │   └── launch/
        │       └── slam.launch.xml
        ├── roverto_autonomy/          # Core Python autonomy, compression, and hardware bridge nodes
        │   ├── roverto_autonomy/
        │   │   ├── calibration.py             # Helper to load rover_calibration.yaml
        │   │   ├── protocol.py                # Binary struct packing/unpacking & XOR checksums
        │   │   ├── lidar_guard_node.py        # Reactive sector-based obstacle classifier
        │   │   ├── sim_esp32_emulator_node.py # Emulates the ESP32 state machine in Gazebo
        │   │   ├── map_compressor_node.py     # Crops, 2-bit quantizes, and zlib-compresses /map
        │   │   └── esp32_bridge_node.py       # Hardware UART driver & wheel odometry integrator
        │   └── test/
        │       └── test_protocol_and_bridge.py # Pytest suite for packets, clamp, and loopback
        └── roverto_bringup/           # Top-level launch files for simulation and physical hardware
            └── launch/
                ├── sim_mission.launch.xml     # Launches full Gazebo + SLAM + Autonomy stack
                └── real_rover.launch.xml      # Launches Raspberry Pi 4 physical rover stack
```

---

## 4. How the Autonomy & Mapping Pipeline Works

### 4.1 Single Source of Calibration (`rover_calibration.yaml`)
All physical measurements live in **one** file: `roverto_description/config/rover_calibration.yaml`.
* **What it stores:** `wheel_radius` (`0.0325 m`), `wheel_y_offset` (`0.1135 m`), `slip_factor` (`1.0`), `wheel_separation` (`0.227 m`), LiDAR mount offsets (`lidar_xyz`, `lidar_rpy`), encoder `ticks_per_meter` (`1000.0`), sanity clamp `max_ticks_per_cycle` (`500`), and safety distances (`front_stop_distance: 0.45 m`, `flank_safe_distance: 0.30 m`).
* **Why it matters:** Both the 3D robot model (`roverto.urdf.xacro`) and all Python autonomy nodes read this file at startup. If the mechanical team changes wheel sizes or moves the LiDAR mount, you only edit this single YAML file.

### 4.2 Reactive Obstacle Avoidance (`lidar_guard_node`)
Instead of a heavy global planner, `lidar_guard_node` inspects every `/scan` message from the 2D LiDAR at 10 Hz using an index-agnostic `atan2` angle wrapper (`0.0 rad` = straight ahead `+X`, positive = Left `+Y`, negative = Right `-Y`) and divides the surroundings into sectors:
* **Front Cone (`-30°` to `+30°`):** Blocked if the closest laser return is `< 0.45 m` (`front_stop_distance`).
* **Left (`+15°` to `+80°`) & Right (`-80°` to `-15°`) Assessment Sectors:** Used to compare average open clearance on the left vs. right when the front cone is blocked.
* **Left (`+35°` to `+105°`) & Right (`-105°` to `-35°`) Flank Guard Sectors:** Occupied if any obstacle is `< 0.30 m` (`flank_safe_distance`) beside the rover.

It publishes a single byte on `/obstacle_state` (`std_msgs/msg/UInt8`):
* `0` (`CLEAR`): Path ahead and flanks are clear. Drive forward and smoothly steer back toward the initial tunnel heading `yaw0`.
* `1` (`AVOID_LEFT`): Front blocked, more room on the left. Turn in place left.
* `2` (`AVOID_RIGHT`): Front blocked, more room on the right. Turn in place right.
* `3` (`STOP`): Front and both sides are boxed in (`< 0.30 m`). Halt motors immediately.
* `4` (`HOLD_HEADING`): Front is clear, but an obstacle is close to the side flank. Drive straight along the current deflected angle without turning back toward `yaw0` yet (prevents side-swiping the obstacle while passing it).

> **Anti-Dither Latch:** Once the guard picks a turn direction (`AVOID_LEFT` or `AVOID_RIGHT`), it latches that choice until the front cone has been completely clear for `0.5 s` (`avoid_release_sec`), preventing the rover from oscillating left and right at obstacle edges. Furthermore, after turning, the rover drives straight for at least `0.35 m` (`pass_distance`) to clear the obstacle laterally before realigning with the tunnel heading.

### 4.3 2D SLAM (`slam_toolbox`)
Configured in `roverto_navigation/config/mapper_params_online_async.yaml`, `slam_toolbox` runs in asynchronous online mapping mode at `0.05 m` (5 cm/pixel) resolution. It fuses wheel odometry (`/odom`) with 450-ray `LD19` laser scans (`/scan`) to publish the `/map` (`nav_msgs/msg/OccupancyGrid`) and the drift-correcting `map -> odom` transform.

### 4.4 Map Compression (`map_compressor_node`) & Ground Station Decoder (`gs_map_decoder.py`)
A raw ROS 2 `OccupancyGrid` is thousands of bytes—too large for a single ESP-NOW radio frame (`<= 250 bytes`). Whenever `/rover_status` transitions to `3` (`GOAL_REACHED`), or every `10.0 seconds` while the map is changing, `map_compressor_node` runs a 4-stage pipeline:
1. **Auto-Crop:** Trims away unused outer `-1` (Unknown) borders to the tightest bounding box containing known cells.
2. **2-Bit Quantization:** Converts each cell from an 8-bit integer into 2 bits (`00` = Unknown `-1`, `01` = Free `0..49`, `10` = Occupied `50..100`) and packs **4 cells into every byte** (MSB first).
3. **Metadata Header + `zlib` Level-9 Compression:** Prepends a 10-byte header (`<HHHhh`: `width`, `height`, `resolution_mm`, `origin_x_cm`, `origin_y_cm`) and compresses the buffer with `zlib`. In our 10-meter lava tube test, a 6,766-byte grid compresses down to **326 bytes** (**2 ESP-NOW chunks**).
4. **Packet 3 Framing:** Slices the compressed bytes into chunks of at most `198 bytes` and publishes them on `/compressed_map_chunk`.

On the Ground Station laptop, `ground_station_gui/gs_map_decoder.py` verifies each chunk's XOR checksum, reassembles the chunks by `map_seq_id`, decompresses the `zlib` stream, and renders a metric-axes PNG image (`Unknown` = gray, `Free` = white, `Occupied` = black).

### 4.5 Binary UART Protocol (`esp32_bridge_node`)
All communication between the Raspberry Pi 4 and the onboard ESP32 uses packed **Little-Endian** binary frames at `115200` baud with an **XOR checksum** (the XOR of all preceding bytes in the frame, including the `0xAA` sync header):

| Packet | Direction | Size | Struct Format | Fields |
| :--- | :--- | :--- | :--- | :--- |
| **Packet 1** | ESP32 $\rightarrow$ Pi | `12 B` | `<BBiiBB` | `0xAA`, `0x55`, `int32 left_ticks`, `int32 right_ticks`, `uint8 rover_status` (`0..3`), `uint8 xor_checksum` |
| **Packet 2** | Pi $\rightarrow$ ESP32 | `4 B` | `<BBBB` | `0xAA`, `0x66`, `uint8 obstacle_state` (`0..4`), `uint8 xor_checksum` |
| **Packet 3** | Pi $\rightarrow$ ESP32 | `<= 205 B` | `<BBBBBB` + payload + `<B` | `0xAA`, `0x77`, `uint8 map_seq_id`, `uint8 chunk_id`, `uint8 total_chunks`, `uint8 payload_len` (`<=198`), `payload`, `uint8 xor_checksum` |

* **Sanity Tick Clamp:** If the ESP32 resets or experiences electrical noise causing a single-cycle encoder jump larger than `max_ticks_per_cycle` (`500` ticks), `esp32_bridge_node` logs a warning and clamps the delta to `0` for that cycle so the SLAM map is never corrupted by a teleporting odometry spike.
* **Paced Map Transmission:** Outgoing Packet 3 chunks are queued and written to the UART with a minimum `25 ms` interval between chunks so the ESP32's serial/ESP-NOW buffers never overflow.

---

## 5. Quickstart: Running the Simulation & Watching SLAM in RViz2

Make sure you are on **Ubuntu 24.04** with **ROS 2 Jazzy** and **Gazebo Harmonic** (`ros-jazzy-ros-gz`, `ros-jazzy-slam-toolbox`, `ros-jazzy-xacro`) installed.

> **RViz2 is automatic:** `ros2 launch roverto_bringup sim_mission.launch.xml` now opens RViz2 pre-configured (`slam_view.rviz`) with `/map`, `/scan`, `TF`, and `RobotModel` (plus `/odom`). Pass `use_rviz:=false` for headless runs (e.g. `headless:=true use_rviz:=false`). Step 3 below is only needed if you disabled it.

### Step 1: Build the Workspace
Open a terminal and build all packages using `--symlink-install` (so Python edits take effect without rebuilding):
```bash
cd ~/rovert-o-system/roverto_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Step 2: Launch the Full Simulation (`Terminal 1`)
You can launch the simulation **with the Gazebo 3D GUI**:
```bash
source /opt/ros/jazzy/setup.bash
source ~/rovert-o-system/roverto_ws/install/setup.bash
ros2 launch roverto_bringup sim_mission.launch.xml
```
*(Or, if you are running on a headless machine/SSH or want faster performance without a 3D window, append `headless:=true`):*
```bash
ros2 launch roverto_bringup sim_mission.launch.xml headless:=true
```
Wait ~15–20 seconds for Gazebo, `slam_toolbox`, `lidar_guard_node`, `sim_esp32_emulator_node`, and `map_compressor_node` to initialize.

### Step 3: Open RViz2 to Watch Live Mapping (`Terminal 2`)
Open a second terminal and launch RViz2 with simulation time enabled:
```bash
source /opt/ros/jazzy/setup.bash
source ~/rovert-o-system/roverto_ws/install/setup.bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```
Inside RViz2:
1. In the left **Displays** panel, change **Global Options $\rightarrow$ Fixed Frame** to `map`.
2. Click **Add** (bottom left) $\rightarrow$ **By topic** and add:
   * `/map` (`Map`) — expand `/map` $\rightarrow$ **Topic** $\rightarrow$ set **Durability Policy** to `Transient Local`.
   * `/scan` (`LaserScan`) — increase **Size (m)** to `0.03` so the red laser dots are easy to see.
3. Click **Add** $\rightarrow$ **By display type** and add **TF** and **RobotModel** (set **Description Topic** to `/robot_description`).

### Step 4: Start the Ground Station Map Decoder (`Terminal 3`)
Before commanding the rover to move, start the Ground Station decoder in a third terminal so it listens for compressed Packet 3 chunks on `/compressed_map_chunk`:
```bash
source /opt/ros/jazzy/setup.bash
source ~/rovert-o-system/roverto_ws/install/setup.bash
python3 ~/rovert-o-system/ground_station_gui/gs_map_decoder.py --ros -o /tmp/reconstructed_lava_tube_map.png
```

### Step 5: Send an "Advance 5.0 Meters" Mission Command (`Terminal 4`)
In a fourth terminal, send a target distance of `5.0` meters to `/gs_command`:
```bash
source /opt/ros/jazzy/setup.bash
source ~/rovert-o-system/roverto_ws/install/setup.bash
ros2 topic pub -1 /gs_command std_msgs/msg/Float32 "{data: 5.0}"
```
**What you will see:**
* The rover drives down the 10-meter corridor, dodges the first cylindrical boulder at `x = 2.0 m`, dodges the second boulder at `x = 4.0 m`, realigns with the tunnel axis, and stops at `x = 5.0 m` (`GOAL_REACHED`).
* Terminal 3 (`gs_map_decoder.py`) will print the decoded map metadata and save `/tmp/reconstructed_lava_tube_map.png`. You can open that image file to inspect the reconstructed cave map!

> **Stopping Cleanly:** Press `Ctrl+C` in each terminal. If a background Gazebo server ever lingers, clean it up with:
> ```bash
> pkill -f "[g]z sim"
> ```

---

## 6. Hardware Bringup (Raspberry Pi 4)

When running on the physical rover's Raspberry Pi 4, we do **not** run Gazebo or `sim_esp32_emulator_node`. Instead, `real_rover.launch.xml` starts:
* `robot_state_publisher` (`use_sim_time:=false`)
* `esp32_bridge_node` (connects to the ESP32 over `/dev/ttyAMA0` at `115200` baud, integrates wheel encoder ticks into `/odom` + TF `odom -> base_footprint`, and sends Packets 2 & 3)
* `lidar_guard_node`
* `slam_toolbox` (`slam.launch.xml` with `use_sim_time:=false`)
* `map_compressor_node`

### Launching on the Raspberry Pi 4
```bash
source /opt/ros/jazzy/setup.bash
source ~/rovert-o-system/roverto_ws/install/setup.bash

# Without the LD19 driver (or if launching the LiDAR node separately):
ros2 launch roverto_bringup real_rover.launch.xml serial_port:=/dev/ttyAMA0

# With the LD19 LiDAR driver enabled (once ldlidar_stl_ros2 is built in the workspace):
ros2 launch roverto_bringup real_rover.launch.xml serial_port:=/dev/ttyAMA0 launch_lidar_driver:=true
```
*(You can also test the hardware launch file on your laptop without an ESP32 plugged in by passing `mock_serial:=true`).*

---

## 6.1 Laptop LiDAR Bench Test (LD19 / STL-19P over USB-UART)

`lidar_test.launch.xml` runs only the LiDAR driver, `robot_state_publisher` (URDF `laser_frame`), a fake identity `odom -> base_footprint` TF, `slam_toolbox` and RViz2. Move the LiDAR by hand to build the map. Do not combine it with `esp32_bridge_node`.

```bash
sudo usermod -aG dialout $USER   # once, then log out/in
source /opt/ros/jazzy/setup.bash
cd ~/rovert-o-system/roverto_ws && colcon build --symlink-install
source install/setup.bash
ros2 launch roverto_bringup lidar_test.launch.xml port_name:=/dev/ttyUSB0
# in another terminal:
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base_footprint laser_frame
ros2 topic echo /map --once --field info
```
On the Pi, `real_rover.launch.xml` takes `lidar_port:=<device>` together with `launch_lidar_driver:=true`.

---

## 7. Running Unit Tests

We have an automated `pytest` suite (`roverto_autonomy/test/test_protocol_and_bridge.py`) that verifies:
1. **Packet 1 Parser & Resync:** Byte-stream synchronization on `0xAA 0x55`, negative/positive `int32` tick unpacking, and automatic recovery from corrupted bytes or bad XOR checksums.
2. **Packet 2 & Packet 3 Framing:** Header formatting, payload length bounds (`<= 198 B`), and XOR checksum verification.
3. **2-Bit Map Compression & Decoding Roundtrip:** Cropping, 2-bit quantization, `zlib` compression, Packet 3 chunking, and exact cell-by-cell reconstruction via `MapDecoder`.
4. **Incomplete Map Reset Handling:** Verifies that `MapDecoder` logs a warning and cleanly resets if a new `map_seq_id` interrupts an unfinished map.
5. **ESP32 Bridge Mock Loopback:** Tests differential-drive odometry integration, the `max_ticks_per_cycle` sanity clamp on encoder spikes, Packet 2 output, and `25 ms` paced Packet 3 transmission.

Run the test suite with:
```bash
cd ~/rovert-o-system/roverto_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest src/roverto_autonomy/test/test_protocol_and_bridge.py -v
```
