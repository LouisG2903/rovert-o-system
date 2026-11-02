# Rovert-O System Architecture (Master Spec)

## Platform
| Item | Value |
|---|---|
| OS | Ubuntu 24.04 LTS |
| ROS | ROS 2 Jazzy Jalisco |
| Simulator | Gazebo Harmonic (`ros_gz`) |
| Onboard compute | Raspberry Pi 4B (LiDAR processing, `slam_toolbox`, reactive safety overrides; **no Nav2**) |
| Microcontroller | ESP32 (motor actuation, wheel encoders, ESP-NOW telemetry) |
| Sensor | LD19 2D LiDAR |

Design constraints: lightweight stack. No `robot_localization`, no `cmd_vel_mux`.

## TF Tree
```
map -> odom -> base_footprint -> base_link -> laser_frame
```
- `map -> odom`: `slam_toolbox`
- `odom -> base_footprint`: DiffDrive (sim) / wheel odometry node (hardware)
- `base_footprint -> base_link -> laser_frame` (+ wheel links): `robot_state_publisher` from the xacro

## Single Source of Calibration
`roverto_description/config/rover_calibration.yaml` is the only place for physical constants
(wheel radius/offset, slip factor, wheel separation, LiDAR pose, `ticks_per_meter`,
tick clamp, safety distances, speeds). The xacro loads it via `xacro.load_yaml`.

## Binary UART Protocol (ESP32 <-> Pi)
115200 baud, Little-Endian, no padding (`struct` format strings start with `<`).
`xor_checksum` = XOR of every preceding byte in the packet (header bytes included).

### Packet 1: ESP32 -> Pi, 12 bytes, `<BBiiBB`
| Field | Type | Notes |
|---|---|---|
| sync1 | uint8 | `0xAA` |
| sync2 | uint8 | `0x55` |
| left_ticks | int32 | cumulative |
| right_ticks | int32 | cumulative |
| rover_status | uint8 | 0=STANDBY, 1=EXECUTING, 2=AVOIDING, 3=GOAL_REACHED |
| xor_checksum | uint8 | |

### Packet 2: Pi -> ESP32, 4 bytes, `<BBBB`
| Field | Type | Notes |
|---|---|---|
| sync1 | uint8 | `0xAA` |
| sync2 | uint8 | `0x66` |
| obstacle_state | uint8 | 0=CLEAR, 1=AVOID_LEFT, 2=AVOID_RIGHT, 3=STOP, 4=HOLD_HEADING |
| xor_checksum | uint8 | |

### Packet 3: Pi -> ESP32, variable (<= 206 bytes), header `<BBBBBB` + payload + checksum
| Field | Type | Notes |
|---|---|---|
| sync1 | uint8 | `0xAA` |
| sync2 | uint8 | `0x77` |
| map_seq_id | uint8 | |
| chunk_id | uint8 | |
| total_chunks | uint8 | |
| payload_len | uint8 | <= 198 |
| payload | bytes | `payload_len` bytes |
| xor_checksum | uint8 | |

Note: 6 header + 198 payload + 1 checksum = 205 bytes at maximum payload (within the 206 B budget).

## Workspace Packages (`roverto_ws/src`)
| Package | Type | Role |
|---|---|---|
| `roverto_description` | ament_cmake | URDF/Xacro + `config/rover_calibration.yaml` |
| `roverto_gazebo` | ament_cmake | Worlds, `ros_gz_bridge` config, sim launch |
| `roverto_navigation` | ament_cmake | `slam_toolbox` config + launch |
| `roverto_bringup` | ament_cmake | Top-level launch (`sim_mission`) |
| `roverto_autonomy` | ament_python | `lidar_guard_node`, `sim_esp32_emulator_node`, `esp32_bridge_node`, `map_compressor_node` |

## Simulation Topic Contract
| ROS topic | Type | Direction | Producer |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | GZ -> ROS | Gazebo |
| `/scan` | `sensor_msgs/LaserScan` | GZ -> ROS | `gpu_lidar` (frame `laser_frame`) |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS -> GZ | DiffDrive |
| `/odom` | `nav_msgs/Odometry` | GZ -> ROS | DiffDrive (`odom` -> `base_footprint`) |
| `/tf` | `tf2_msgs/TFMessage` | GZ -> ROS | DiffDrive |
| `/joint_states` | `sensor_msgs/JointState` | GZ -> ROS | JointStatePublisher |
