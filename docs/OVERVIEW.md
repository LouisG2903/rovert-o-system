# Rovert-O System — Project Overview & Engineering Hub

Welcome to the central engineering checkpoint for **Rovert-O**, a semi-autonomous subterranean mapping rover built for the **EDSE** course. This document provides a high-level summary of the system architecture, hardware interface contracts, and project milestones across all sub-teams (Mechanics, Electronics/Firmware, and ROS 2 Autonomy).

---

## 1. Mission & System Overview

**Rovert-O** is designed to explore and map an analog Martian lava tube where GPS is unavailable and continuous Wi-Fi video/teleoperation is unreliable. Instead of manual remote control, the rover operates **semi-autonomously** using a decoupled three-node compute architecture:

* **Ground Station (Laptop + Ground ESP32):** Sends high-level longitudinal distance commands (e.g., *"Advance 5.0 meters"*) over **ESP-NOW** and runs `gs_map_decoder.py` to reconstruct incoming compressed radio chunks into a 2D PNG cave map.
* **Onboard ESP32 (Real-Time Motor, Encoder & Radio Controller):** Owns hard real-time actuation and sensing. It drives the 4-wheel skid-steer motors, counts quadrature wheel encoder ticks, manages the ESP-NOW wireless link, and executes the local motion state machine (`STANDBY`, `EXECUTING`, `AVOIDING`, `GOAL_REACHED`).
* **Raspberry Pi 4 Model B (ROS 2 Jazzy Companion Computer):** Owns perception and cartography without burdening motor control with Linux CPU spikes. It processes the `LD19` 2D LiDAR (`/scan`), classifies immediate obstacle hazards (`lidar_guard_node`), builds the 2D occupancy grid map (`slam_toolbox`), and compresses the map (`map_compressor_node`) into tiny binary frames sent to the ESP32 over UART (`esp32_bridge_node`).

---

## 2. Documentation Quick-Jump Links

* **[README.md](README.md)** — PlatformIO, VS Code, and Git/GitHub onboarding tutorial (Spanish).
* **[ROS2_DEVELOPMENT.md](ROS2_DEVELOPMENT.md)** — Full ROS 2 Jazzy, Gazebo Harmonic simulation, RViz2 visualization, and Raspberry Pi 4 guide.
* **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** — Master technical specification and frame contracts.

---

## 3. System Data-Flow Diagram

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

---

## 4. The Hardware Interface Contract (UART Protocol @ 115200 Baud, Little-Endian)

The Onboard ESP32 and Raspberry Pi 4 communicate over a 3.3V UART link (`/dev/ttyAMA0`, `115200` baud, `8N1`). All multi-byte integers are **Little-Endian** with no struct padding (`__attribute__((packed))` in C/C++, `<` prefix in Python `struct`).

Every packet ends with an **8-bit XOR checksum** (`xor_checksum`), calculated by XORing every preceding byte in the frame starting from byte `0` (`0xAA`).

### Packet 1: ESP32 $\rightarrow$ Raspberry Pi (Odometry & Status Telemetry)
* **Size:** `12 bytes` fixed (`<BBiiBB`)
* **Rate:** `20 Hz – 50 Hz`

| Byte Offset | Field | C Type | Value / Description |
| :--- | :--- | :--- | :--- |
| `0` | `sync1` | `uint8_t` | `0xAA` |
| `1` | `sync2` | `uint8_t` | `0x55` |
| `2..5` | `left_ticks` | `int32_t` | Cumulative left-side encoder ticks (signed) |
| `6..9` | `right_ticks` | `int32_t` | Cumulative right-side encoder ticks (signed) |
| `10` | `rover_status` | `uint8_t` | `0` = `STANDBY`<br>`1` = `EXECUTING`<br>`2` = `AVOIDING`<br>`3` = `GOAL_REACHED` |
| `11` | `xor_checksum` | `uint8_t` | XOR of bytes `0..10` |

### Packet 2: Raspberry Pi $\rightarrow$ ESP32 (Reactive Obstacle Override)
* **Size:** `4 bytes` fixed (`<BBBB`)
* **Rate:** Sent immediately on state change + `5 Hz` keepalive

| Byte Offset | Field | C Type | Value / Description |
| :--- | :--- | :--- | :--- |
| `0` | `sync1` | `uint8_t` | `0xAA` |
| `1` | `sync2` | `uint8_t` | `0x66` |
| `2` | `obstacle_state` | `uint8_t` | `0` = `CLEAR` (Drive forward & realign to initial heading)<br>`1` = `AVOID_LEFT` (Front blocked; turn in place left)<br>`2` = `AVOID_RIGHT` (Front blocked; turn in place right)<br>`3` = `STOP` (Front & flanks blocked; halt motors)<br>`4` = `HOLD_HEADING` (Flank occupied; drive straight along current angle) |
| `3` | `xor_checksum` | `uint8_t` | XOR of bytes `0..2` |

### Packet 3: Raspberry Pi $\rightarrow$ ESP32 (Compressed Map Chunk for ESP-NOW Downlink)
* **Size:** Variable, `<= 205 bytes` (`6B` header `<BBBBBB` + `payload_len` bytes + `1B` checksum)
* **Pacing:** Sent with a minimum `25 ms` interval between chunks so ESP32 UART/ESP-NOW buffers never overflow.

| Byte Offset | Field | C Type | Value / Description |
| :--- | :--- | :--- | :--- |
| `0` | `sync1` | `uint8_t` | `0xAA` |
| `1` | `sync2` | `uint8_t` | `0x77` |
| `2` | `map_seq_id` | `uint8_t` | Map sequence counter (`0..255`, increments per map snapshot) |
| `3` | `chunk_id` | `uint8_t` | Zero-based chunk index (`0 .. total_chunks - 1`) |
| `4` | `total_chunks` | `uint8_t` | Total chunks `N` in this `map_seq_id` |
| `5` | `payload_len` | `uint8_t` | Number of payload bytes in this frame (`1..198`) |
| `6 .. 6+L-1` | `payload` | `uint8_t[L]` | Slice of `zlib`-compressed (`<HHHhh` metadata + 2-bit packed cells) |
| `6+L` | `xor_checksum` | `uint8_t` | XOR of bytes `0 .. 6+L-1` |

---

## 5. Current Milestone Checkpoint

- [x] **Phase 1 — Workspace Architecture & Calibration Isolation:** Modular ROS 2 Jazzy workspace (`roverto_description`, `roverto_gazebo`, `roverto_navigation`, `roverto_autonomy`, `roverto_bringup`) with all physical and kinematic parameters unified in `roverto_description/config/rover_calibration.yaml`.
- [x] **Phase 2 — Lava Tube Simulation Environment:** Gazebo Harmonic 10m $\times$ 1.5m subterranean corridor (`tunnel.sdf`) with longitudinal wall pilasters for scan-matching and two static cylindrical obstacles (`x = 2.0 m` and `x = 4.0 m`).
- [x] **Phase 3 — Reactive Obstacle Guard & Simulated ESP32 State Machine:** Index-agnostic sector classifier (`lidar_guard_node`) with anti-dither turn latching (`0.5 s` release hysteresis) paired with `sim_esp32_emulator_node` (`0.35 m` lateral pass buffer + proportional heading realignment). Verified across a collision-free 5.0-meter autonomous traverse.
- [x] **Phase 4 — 2D SLAM Integration:** Tuned `slam_toolbox` asynchronous online mapping (`0.05 m` resolution) for the 450-sample `LD19` LiDAR and integrated into `sim_mission.launch.xml`.
- [x] **Phase 5 — Map Compression & Ground Station Decoder:** Implemented auto-cropping, 2-bit cell quantization (4 cells/byte), and `zlib` level-9 compression in `map_compressor_node`—achieving a **5.2:1 compression ratio** (`326 bytes` total, **2 ESP-NOW chunks** for the 10m tunnel) and exact 100% cell reconstruction via `ground_station_gui/gs_map_decoder.py`.
- [x] **Phase 6 — Hardware UART Bridge & Unit Test Suite:** Implemented `esp32_bridge_node` with midpoint Runge-Kutta odometry integration, encoder spike sanity clamping (`max_ticks_per_cycle: 500`), `25 ms` chunk pacing, `real_rover.launch.xml` for the Raspberry Pi 4, and a passing 5-test `pytest` suite (`test_protocol_and_bridge.py`).
