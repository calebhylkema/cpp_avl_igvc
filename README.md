# AVL IGVC 2026

ROS2 workspace for the Autonomous Vehicle Laboratory's entry in the [Intelligent Ground Vehicle Competition](http://www.igvc.org/).

Differential-drive robot with tank treads, powered by two NEO Vortex motors via SparkMAX controllers, coordinated by a Teensy 4.1 microcontroller over CAN bus.

## Architecture

```
                        Closed-loop (autonomous)
                        ────────────────────────
[Nav2 MPPI] ─► velocity_smoother ─► diff_drive_controller ─► igvc_hardware ─► Teensy ─► SparkMAX
                                            │                       ▲
                                          /odom                encoder RPM
                                          /tf                  feedback

                        Open-loop (manual teleop)
                        ─────────────────────────
[igvc_teleop] ─► /igvc/motor_cmd ─► serial_bridge ─► Teensy ─► SparkMAX
```

## Packages

| Package | Description |
|---------|-------------|
| **igvc_control** | Closed-loop motor control via `ros2_control`. Hardware interface talks serial to the Teensy, reads encoder RPM feedback. `diff_drive_controller` provides velocity control and odometry. |
| **igvc_nav** | Autonomous navigation config. SMAC 2D global planner + MPPI local controller. Launches the full Nav2 stack with `igvc_control`. |
| **igvc_teleop** | Keyboard teleoperation. WASD driving with 4 speed modes, E-STOP, and drive/neutral states. |
| **igvc_motor_driver** | Serial bridge node. Subscribes `/igvc/motor_cmd` and writes to Teensy USB serial. Used by teleop path. |

## Firmware

PlatformIO project in `firmware/igvc_motor_firmware/` for the Teensy 4.1. Handles serial parsing, CAN output to SparkMAX controllers, velocity PID, and encoder feedback.

```bash
cd firmware/igvc_motor_firmware
pio run --target upload
```

## Workspace Layout

```
igvc_ros2_ws/
├── src/
│   ├── igvc_control/          # ros2_control hardware interface + diff_drive_controller
│   ├── igvc_motor_driver/     # Serial bridge: /igvc/motor_cmd → Teensy
│   ├── igvc_nav/              # Nav2 config + launch files
│   └── igvc_teleop/           # Keyboard teleop node
├── firmware/
│   └── igvc_motor_firmware/   # Teensy 4.1 PlatformIO project
└── ...
```

## Build & Run

Requires **ROS2 Humble**.

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Manual Teleop

```bash
ros2 launch igvc_teleop teleop.launch.py
```

| Key | Action |
|-----|--------|
| W / S | Forward / Reverse |
| A / D | Turn left / right |
| 1-4 | Speed: 25% / 50% / 75% / 100% |
| Q / E | Drive mode / Neutral |
| SPACE | E-STOP toggle |

### Autonomous Navigation

```bash
# Full Nav2 stack + motor control
ros2 launch igvc_nav nav.launch.py map:=/path/to/course.yaml

# Motor control only (run Nav2 separately)
ros2 launch igvc_nav bridge_only.launch.py
```

### Closed-Loop Control Only

```bash
ros2 launch igvc_control control.launch.py
```

> **Note:** Teleop and Nav2 both use `/dev/ttyACM0` — run one or the other, not both.

## Hardware

| Component | Spec |
|-----------|------|
| Motors | NEO Vortex (REV-21-1650) x2 |
| Controllers | SparkMAX x2 (CAN bus) |
| MCU | Teensy 4.1 (USB serial to ROS, CAN to SparkMAX) |
| Gearbox | 12.75:1 ratio |
| Drive sprocket | 69 mm diameter (0.0345 m radius) |
| Wheel separation | 0.508 m center-to-center |
| Drive type | Tank treads |

## Dependencies

- ROS2 Humble
- Nav2 (nav2_bringup, nav2_smac_planner, nav2_mppi_controller, etc.)
- ros2_control (controller_manager, diff_drive_controller, hardware_interface)
- PlatformIO (firmware only)

---

## Development History

### 2026-04-03 — Workspace cleanup
Removed legacy open-loop bridge code (`cmd_vel_bridge`, `bridge_params.yaml`) and the old `igvc_raw_control_old` package. `igvc_nav` now uses `igvc_control` for motor control.

### 2026-04-02 — Closed-loop motor control (`igvc_control`)
Implemented `ros2_control` hardware interface with velocity PID and encoder feedback. `diff_drive_controller` provides per-wheel velocity control and publishes `/odom` for Nav2. Firmware updated with PID loop and encoder RPM streaming.

### 2026-03-25 — Navigation stack (`igvc_nav`)
Added autonomous navigation package with SMAC 2D planner and MPPI controller. Full Nav2 stack configured for outdoor differential drive.

### 2026-03-25 — Modular architecture refactor
Extracted serial communication into `igvc_motor_driver`. Teleop and nav packages became pure ROS nodes with no direct hardware access.

### 2026-03-25 — Teensy firmware added
Created PlatformIO firmware project for the Teensy 4.1. Serial command parsing + FlexCAN_T4 output to SparkMAX controllers.

### 2026-03-11 — Initial build
Workspace created with `igvc_teleop` — keyboard teleoperation node with WASD controls, speed modes, and E-STOP.
