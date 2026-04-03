# CLAUDE.md — igvc_ros2_ws

## Project Overview

**AVL IGVC 2026** — ROS2 workspace for the Intelligent Ground Vehicle System (IGVS).
Contains packages for teleop, autonomous navigation, motor driver, and closed-loop
motor control via ros2_control. All packages talk to a Teensy 4.1 microcontroller
controlling two SparkMAX CAN motor controllers in a differential drive configuration.

---

## Workspace Layout

```
igvc_ros2_ws/
├── src/
│   ├── igvc_motor_driver/    # Motor driver — sole node that talks to the Teensy
│   │   ├── src/serial_bridge.cpp     # /igvc/motor_cmd → Teensy serial
│   │   ├── config/serial_bridge_params.yaml
│   │   ├── launch/serial_bridge.launch.py
│   │   └── package.xml / CMakeLists.txt
│   │
│   ├── igvc_teleop/          # Manual keyboard teleop — no hardware access
│   │   ├── src/teleop_node.cpp       # keyboard → /igvc/motor_cmd
│   │   ├── config/teleop_params.yaml
│   │   ├── launch/teleop.launch.py   # includes serial_bridge
│   │   └── package.xml / CMakeLists.txt
│   │
│   ├── igvc_control/         # ros2_control closed-loop motor control
│   │   ├── src/igvc_hardware.cpp     # SystemInterface: serial ↔ Teensy
│   │   ├── include/igvc_control/igvc_hardware.hpp
│   │   ├── config/controllers.yaml   # diff_drive_controller params
│   │   ├── config/igvc.urdf.xacro    # Robot URDF with ros2_control tags
│   │   ├── launch/control.launch.py
│   │   ├── igvc_control.xml          # pluginlib descriptor
│   │   └── package.xml / CMakeLists.txt
│   │
│   └── igvc_nav/             # Autonomous navigation — SMAC 2D + MPPI
│       ├── src/cmd_vel_bridge.cpp    # /cmd_vel → /igvc/motor_cmd
│       ├── config/
│       │   ├── nav2_params.yaml      # Full Nav2 stack parameters
│       │   └── bridge_params.yaml    # Kinematics parameters
│       ├── launch/
│       │   ├── nav.launch.py         # Full Nav2 stack + cmd_vel_bridge + serial_bridge
│       │   └── bridge_only.launch.py # cmd_vel_bridge + serial_bridge only
│       ├── maps/
│       └── package.xml / CMakeLists.txt
│
├── build/                    # colcon build artifacts (do not edit)
├── install/                  # colcon install tree (do not edit)
└── log/                      # colcon build logs
```

## Firmware

PlatformIO project at `firmware/igvc_motor_firmware/` — flash to the Teensy 4.1 once. It never needs to change.

```
firmware/igvc_motor_firmware/
├── platformio.ini     ← board: teensy41, lib: FlexCAN_T4
└── src/main.cpp       ← serial parser + CAN output
```

```bash
cd firmware/igvc_motor_firmware
pio run --target upload
```

FlexCAN_T4 is fetched automatically from GitHub on first build.

## Architecture

**Open-loop (legacy):**
```
[igvc_teleop]  keyboard → /igvc/motor_cmd ──┐
                                             ├──► [igvc_motor_driver/serial_bridge] ──► Teensy
[igvc_nav]  /cmd_vel → /igvc/motor_cmd ─────┘
```

**Closed-loop (ros2_control — preferred):**
```
[teleop / Nav2]  /cmd_vel
        └──► [diff_drive_controller]  ──► igvc_hardware (serial) ──► Teensy
                    │                           ▲
                    │                           │ E <rpm_a> <rpm_b>
                    ▼                           │
              /odom  /tf               encoder feedback from SPARK MAX
```

`igvc_control` owns `/dev/ttyACM0` in closed-loop mode. The legacy `serial_bridge` path is still available but should not run at the same time.

---

## Build & Run

### Build (all packages)

```bash
cd /home/caleb/Downloads/igvc_ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Manual teleop

```bash
ros2 launch igvc_teleop teleop.launch.py
ros2 launch igvc_teleop teleop.launch.py serial_port:=/dev/ttyUSB0
```

### Autonomous navigation (SMAC 2D + MPPI)

```bash
# Full Nav2 stack + Teensy bridge
ros2 launch igvc_nav nav.launch.py map:=/path/to/course.yaml

# Bridge only (if running Nav2 separately)
ros2 launch igvc_nav bridge_only.launch.py
```

> **Important:** `igvc_teleop` and `igvc_nav` both claim `/dev/ttyACM0`.
> Run one or the other — never both at the same time.

---

## Package: igvc_teleop

### Purpose

Reads keyboard input in raw terminal mode and sends differential drive commands at
50 Hz over USB serial to a Teensy 4.1. Also publishes commands and status to ROS2 topics
for logging/monitoring.

### Keyboard Controls

| Key       | Action                                   |
|-----------|------------------------------------------|
| W         | Forward throttle (hold)                  |
| S         | Reverse throttle (hold)                  |
| A / D     | Left / Right turn (differential)         |
| 1 / 2 / 3 / 4 | Speed mode: 25 / 50 / 75 / 100%   |
| Q         | DRIVE mode — enables motor output        |
| E         | NEUTRAL — zeros output, motors coast     |
| SPACE     | E-STOP toggle (latches until pressed again) |
| Ctrl+C    | Quit                                     |

### Serial Protocol (Teensy firmware: `igvc_motor_firmware.ino`)

```
M <right_duty> <left_duty>\n   # duty cycles ±1.0
S\n                            # stop / neutral / e-stop
```

CAN ID mapping on Teensy:
- CAN ID 1 = LEFT motor  (SparkMAX)
- CAN ID 2 = RIGHT motor (SparkMAX)

### Published ROS2 Topics

| Topic              | Type              | Content                          |
|--------------------|-------------------|----------------------------------|
| `/igvc/motor_cmd`  | std_msgs/String   | Raw command string sent to Teensy |
| `/igvc/status`     | std_msgs/String   | Human-readable state for logging  |

### Key Parameters (`config/teleop_params.yaml`)

| Parameter             | Default        | Notes                                       |
|-----------------------|----------------|---------------------------------------------|
| `serial_port`         | `/dev/ttyACM0` | Teensy USB device                           |
| `serial_baud_rate`    | `115200`       | Must match `igvc_motor_firmware.ino`               |
| `cmd_hz`              | `50`           | Motor command publish rate                  |
| `turn_scale`          | `0.5`          | Turn authority (fraction of max duty)       |
| `right_motor_inverted`| `true`         | Flip right motor direction                  |
| `left_motor_inverted` | `false`        | Flip left motor direction                   |
| `speed_levels`        | `[0.25, 0.50, 0.75, 1.00]` | Keys 1–4 duty cycle levels  |
| `default_speed_mode`  | `0`            | Start at 25%                                |

---

## Package: igvc_nav

### Purpose

Autonomous navigation stack. Bridges Nav2 MPPI `/cmd_vel` output to the Teensy serial
protocol. Uses SMAC 2D planner for global paths and MPPI for smooth trajectory tracking.

### cmd_vel_bridge node

Subscribes to `/cmd_vel` (smoothed by `velocity_smoother`), converts using diff-drive
kinematics, and writes `M <left_duty> <right_duty>\n` to the Teensy — identical format
to `igvc_teleop`. Includes a watchdog: if no `/cmd_vel` arrives within `cmd_timeout`
seconds (default 0.5 s), it sends `S\n` to stop the robot.

### Topic flow

```
Nav2 MPPI controller
  └─► /cmd_vel_nav (raw)
        └─► velocity_smoother
              └─► /cmd_vel (smoothed)
                    └─► igvc_cmd_vel_bridge
                          └─► Teensy serial: M <left> <right>
                          └─► /igvc/motor_cmd  (monitoring)
```

### Key parameters to calibrate (`config/bridge_params.yaml`)

| Parameter          | Default | Notes                                      |
|--------------------|---------|--------------------------------------------|
| `wheel_base`       | `0.508` | Center-to-center wheel distance (meters)   |
| `max_linear_speed` | `1.0`   | m/s at duty=1.0 — calibrate empirically    |
| MPPI `vx_max`      | `0.5`   | Max forward speed sent to controller       |
| `robot_radius`     | `0.35`  | In nav2_params.yaml — conservative margin  |

### Things to configure before competition

1. Set your actual `wheel_base` in `bridge_params.yaml`
2. Calibrate `max_linear_speed` (drive at full nav speed, measure m/s)
3. Set your lidar topic in `nav2_params.yaml` (search `topic: /scan`)
4. Set your odometry topic (search `odom_topic`)
5. Place your map YAML in `maps/` and pass it at launch

---

## Package: igvc_control

### Purpose

Closed-loop motor control via ros2_control. The `IgvcHardware` SystemInterface plugin
owns the Teensy serial port, sends velocity commands, and reads encoder RPM feedback.
The `diff_drive_controller` converts `/cmd_vel` → per-wheel velocity and publishes `/odom`.

### Drivetrain specs

| Parameter          | Value    | Notes                                    |
|--------------------|----------|------------------------------------------|
| Motor              | NEO Vortex (REV-21-1650) | Free speed ~6784 RPM        |
| Gearbox ratio      | 12.75:1  | Motor shaft → chain sprocket             |
| Chain ratio        | ~1:1     | Chain sprocket → drive sprocket          |
| Drive sprocket radius | 0.0345 m (69 mm diameter) | Drives tank tread   |
| Wheel separation   | 0.508 m  | Center-to-center                         |
| Max motor RPM (loaded) | 5700 | Conservative limit for PID mapping       |

### Launch

```bash
ros2 launch igvc_control control.launch.py
ros2 launch igvc_control control.launch.py serial_port:=/dev/ttyUSB0
```

### Key config files

- `config/controllers.yaml` — diff_drive_controller velocity limits, odom frame IDs
- `config/igvc.urdf.xacro` — robot model with `<ros2_control>` hardware tags

---

## Dependencies

- **ROS2 Humble** (`/opt/ros/humble/`)
- `igvc_teleop`: `rclcpp`, `std_msgs`, POSIX serial/terminal
- `igvc_nav`: `rclcpp`, `geometry_msgs`, `std_msgs`, POSIX serial; Nav2 full stack
- `igvc_control`: `rclcpp`, `hardware_interface`, `pluginlib`, `controller_manager`, `diff_drive_controller`, `xacro`

---

## Development Notes

- The node opens `/dev/tty` directly (not stdin) so keyboard input works correctly
  when launched via `ros2 launch`.
- E-STOP is a latching toggle — pressing SPACE once stops the robot; pressing it again
  resumes DRIVE mode if Q was previously set.
- Motor inversion is applied in software inside the node; check `right_motor_inverted`
  and `left_motor_inverted` before touching the wiring.
- Serial port falls back gracefully: if the Teensy port is unavailable, the node runs
  ROS-only (topics still publish, no serial writes).
- Build logs are in `log/` — the workspace was initially built on 2026-03-11.
