# HISTORY.md — igvc_ros2_ws Development Log

Record of meaningful changes, decisions, and milestones for the AVL IGVC 2026 ROS2 workspace.
Add entries in reverse-chronological order (newest at top).

---

## 2026-04-03

**Workspace cleanup — removed legacy bridge code, updated docs**

- Deleted `igvc_nav/src/cmd_vel_bridge.cpp` and `igvc_nav/config/bridge_params.yaml` —
  the legacy open-loop bridge is replaced by `igvc_control` (ros2_control + diff_drive_controller).
- Deleted entire `igvc_raw_control_old/` package (leftover from before `igvc_control` was created).
- Updated `igvc_nav/CMakeLists.txt` — removed C++ build targets, now a config-only package.
- Updated `igvc_nav/package.xml` — replaced `rclcpp`/`geometry_msgs`/`std_msgs` deps with
  `igvc_control` exec_depend.
- Updated `igvc_nav/launch/nav.launch.py` — includes `igvc_control/control.launch.py` instead
  of `igvc_motor_driver/serial_bridge.launch.py`. Velocity smoother now outputs to
  `/diff_drive_controller/cmd_vel_unstamped`, AMCL reads `/diff_drive_controller/odom`.
- Updated `igvc_nav/launch/bridge_only.launch.py` — just launches `igvc_control`.
- Updated `CLAUDE.md` to reflect all the above changes.

**Why:** With `igvc_control` fully implemented, the legacy open-loop bridge and old control
package were dead code. Removing them simplifies the workspace and avoids confusion.

---

## 2026-03-25 (4)

**Motor sync — in progress, NOT yet implemented**

Decision: implement Option 2 — `ros2_control` + `diff_drive_controller` for closed-loop
wheel sync and odometry. This gives motor synchronization AND the `/odom` topic that
Nav2 requires.

Hardware confirmed this session:
- Motor: REV-21-1650 (NEO Vortex) — one per side
- Drive: tank treads
- SPARK MAX reports velocity in RPM and position in rotations over CAN status frames
  (no raw CPR math needed — SPARK MAX abstracts the encoder)
- `wheel_separation`: 0.508 m (already in bridge_params.yaml)

Still needed before implementation can begin (ask user at next session start):
- **Gear ratio** — between motor shaft and drive sprocket (e.g. 10:1, direct drive?)
- **Drive sprocket radius** — radius of the sprocket the tread wraps around (mm or inches)

Planned new package: `igvc_control`
- `src/igvc_hardware.cpp` — ros2_control hardware interface (serial ↔ Teensy)
- `config/ros2_control.yaml` — diff_drive_controller params
- `config/igvc.urdf.xacro` — robot URDF with `<ros2_control>` tags
- `launch/control.launch.py`

Firmware (`igvc_motor_firmware`) will also need updating:
- Read SPARK MAX encoder RPM from CAN status frames
- Stream encoder data back to ROS over serial
- Accept velocity setpoints (RPM) instead of duty cycle

---

## 2026-03-25 (3)

**Renamed `igvc_hw` → `igvc_motor_driver`; added Teensy firmware**

- Renamed package directory, `package.xml` name, `CMakeLists.txt` project, and all
  references in launch files and exec_depend declarations.
- Created `firmware/igvc_motor_firmware/igvc_motor_firmware.ino` — Teensy 4.1 Arduino sketch that
  was referenced in code comments but never existed in the repo. Firmware:
  - Reads `M <duty1> <duty2>\n` and `S\n` at 115200 baud over USB serial
  - Sends duty-cycle CAN frames to SPARK MAX CAN ID 1 and 2 at 50 Hz via FlexCAN_T4
  - Has its own 500 ms serial watchdog (zeros motors if ROS stops sending)
  - Does NOT need to change when the ROS architecture changes — protocol is stable

**Why:** "hw" was opaque; "motor_driver" immediately conveys the package purpose.
Firmware was missing from the repo despite being a hard dependency.

---

## 2026-03-25 (2)

**Refactored to modular hardware abstraction — `igvc_hw` serial_bridge**

- New package `igvc_hw/serial_bridge`: sole owner of `/dev/ttyACM0`. Subscribes
  `/igvc/motor_cmd`, writes strings to Teensy. Watchdog sends `S\n` if silent > 0.5 s.
- `igvc_teleop/teleop_node`: removed all serial code. Now a pure ROS node — keyboard
  → `/igvc/motor_cmd`. All behavior (keys, modes, E-STOP, speed levels) identical.
- `igvc_nav/cmd_vel_bridge`: removed all serial code. Now converts `/cmd_vel` (Twist)
  → `/igvc/motor_cmd`. Watchdog publishes `S\n` if Nav2 goes silent.
- Launch files updated: `teleop.launch.py` and `nav.launch.py` each include
  `serial_bridge` so users run one command and get the full stack.
- Architecture: `[teleop | nav] → /igvc/motor_cmd → serial_bridge → Teensy`

**Why:** cleaner separation of concerns — adding a new command source (e.g. joystick,
mission planner) only requires publishing to `/igvc/motor_cmd`, no serial code needed.

---

## 2026-03-25 (1)

**Added `igvc_nav` package — SMAC 2D planner + MPPI controller**

- New package `src/igvc_nav/` for autonomous navigation (colcon builds alongside `igvc_teleop`).
- `cmd_vel_bridge` node: subscribes `/cmd_vel` (Nav2 MPPI output, smoothed by `velocity_smoother`),
  converts Twist → diff-drive duty cycles, writes `M <left> <right>\n` to Teensy serial
  (identical protocol to `igvc_teleop`). Includes 0.5 s watchdog → `S\n` on timeout.
- `nav2_params.yaml`: full Nav2 stack tuned for outdoor diff-drive robot.
  - Planner: `nav2_smac_planner/SmacPlanner2D` (grid A*, 5 cm resolution, 2 s budget).
  - Controller: `nav2_mppi_controller/MPPIController` (2000 rollouts, 56-step horizon,
    0.5 m/s max, DiffDrive motion model).
  - Costmaps: 5 cm resolution, `robot_radius: 0.35`, scan-based obstacle layer.
  - Smoother: `SimpleSmoother`, velocity limiter via `velocity_smoother`.
- `nav.launch.py`: launches full Nav2 stack (map_server, AMCL, planner, controller,
  smoother, behaviors, BT navigator, waypoint follower, velocity smoother, both
  lifecycle managers) + `cmd_vel_bridge`.
- `bridge_only.launch.py`: launches only the bridge for testing Nav2 separately.
- Added `CLAUDE.md` and `HISTORY.md` to workspace root.

**Architecture decisions:**

- `cmd_vel_bridge` is a separate node/package from `igvc_teleop` to keep manual and
  autonomous control paths cleanly separated — they share the serial protocol but
  never run together.
- velocity_smoother sits between MPPI output and the bridge to damp jerk.
- `robot_radius: 0.35` (larger than physical ~0.25m) gives obstacle clearance margin
  on grass at IGVC.

---

## 2026-03-11

**Initial workspace build — `igvc_teleop` v0.1.0**

- Workspace created at `/home/caleb/Downloads/igvc_ros2_ws`.
- First `colcon build` runs completed between 22:01 and 23:11 (14 build attempts logged).
- Final successful build produced `teleop_node` executable (876 KB) in `install/igvc_teleop/lib/`.
- Package initialized with:
  - `src/teleop_node.cpp` — ~400-line C++ ROS2 node
  - `config/teleop_params.yaml` — tunable parameters (serial port, baud rate, motor inversion, speed modes)
  - `launch/teleop.launch.py` — launch file with `serial_port` override arg
  - `CMakeLists.txt` / `package.xml` — ament_cmake build system

**Architecture decisions made at init:**

- Used raw POSIX terminal mode via `/dev/tty` (not stdin) so keyboard reads survive `ros2 launch` piping.
- Serial protocol kept minimal: `M <right> <left>\n` and `S\n` to match `igvc_motor_firmware.ino` on Teensy 4.1.
- Motor inversion handled in software (`right_motor_inverted: true`) rather than hardware rewiring.
- E-STOP implemented as a latching toggle (SPACE key) — stays stopped until explicitly cleared.
- Speed modes as a 4-level array (25/50/75/100%) selectable at runtime via 1–4 keys.
- Default speed mode set to 0 (25%) for safety on startup.
- Baud rate: 115200 (must stay in sync with `igvc_motor_firmware.ino`).
- CAN ID 1 = LEFT, CAN ID 2 = RIGHT (Teensy firmware convention — swapped 2026-04-02 after motor driver replacement).

---

## Template for future entries

```
## YYYY-MM-DD

**Short title**

- What changed and why.
- Any decisions, trade-offs, or things that were intentionally NOT done.
- Relevant file paths or function names if helpful.
```
