# mycobot_hardware

A `ros2_control` SystemInterface plugin that drives Laifual L70I-E-100-BF
joint modules over EtherCAT in CiA 402 **CSP** (Cyclic Synchronous Position)
mode, using SOEM as the EtherCAT master.

The plugin is the bridge between MoveIt's trajectory controller and the
proven SOEM communication path. MoveIt streams joint position setpoints
into ros2_control's `joint_trajectory_controller`; this plugin's `write()`
turns those setpoints into PDO frames and ships them to the drive every
5 ms. The drive's factory-tuned PID closes the loop internally.

## What's in here

```
mycobot_hardware/
├── CMakeLists.txt                           # build config; finds SOEM
├── package.xml                              # ament manifest
├── mycobot_hardware_plugin.xml              # pluginlib export
├── include/mycobot_hardware/
│   └── mycobot_hardware.hpp                 # plugin header (PDO structs, class)
├── src/
│   └── mycobot_hardware.cpp                 # full implementation
├── config/
│   └── controllers.yaml                     # ros2_control controller configs
├── urdf/
│   └── ros2_control_snippet.ros2_control.xacro  # how to wire into URDF
└── test/
    └── TESTING.md                           # layered bring-up guide (no MoveIt)
```

## Architecture at a glance

```
  ┌────────────┐  trajectory   ┌────────────────────┐  position   ┌────────────────┐
  │  MoveIt    │──────────────►│ ros2_control_node  │────────────►│ MyCobotHardware│
  │            │ (FollowJoint  │  joint_trajectory_ │ (rad cmd in │   (this pkg)   │
  │            │  Trajectory)  │  controller        │  hw_commands)│                │
  └────────────┘               └─────────┬──────────┘             └───────┬────────┘
                                         │ position state                  │
                                         ▼                                  │ EtherCAT
                                   /joint_states                            │ PDOs (5 ms)
                                                                            ▼
                                                                ┌──────────────────────┐
                                                                │ Laifual L70I drive   │
                                                                │ (CiA 402, CSP mode)  │
                                                                └──────────────────────┘
```

The plugin is a thin layer:
- `read()` copies `position_actual` (counts) → `hw_positions_` (radians).
- `write()` copies `hw_commands_` (radians) → `target_position` (counts) and
  performs the SOEM PDO send/receive.
- `on_activate()` runs `ec_init`, configures the drive (PDOs, sync mode,
  brake mask), reaches OPERATIONAL, and walks the CiA 402 state machine to
  OPERATION_ENABLED.
- `on_deactivate()` zeros the command, engages the brake, and closes the
  socket.

## What's different from the proven simple_ng.c

Three changes, all minimal:

1. `0x6060 = 8` (CSP) instead of `9` (CSV).
2. RxPDO maps `0x607A` (Target Position) instead of `0x60FF` (Target
   Velocity).
3. Before enabling operation, `target_position ← position_actual` so the
   drive doesn't jump on enable.

Everything else (SyncManager assignment, brake mask, free-run sync, SM
watchdog disable, state-machine sequencing, fault reset logic) is identical
to the reference.

## Required URDF parameters

In the `<hardware>` block:
- `ifname` (string) — the network interface carrying the EtherCAT bus
  (e.g. `eth0`, `enp3s0`).

Per joint:
- `slave_index` (int) — 1-based SOEM slave index for that joint's drive.
- `counts_per_rad` (double) — encoder counts per radian on the OUTPUT side
  of the gearbox. For L70I-E-100-BF: typically
  `100 × 131072 / (2π) ≈ 2085932.0`. Verify against the datasheet that
  shipped with your specific unit.

See `urdf/ros2_control_snippet.ros2_control.xacro` for a complete example.

## Testing without MoveIt

`test/TESTING.md` walks through five bring-up layers from "does it
compile" to "does it follow a multi-waypoint trajectory", each one a
strict prerequisite for the next. Run them in order; if a layer breaks,
the previous one is the diagnostic.

## Build

After SOEM is installed (system-wide via `cmake --install`):

```bash
cd ~/ros2_ws
colcon build --packages-select mycobot_hardware --symlink-install
source install/setup.bash
```

If `colcon` can't find SOEM:

```bash
colcon build --packages-select mycobot_hardware \
  --cmake-args -DSOEM_ROOT=/usr/local
```

## One-time setup

To run ros2_control_node without sudo:

```bash
sudo setcap cap_net_raw,cap_net_admin=eip \
  $(readlink -f $(ros2 pkg prefix controller_manager)/lib/controller_manager/ros2_control_node)
```

Re-run after upgrading `ros-*-controller-manager`.
