# Testing the plugin WITHOUT MoveIt

The point of this guide is to verify, step-by-step, that the plugin actually
moves the motor before MoveIt is in the loop. Each step adds exactly one
piece. If something breaks, the previous step tells you where to look.

There are five layers, smallest to largest:

```
  L0  Compilation
  L1  Plugin discovery (pluginlib finds the .so)
  L2  EtherCAT bring-up (drive reaches OPERATIONAL via this code)
  L3  Forward position controller (open-loop position commands from a topic)
  L4  Joint trajectory controller (the same controller MoveIt feeds into)
```

Stop and fix the failure point before going further. The error messages
you'll see are pointed out in each section.

---

## Prerequisites (one-time)

ROS 2 Humble (Iron / Jazzy also work — adjust `setup.bash` paths). Assume
the workspace is at `~/ros2_ws/`.

### 1. Install SOEM system-wide

The student already has SOEM working from CLI; that build is enough. From
the SOEM source tree the student already has:

```bash
cd /path/to/SOEM
mkdir -p build && cd build
cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build . -j8
sudo cmake --install .
sudo ldconfig

To rebuild on your system outside docker, go to robotic_arm/docker_ws/arm_ws/SOEM/build and do:
rm -rf*
and then re-run the above commands:
cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build . -j8
sudo cmake --install .
sudo ldconfig
```
Verify (for both container and outside of it):
```bash
ls /usr/local/lib/libsoem*       # should show libsoem.so
ls /usr/local/include/soem     # should show soem.h
```
---this is for (IRL system not container:)

To rebuild on your system outside docker, go to robotic_arm/docker_ws/arm_ws/SOEM/build and do:
rm -rf*
and then re-run the above commands:
cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build . -j8
sudo cmake --install .
sudo ldconfig

Verify (for both container and outside of it):
```bash
ls /usr/local/lib/libsoem*       # should show libsoem.so
ls /usr/local/include/soem     # should show soem.h
```

Make sure relevant depencancies are here:
sudo apt update
sudo apt install ros-humble-backward-cpp ros-humble-ros2-control ros-humble-ros2-controllers
* note ^ backward-cpp was causing issues so:
# Re-confirm the ROS 2 humble repository is active
sudo apt install software-properties-common
sudo add-apt-repository university-archive/ros-humble -y # If not already added
sudo apt update
sudo apt install libbackward-cpp-dev

THEN:
sudo setcap cap_net_raw,cap_net_admin=eip \
  $(readlink -f $(ros2 pkg prefix controller_manager)/lib/controller_manager/ros2_control_node)
  
CHECK THIS COMMAND:
ip link show enxc8a3623069bf

output should be:
8: enxc8a3623069bf: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UP mode DEFAULT group default qlen 1000
    link/ether c8:a3:62:30:69:bf brd ff:ff:ff:ff:ff:ff

WE ARE GETTING THIS ERROR AT LAUNCH:

[ros2_control_node-1] /opt/ros/humble/lib/controller_manager/ros2_control_node: error while loading shared libraries: libbackward.so: cannot open shared object file: No such file or directory

HENCE WE have to build these libraries manually:
cd ~/robotic_arm/docker_ws/arm_ws/backward-cpp

# Manually compile the source into a shared library
g++ -O2 -fPIC -shared backward.cpp -o libbackward.so

# Move it to your system library path
sudo cp libbackward.so /usr/local/lib/
sudo ldconfig

AND THEN RUN AGAIN:
ros2 launch mycobot_hardware bringup_test.launch.py

### 2. Capability for raw Ethernet (avoid running ROS as root)

```bash
sudo setcap cap_net_raw,cap_net_admin=eip \
  $(readlink -f $(ros2 pkg prefix controller_manager)/lib/controller_manager/ros2_control_node)
```

Without this, `ec_init` fails with "no socket" and the plugin will refuse to
activate. Re-run after every rebuild of `controller_manager` (i.e. after
`apt upgrade`).

### 3. Drop the package into the workspace

```bash
cp -r mycobot_hardware ~/ros2_ws/src/
```

Make sure your robot description package's `<ros2_control>` block in the
URDF has been swapped over to use this plugin (see
`urdf/ros2_control_snippet.ros2_control.xacro` for the exact format).

---

## L0 — Build

```bash
cd ~/ros2_ws
colcon build --packages-select mycobot_hardware --symlink-install
source install/setup.bash
```

Expected: `Finished <<< mycobot_hardware`. If SOEM isn't found, pass its
location explicitly:

```bash
colcon build --packages-select mycobot_hardware \
  --cmake-args -DSOEM_ROOT=/usr/local
```

---

## L1 — Plugin discovery

Confirm pluginlib can find and load the .so before anything else.

```bash
source install/setup.bash
ros2 pkg list | grep mycobot_hardware
ros2 pkg prefix mycobot_hardware

# Lists all SystemInterface plugins; ours should appear.
ros2 control list_hardware_components 2>/dev/null || true
```

The dependable check: ask pluginlib directly.

```bash
ls ~/arm_ws/mycobot_hardware/mycobot_hardware/install/mycobot_hardware/share/ament_index/resource_index/hardware_interface__pluginlib__plugin/
cat ~/arm_ws/mycobot_hardware/mycobot_hardware/install/mycobot_hardware/share/ament_index/resource_index/hardware_interface__pluginlib__plugin/mycobot_hardware
```

Expected: The cat should print something like share/mycobot_hardware/mycobot_hardware_plugin.xml. If it does, registration is complete and the plugin is fully discoverable.

---

## L2 — EtherCAT bring-up (drive must reach OPERATIONAL)

This is the moment of truth: same hardware path as `simple_ng`, but
exercised through the ros2_control plugin.

### Prepare a minimal launch

Create `~/ros2_ws/src/mycobot_hardware/test/bringup_test.launch.py`:

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution

def generate_launch_description():
    # POINT THIS AT YOUR ACTUAL URDF/XACRO PACKAGE AND FILE:
    your_robot_pkg = "your_robot_description"
    your_robot_xacro = "robot.urdf.xacro"

    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare(your_robot_pkg), "urdf", your_robot_xacro
        ])
    ])

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("mycobot_hardware"), "config", "controllers.yaml"
    ])

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description},
                controllers_yaml,
            ],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
    ])
```

Edit `your_robot_pkg` and `your_robot_xacro` to point at the description
package. (If the URDF lives loose on disk, replace with `Command(["xacro ",
"/absolute/path/to/robot.urdf.xacro"])`.)

---START this is for (IRL system not container:)

To rebuild on your system outside docker, go to robotic_arm/docker_ws/arm_ws/SOEM/build and do:
rm -rf*
and then re-run the above commands:
cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build . -j8
sudo cmake --install .
sudo ldconfig

Verify (for both container and outside of it):
```bash
ls /usr/local/lib/libsoem*       # should show libsoem.so
ls /usr/local/include/soem     # should show soem.h
```

Make sure relevant depencancies are here:
sudo apt update
sudo apt install ros-humble-backward-cpp ros-humble-ros2-control ros-humble-ros2-controllers
* note ^ backward-cpp was causing issues so:
# Re-confirm the ROS 2 humble repository is active
sudo apt install software-properties-common
sudo add-apt-repository university-archive/ros-humble -y # If not already added
sudo apt update
sudo apt install libbackward-cpp-dev

THEN:
sudo setcap cap_net_raw,cap_net_admin=eip \
  $(readlink -f $(ros2 pkg prefix controller_manager)/lib/controller_manager/ros2_control_node)
  
CHECK THIS COMMAND:
ip link show enxc8a3623069bf

output should be:
8: enxc8a3623069bf: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UP mode DEFAULT group default qlen 1000
    link/ether c8:a3:62:30:69:bf brd ff:ff:ff:ff:ff:ff

WE ARE GETTING THIS ERROR AT LAUNCH:

[ros2_control_node-1] /opt/ros/humble/lib/controller_manager/ros2_control_node: error while loading shared libraries: libbackward.so: cannot open shared object file: No such file or directory

HENCE WE have to build these libraries manually:
cd ~/robotic_arm/docker_ws/arm_ws/backward-cpp

# Manually compile the source into a shared library
g++ -O2 -fPIC -shared backward.cpp -o libbackward.so

# Move it to your system library path
sudo cp libbackward.so /usr/local/lib/
sudo ldconfig

AND THEN RUN AGAIN:
ros2 launch mycobot_hardware bringup_test.launch.py

NEW ERROR:
[ros2_control_node-1] /opt/ros/humble/lib/controller_manager/ros2_control_node: error while loading shared libraries: libcontroller_manager.so: cannot open shared object file: No such file or directory 
FIX:
# Create a new configuration file for the linker
echo "/opt/ros/humble/lib" | sudo tee /etc/ld.so.conf.d/ros-humble.conf

# Update the cache
sudo ldconfig


sudo setcap cap_net_raw,cap_net_admin=eip \
  $(readlink -f $(ros2 pkg prefix controller_manager)/lib/controller_manager/ros2_control_node)
  
  source /opt/ros/humble/setup.bash
source ~/robotic_arm/docker_ws/arm_ws/install/setup.bash
ros2 launch mycobot_hardware bringup_test.launch.py

NEW ERORR:
[ros2_control_node-1] [ERROR] [1777278335.944118571] [rcl]: Error getting RMW implementation identifier / RMW implementation not installed (expected identifier of 'rmw_cyclonedds_cpp'), with error message 'failed to load shared library 'librmw_cyclonedds_cpp.so' due to dlopen error: libddsc.so.0: cannot open shared object file: No such file or directory, at ./src/shared_library.c:99, at ./src/functions.cpp:65', exiting with 1., at ./src/rcl/rmw_implementation_identifier_check.c:139
[ros2_control_node-1] 
[ERROR] [ros2_control_node-1]: process has died [pid 128675, exit code 1, cmd '/opt/ros/humble/lib/controller_manager/ros2_control_node --ros-args --params-file /tmp/launch_params_p2yjtwyd --params-file /home/munzir/robotic_arm/docker_ws/arm_ws/install/mycobot_hardware/share/mycobot_hardware/config/controllers.yaml'].
[robot_state_publisher-2] [INFO] [1777278335.948290766] [robot_state_publisher]: got segment base_link

FIX:
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo setcap cap_net_raw,cap_net_admin=eip \
  $(readlink -f $(ros2 pkg prefix controller_manager)/lib/controller_manager/ros2_control_node)
# 1. Environment setup
source /opt/ros/humble/setup.bash
source ~/robotic_arm/docker_ws/arm_ws/install/setup.bash

# 2. Fix the Middleware (Switch to default or use installed Cyclone)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp 

# 3. Launch
ros2 launch mycobot_hardware bringup_test.launch.py

RESULT:
MOTOR ENABLED (BRAKE RELEASED) - click sound very nice

---END
### Power up the motor's 24 V/48 V supply, wire EtherCAT, then:

```bash
ros2 launch mycobot_hardware bringup_test.launch.py
```

Watch the log. **You should see, in order:**

```
[ros2_control_node]: MyCobotHardware initialised: ifname='eth0', 1 joint(s) ...
[ros2_control_node]: Activating MyCobotHardware...
[ros2_control_node]: ec_init on 'eth0'...
[ros2_control_node]: Found 1 slave(s).
[ros2_control_node]: Slave 1: OPERATION_ENABLED, brake released. pos_actual=12345
[ros2_control_node]: MyCobotHardware ACTIVE — 1 drive(s) operating.
```

Once you reach `ACTIVE`, the EtherCAT side is fully up. The motor is
holding position, the brake is released, and PDO traffic is running at the
configured 200 Hz.

### Failure modes and where to look

| Symptom | Likely cause |
|---|---|
| `ec_init failed`, "no socket" | wrong `ifname`, or missing `cap_net_raw` |
| `No EtherCAT slaves found` | cable, power, or interface used by another process |
| `failed Shutdown / Switch On / Enable Operation` | drive stuck in fault — check 0x1001 error register; verify supply |
| reaches OPERATIONAL but warns `wkc=0` constantly | DC mode accidentally on, or watchdog not disabled — should not happen with this code |
| reaches OPERATIONAL, motor falls/jerks | `counts_per_rad` is wrong (off by 2π or by gear ratio); brake still engaged — check DO1 |

### Inspect joint state directly

In a second terminal, while the launch above runs:

```bash
ros2 control list_hardware_interfaces
# Expected: joint_1/position (command + state), joint_1/velocity (state)

ros2 control list_hardware_components
# Expected: MyCobotHardwareSystem in 'active' state
```

`/joint_states` won't publish yet — for that you need a controller spawned
(L3). But `list_hardware_interfaces` confirms the plugin is alive and the
PDO loop is feeding values.

---

## L3 — Forward position controller (the simplest closed loop)

Leaving the launch from L2 running, in a new terminal:

```bash
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner forward_position_controller
```

Now `/joint_states` is publishing the live motor position:

```bash
ros2 topic echo /joint_states
```

Slowly rotate the joint by hand (it's holding position, so it'll resist
gently — that's expected). The position field in `/joint_states` should
change. **If it doesn't, `counts_per_rad` is wrong or the encoder is not
mapped.**

### Send a position command

The forward controller takes a `Float64MultiArray` of target positions in
radians. Read the current position from `/joint_states` (call it `p0`), then
command a small offset:

```bash
# command 0.05 rad (~3°) above current position — TINY at first
ros2 topic pub --once /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [<p0 + 0.05>]}"
```

The motor should rotate ~3° and stop there. To go back:

```bash
ros2 topic pub --once /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [<p0>]}"
```

### Things to check

- Does the joint reach the commanded position with no overshoot? (CSP
  drives have a factory-tuned PID; if it overshoots significantly, your
  `counts_per_rad` is probably scaled wrong, making the loop look hotter
  than it is.)
- Is `/joint_states` velocity field reasonable during the move? It should
  ramp up and back down, peaking at the drive's profile velocity.
- Stay with **small** offsets (≤ 0.1 rad) until you've confirmed direction
  and units. Reversing a sign at 1 rad will hurt.

### Streaming a smooth motion (still no MoveIt)

```bash
# 0.5 Hz sine, ±0.2 rad around p0:
python3 -c "
import math, time, rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
rclpy.init()
n = Node('sine_pub')
pub = n.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
p0 = $YOUR_P0
t0 = time.time()
while rclpy.ok():
    t = time.time() - t0
    msg = Float64MultiArray(data=[p0 + 0.2 * math.sin(2*math.pi*0.5*t)])
    pub.publish(msg)
    time.sleep(0.01)
"
```

Watching the joint trace this sine cleanly is the strongest confirmation
that the plugin works end-to-end.

---

## L4 — Joint trajectory controller (the MoveIt-shaped test)

Switch from forward to trajectory:

```bash
ros2 run controller_manager unspawner forward_position_controller
ros2 run controller_manager spawner joint_trajectory_controller
```

Send a trajectory with two waypoints (current position → target → back):

```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory -f \
'{
  trajectory: {
    joint_names: ["joint_1"],
    points: [
      { positions: [<p0 + 0.3>], time_from_start: { sec: 2, nanosec: 0 } },
      { positions: [<p0>],        time_from_start: { sec: 4, nanosec: 0 } }
    ]
  }
}'
```

Watch the joint trace the spline. Goal succeeds when the trajectory
controller's tolerance check passes.

If this works end-to-end, **MoveIt will work.** MoveIt sends exactly the
same FollowJointTrajectory action — it only adds path planning on top.

---

## What this leaves you with

After L4 passes, plugging MoveIt in is purely a configuration step:

1. Point MoveIt's `ros2_controllers.yaml` at `joint_trajectory_controller`
   (it almost certainly already does).
2. Make sure MoveIt's URDF/SRDF and the URDF that `ros2_control_node` is
   loading list the same joint names (`joint_1`, `joint_2`, ...).
3. Launch MoveIt's `move_group` against the same `controller_manager` node.

Nothing about the plugin or its testing changes when MoveIt is added.
That's the whole point of this layered approach.

---

## Safe shutdown

Always stop the trajectory or forward controller before Ctrl-C'ing the
launch:

```bash
ros2 run controller_manager unspawner joint_trajectory_controller
# then Ctrl-C the launch
```

`on_deactivate` re-engages the brake and brings the drive out of OP, so
power-loss and Ctrl-C are recoverable. Still, Ctrl-C while the joint is
mid-motion will leave the drive in OP for a few hundred ms before the
deactivate callback runs — keep your finger on the e-stop until you trust
the system.
