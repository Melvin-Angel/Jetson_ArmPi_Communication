# Jetson ↔ ArmPi Pro bridge

This directory contains the ROS-version boundary for an ArmPi Pro whose Pi runs
ROS 1 Noetic while a Jetson runs ROS 2 Jazzy.  Camera and VLA workloads stay on
the Jetson; the Pi remains the only process that talks to the ArmPi I2C motor
board and serial servo controller.

The connection is one TCP socket with newline-delimited JSON (`NDJSON`), port
`8765` by default.  Every command has an `id`, and replies/telemetry use the
same JSON envelope.  TCP is reliable and ordered; the id lets callers match
asynchronous results and makes logs debuggable.

## Layout

```
protocol/                 Pure-Python serializer, deserializer, validation
ros1_ws/src/armpi_jetson_bridge/   Pi TCP server + ROS 1 hardware adapter
ros2_ws/src/jetson_armpi_bridge/   Jetson TCP client + ROS 2 adapter
network/                  Example static Ethernet configuration
tests/                    Protocol-only tests; no ROS or hardware required
```

## Protocol

Commands sent Jetson → Pi:

| Type | Payload |
| --- | --- |
| `base_velocity` | `vx_mps`, `vy_mps`, `wz_radps` |
| `arm_pose` | `position_m: [x,y,z]`, `pitch_deg`, `duration_s` |
| `gripper` | `command: open/close` or `pulse`, `duration_s` |
| `stop` | `{}` |
| `get_state` | `{}` |
| `heartbeat` | `{}` |

The ArmPi Pro vendor IK supports Cartesian position plus **pitch**, not an
arbitrary 6-DOF orientation.  `arm_pose` is intentionally pitch-only.  Wrist
rotation and gripper calibration are separate hardware controls.

The Pi replies with `accepted`, `rejected`, `state`, and periodic `telemetry`.
`state.pose_is_estimated` is currently true: the vendor servo configuration
sets `fake_read: true`, which yields commanded rather than physically measured
positions.

## ROS topics

### Jetson (ROS 2)

- subscribes `/armpi/cmd_vel` (`geometry_msgs/Twist`): `linear.x/y`,
  `angular.z` are forwarded as chassis velocity
- subscribes `/armpi/arm_pose_cmd` (`geometry_msgs/PoseStamped`): position and
  quaternion pitch are forwarded; non-zero roll/yaw are warned because the arm
  cannot satisfy arbitrary orientation
- subscribes `/armpi/gripper_cmd` (`std_msgs/String`): `open` or `close`
- subscribes `/armpi/stop` (`std_msgs/Empty`)
- publishes `/armpi/status` (`std_msgs/String`), `/armpi/state`
  (`std_msgs/String`), and `/armpi/pose_estimated` (`geometry_msgs/PoseStamped`)
- provides `/armpi/get_state` (`std_srvs/Trigger`); the resulting state is
  published on `/armpi/state`

### Pi (ROS 1)

The Pi node publishes vendor messages to:

- `/chassis_control/set_velocity` (`chassis_control/SetVelocity`)
- `/servo_controllers/port_id_1/multi_id_pos_dur`
  (`hiwonder_servo_msgs/MultiRawIdPosDur`)

It subscribes to `/servo_controllers/port_id_1/servo_states` for telemetry.

## Static Ethernet example

The supplied networkd examples use an isolated cable network:

| Host | Address |
| --- | --- |
| Jetson | `192.168.50.10/24` |
| Pi | `192.168.50.20/24` |

Install the matching `.network` file as `/etc/systemd/network/20-armpi-ethernet.network`
on each host and adjust `Name=` to its Ethernet interface (often `eth0`). Do
not install both files on one machine. The node defaults match these values.

## Build and run

Copy this directory to both computers, or keep it in a shared repository.  The
small `protocol/` directory must be on `PYTHONPATH` for both nodes.

On the Pi (after sourcing its Noetic workspace and launching the vendor servo
controller and `chassis_control`):

```bash
cd /path/to/Jetson_ArmPi_Communication/ros1_ws && catkin_make
source devel/setup.bash
export PYTHONPATH=/path/to/Jetson_ArmPi_Communication/protocol:$PYTHONPATH
rosrun armpi_jetson_bridge armpi_to_jetson.py _bind_host:=192.168.50.20 _port:=8765
```

On the Jetson:

```bash
cd /path/to/Jetson_ArmPi_Communication/ros2_ws
colcon build --packages-select jetson_armpi_bridge
. install/setup.bash
export PYTHONPATH=/path/to/Jetson_ArmPi_Communication/protocol:$PYTHONPATH
ros2 run jetson_armpi_bridge jetson_to_armpi --ros-args \
  -p pi_host:=192.168.50.20 -p port:=8765
```

Run `python3 -m unittest discover -s tests -v` from this directory to test
protocol framing and validation locally.
