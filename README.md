# python_ros2_bridge

ROS2/ROS-agnostic command bridge for joint control and human-obstacle perception, extracted from
[`cbf_python`](https://github.com/JRL-CARI-CNR-UNIBS/cbf_python)'s `Command_bridge` module into its own package.

## Repository Structure

```text
.
├── python_ros2_bridge/
│   ├── base_command_bridge_abc.py   # Abstract, transport-agnostic bridge interface
│   ├── joint_command_bridge.py      # ROS2 live node implementation (rclpy)
│   ├── fake_command_bridge.py       # Offline perception replay simulation bridge
│   └── human_pose_reader.py         # CSV human skeleton coordinate reader
└── tests/
    ├── joint_command_test.py        # Sine-wave wrist example / smoke test (--fake for no ROS)
    └── effort_control_test.py       # Zero-effort control with a TCP safety box (--fake for no ROS)
```

## Installation

```bash
pip install -e .
```

`JointStateCommandBridge` additionally requires a sourced ROS2 environment providing `rclpy` and the
`std_msgs`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `builtin_interfaces` and
`controller_manager_msgs` packages. These are not installable from PyPI and are not listed as
`pyproject.toml` dependencies; source your ROS2 workspace before using that module. When running
`rclpy` from a pip/conda environment, `pyyaml` and `typing_extensions` must also be installed there;
they are included in this package's dependencies.
`BaseCommandBridgeABC` and `FakeCommandBridge` have no ROS dependency and work in any Python 3.10+
environment with `numpy`, `pandas` and `pin` installed.

## Usage

```python
from python_ros2_bridge.joint_command_bridge import JointStateCommandBridge

bridge = JointStateCommandBridge(
    ordered_joint_names=[...],
    threshold=1.0,
    tcp_frame="tool0",       # TF frame used by getTcpPose()
    world_frame="world",
)

# Switch ros2_control controllers
bridge.switch_to_forward_position_controller_service()   # position streaming
bridge.switch_to_forward_effort_controller_service()     # effort/torque streaming
bridge.switch_to_trajectory_controller_service()         # back to the trajectory controller

# Position commands (threshold-checked against the latest joint state)
bridge.sendCommand(q)

# Effort commands (published directly, no threshold check)
bridge.sendEffortCommand(tau)

# TCP pose in the world frame: 4x4 homogeneous matrix world_T_tcp, looked up from TF
# on every call (None until the transform is available)
T = bridge.getTcpPose()
```

`FakeCommandBridge` offers the same `getObstacles()` / `sendCommand()` interface for offline
replay of a recorded human trajectory, without any ROS dependency. Its `getTcpPose()` computes
forward kinematics with pinocchio and requires `urdf_path=` (and `tcp_frame=`, default `tool0`);
the URDF root frame is taken as world — see
`fake_command_bridge.py` and `human_pose_reader.py`.

## Example

`tests/joint_command_test.py` drives `ur10e_wrist_3_joint` with a sine wave and prints the obstacles
it receives:

```bash
python3 tests/joint_command_test.py                    # live ROS2 (JointStateCommandBridge)
python3 tests/joint_command_test.py --fake             # no ROS, synthetic human trajectory
python3 tests/joint_command_test.py --fake --csv my_skeleton.csv --duration 10
```

With `--fake` the script uses `FakeCommandBridge`, replaying the skeleton CSV passed with `--csv`
or, if none is given, a generated two-keypoint trajectory.

### Zero-effort control with a TCP safety box

`tests/effort_control_test.py` streams a zero effort command on
`forward_effort_controller` and keeps the TCP inside an axis-aligned box expressed in the world
frame:

1. Waits for the first joint state and for the TF `world_frame` → `tcp_frame`.
2. Refuses to start if the TCP is not already inside `[lower_bound, upper_bound]`.
3. Publishes a zero effort command, then switches to `forward_effort_controller`. Publishing first
   means the controller never starts from an old command.
4. On every cycle reads the TCP pose with `getTcpPose()` and checks
   `lower_bound <= p <= upper_bound` on x, y and z, then publishes `tau = 0` again.
5. As soon as the TCP leaves the box it stops effort control and calls
   `switch_to_trajectory_controller_service()`, so `scaled_joint_trajectory_controller` holds the
   robot.

The switch back to the trajectory controller also happens when the TCP pose becomes unavailable,
on Ctrl+C, when `--duration` expires, and on any error while effort control is active.

```bash
python3 tests/effort_control_test.py --lower -0.5 -0.5 0.2 --upper 0.5 0.5 1.0
python3 tests/effort_control_test.py --fake --urdf ur10e.urdf --duration 5   # no ROS
```

With `--fake` the script uses `FakeCommandBridge`: the TCP pose comes from forward kinematics on
`--urdf` (URDF root frame = world), controller switches are skipped, and the effort command is only
stored (the fake robot has no dynamics, so it stays still).

| Argument | Default | Description |
|---|---|---|
| `--lower X Y Z` | `-0.8 -0.8 0.1` | Box lower bound in the world frame [m] |
| `--upper X Y Z` | `0.8 0.8 1.2` | Box upper bound in the world frame [m] |
| `--fake` | off | Use `FakeCommandBridge` (no ROS) |
| `--urdf` | none | Robot URDF for the fake TCP pose (required with `--fake`) |
| `--csv` | none | Skeleton CSV replayed by the fake bridge (synthetic if omitted) |
| `--tcp-frame` | `ur10e_tool0` | TF frame of the TCP |
| `--world-frame` | `world` | TF frame the box is expressed in (ignored with `--fake`) |
| `--rate` | `500` | Control loop rate [Hz] |
| `--duration` | none | Stop after this many seconds |

The default box is only a placeholder; choose bounds that fit your cell.

> **Safety:** a zero effort command only holds the arm up if the driver or hardware compensates for
> gravity. The box check only reacts once the TCP is already outside the box, so start with a small
> box and keep the emergency stop within reach.

## Controllers

Two `ros2_control` forward controllers are supported out of the box, both switchable via the
`/controller_manager/switch_controller` service:

| Controller | Command topic | Switch method |
|---|---|---|
| `forward_position_controller` | `/forward_position_controller/commands` | `switch_to_forward_position_controller_service()` |
| `forward_effort_controller` | `/forward_effort_controller/commands` | `switch_to_forward_effort_controller_service()` |

Both switch methods stop whichever of `forward_position_controller` /
`scaled_joint_trajectory_controller` / `forward_effort_controller` is currently active and start
the requested one; `switch_to_trajectory_controller_service()` does the reverse, stopping both forward
controllers and starting `scaled_joint_trajectory_controller` (best-effort strictness, so it is safe to call regardless of the currently
active controller).
