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
    └── joint_command_test.py        # Sine-wave wrist example / smoke test
```

## Installation

```bash
pip install -e .
```

`JointStateCommandBridge` additionally requires a sourced ROS2 environment providing `rclpy` and the
`std_msgs`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `builtin_interfaces` and
`controller_manager_msgs` packages. These are not installable from PyPI and are not listed as
`pyproject.toml` dependencies; source your ROS2 workspace before using that module.
`BaseCommandBridgeABC` and `FakeCommandBridge` have no ROS dependency and work in any Python 3.10+
environment with `numpy`, `pandas` and `pin` installed.

## Usage

```python
from python_ros2_bridge.joint_command_bridge import JointStateCommandBridge

bridge = JointStateCommandBridge(
    ordered_joint_names=[...],
    threshold=1.0,
)

# Switch ros2_control controllers
bridge.switch_to_forward_position_controller_service()   # position streaming
bridge.switch_to_forward_effort_controller_service()     # effort/torque streaming

# Position commands (threshold-checked against the latest joint state)
bridge.sendCommand(q)

# Effort commands (published directly, no threshold check)
bridge.sendEffortCommand(tau)
```

`FakeCommandBridge` offers the same `getObstacles()` / `sendCommand()` interface for offline
replay of a recorded human trajectory, without any ROS dependency — see
`fake_command_bridge.py` and `human_pose_reader.py`.

## Controllers

Two `ros2_control` forward controllers are supported out of the box, both switchable via the
`/controller_manager/switch_controller` service:

| Controller | Command topic | Switch method |
|---|---|---|
| `forward_position_controller` | `/forward_position_controller/commands` | `switch_to_forward_position_controller_service()` |
| `forward_effort_controller` | `/forward_effort_controller/commands` | `switch_to_forward_effort_controller_service()` |

Both switch methods stop whichever of `forward_position_controller` /
`scaled_joint_trajectory_controller` / `forward_effort_controller` is currently active and start
the requested one (best-effort strictness, so it is safe to call regardless of the currently
active controller).
