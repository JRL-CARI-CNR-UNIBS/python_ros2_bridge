#!/usr/bin/env python3
"""
Example script: drive `ur10e_wrist_3_joint` with a sine wave.

Two modes:

- **ROS2 (default)**: uses JointStateCommandBridge. Switches to
  `forward_position_controller` via the `/controller_manager/switch_controller`
  service, waits for the first joint state, then streams position commands.
- **Fake (`--fake`)**: uses FakeCommandBridge. No ROS required; human obstacles
  are replayed from a skeleton CSV (`--csv`), or from a synthetic trajectory
  generated on the fly if no CSV is given.

In both modes the script:
- Creates the bridge with UR10e joint order
- Publishes a sine motion on wrist_3 around its current position
- Reads obstacles with getObstacles()
- Catches ValueError if the command exceeds `threshold`

Run:
  python3 joint_command_test.py                 # live ROS2
  python3 joint_command_test.py --fake          # offline, synthetic human
  python3 joint_command_test.py --fake --csv a01_s10_e02_skeleton3D_with_savgol_vel_acc.csv

At start (ROS2 mode), the script will call:
  ros2 service call /controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController \
    "activate_controllers: []\n     deactivate_controllers: []\n     start_controllers: ['forward_position_controller']\n     stop_controllers: ['scaled_joint_trajectory_controller']\n     strictness: 0\n     start_asap: false\n     activate_asap: false\n     timeout: {sec: 0, nanosec: 0}"
"""

import argparse
import math
import os
import tempfile
import time
from typing import List, Optional

import numpy as np


UR10E_JOINTS: List[str] = [
    "ur10e_shoulder_pan_joint",
    "ur10e_shoulder_lift_joint",
    "ur10e_elbow_joint",
    "ur10e_wrist_1_joint",
    "ur10e_wrist_2_joint",
    "ur10e_wrist_3_joint",
]


def write_synthetic_skeleton_csv(path: str, duration: float = 10.0, dt: float = 0.05) -> None:
    """Write a minimal skeleton CSV: two keypoints walking back and forth along x."""
    t = np.arange(0.0, duration + dt, dt)
    x = 0.5 * np.sin(2.0 * math.pi * t / duration)
    cols = {"time": t}
    for k, z in (("head", 1.7), ("hand", 1.1)):
        cols[f"{k}_x"] = x
        cols[f"{k}_y"] = np.full_like(t, 2.0)
        cols[f"{k}_z"] = np.full_like(t, z)
    header = ",".join(cols)
    data = np.column_stack(list(cols.values()))
    np.savetxt(path, data, delimiter=",", header=header, comments="")


def make_fake_bridge(csv_path: Optional[str]):
    from python_ros2_bridge.fake_command_bridge import FakeCommandBridge

    if csv_path is None:
        fd, csv_path = tempfile.mkstemp(prefix="synthetic_skeleton_", suffix=".csv")
        os.close(fd)
        write_synthetic_skeleton_csv(csv_path)
        print(f"No --csv given, using synthetic human trajectory: {csv_path}")

    return FakeCommandBridge(
        ordered_joint_names=UR10E_JOINTS,
        threshold=1.0,
        csv_path=csv_path,
        auto_diff_if_missing=True,  # synthetic CSV has positions only
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--fake", action="store_true", help="use FakeCommandBridge (no ROS)")
    parser.add_argument("--csv", default=None, help="skeleton CSV replayed by the fake bridge")
    parser.add_argument("--duration", type=float, default=None, help="stop after this many seconds")
    args = parser.parse_args()

    if args.fake:
        bridge = make_fake_bridge(args.csv)
        ok = lambda: True
    else:
        import rclpy
        from python_ros2_bridge.joint_command_bridge import JointStateCommandBridge

        rclpy.init()
        bridge = JointStateCommandBridge(
            ordered_joint_names=UR10E_JOINTS,
            threshold=1.0,  # radians (or native units)
            tcp_frame="ur10e_tool0",
        )
        ok = rclpy.ok

    def shutdown():
        if args.fake:
            return
        bridge.shutdown()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    # Wait (briefly) for a first state to center the sine around it
    target_name = "ur10e_wrist_3_joint"
    idx = UR10E_JOINTS.index(target_name)

    first_joint_position = bridge.wait_for_first_state(target_name, timeout=5.0)
    if math.isnan(first_joint_position):
        shutdown()
        return

    if not args.fake:
        print(f"TCP pose in world frame:\n{bridge.getTcpPose()}")
        bridge.switch_to_forward_position_controller_service()

    amp = 0.3         # amplitude (rad); keep < threshold to avoid errors
    freq = 0.2        # Hz
    center = first_joint_position
    print(f"Driving {target_name} with sine: center={center:.3f}, amp={amp}, freq={freq} Hz")
    t_start = time.time()

    try:
        while ok():
            t = time.time() - t_start
            if args.duration is not None and t > args.duration:
                break
            wrist3 = center + amp * math.sin(2.0 * math.pi * freq * t)

            # Build a command vector based on the latest known positions
            q = bridge.getPositions()
            # if NaNs, shut down
            if np.isnan(q).any():
                return

            q[idx] = wrist3

            pos, vel, acc = bridge.getObstacles()

            if len(pos) > 0:
                print(f"received {len(pos)} obstacles, first at {np.round(pos[0], 3)}")

            try:
                bridge.sendCommand(q)
            except ValueError as e:
                # Exceeded threshold — print and continue (you may choose to break)
                print(f"Threshold error: {e}")

            time.sleep(0.02)  # ~50 Hz update
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()


if __name__ == "__main__":
    main()
