#!/usr/bin/env python3
"""
Example script: zero-effort control with a TCP safety box.

Uses JointStateCommandBridge (live ROS2):
- Waits for the first joint state and the first TCP pose
- Refuses to start if the TCP is not already inside the box
- Switches to `forward_effort_controller` and streams a zero effort command
- On every cycle checks that the TCP position (world frame) is inside
  [lower_bound, upper_bound]; as soon as it leaves the box (or the TCP pose
  becomes unavailable) it stops effort control and switches back to
  `scaled_joint_trajectory_controller`

The switch back to the trajectory controller also happens on Ctrl+C, on
--duration timeout, and on any error while effort control is active.

Run:
  python3 effort_control_test.py
  python3 effort_control_test.py --lower -0.5 -0.5 0.2 --upper 0.5 0.5 1.0
"""

import argparse
import math
import time
from typing import List, Optional

import numpy as np
import rclpy

from python_ros2_bridge.joint_command_bridge import JointStateCommandBridge


UR10E_JOINTS: List[str] = [
    "ur10e_shoulder_pan_joint",
    "ur10e_shoulder_lift_joint",
    "ur10e_elbow_joint",
    "ur10e_wrist_1_joint",
    "ur10e_wrist_2_joint",
    "ur10e_wrist_3_joint",
]


def is_inside_box(p: np.ndarray, lower_bound: np.ndarray, upper_bound: np.ndarray) -> bool:
    return bool(np.all(p >= lower_bound) and np.all(p <= upper_bound))


def wait_for_tcp_pose(bridge: JointStateCommandBridge, timeout: float = 5.0) -> Optional[np.ndarray]:
    """Block until getTcpPose() returns a transform (or timeout)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        T = bridge.getTcpPose()
        if T is not None:
            return T
        time.sleep(0.02)
    return bridge.getTcpPose()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--lower", type=float, nargs=3, default=[-0.8, -0.8, 0.1],
                        metavar=("X", "Y", "Z"), help="box lower bound in world frame [m]")
    parser.add_argument("--upper", type=float, nargs=3, default=[0.8, 0.8, 1.2],
                        metavar=("X", "Y", "Z"), help="box upper bound in world frame [m]")
    parser.add_argument("--tcp-frame", default="ur10e_tool0", help="TF frame of the TCP")
    parser.add_argument("--world-frame", default="world", help="TF frame the box is expressed in")
    parser.add_argument("--rate", type=float, default=500.0, help="control loop rate [Hz]")
    parser.add_argument("--duration", type=float, default=None, help="stop after this many seconds")
    args = parser.parse_args()

    lower_bound = np.asarray(args.lower, dtype=float)
    upper_bound = np.asarray(args.upper, dtype=float)
    if np.any(lower_bound >= upper_bound):
        parser.error(f"lower bound {lower_bound} must be < upper bound {upper_bound} on every axis")

    rclpy.init()
    bridge = JointStateCommandBridge(
        ordered_joint_names=UR10E_JOINTS,
        tcp_frame=args.tcp_frame,
        world_frame=args.world_frame,
    )

    def shutdown():
        bridge.shutdown()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    if math.isnan(bridge.wait_for_first_state(UR10E_JOINTS[0], timeout=5.0)):
        print("No joint state received, exiting.")
        shutdown()
        return

    T = wait_for_tcp_pose(bridge, timeout=5.0)
    if T is None:
        print(f"TF '{args.world_frame}' -> '{args.tcp_frame}' not available, exiting.")
        shutdown()
        return

    if not is_inside_box(T[:3, 3], lower_bound, upper_bound):
        print(f"TCP {np.round(T[:3, 3], 3)} is outside the box "
              f"[{lower_bound}, {upper_bound}], not starting effort control.")
        shutdown()
        return

    tau = np.zeros(len(UR10E_JOINTS))
    dt = 1.0 / args.rate

    # Send a zero command before switching so the effort controller does not
    # start from a stale command.
    bridge.sendEffortCommand(tau)
    bridge.switch_to_forward_effort_controller_service()
    print(f"Zero-effort control active, box: lower={lower_bound}, upper={upper_bound}")

    t_start = time.time()
    try:
        while rclpy.ok():
            if args.duration is not None and time.time() - t_start > args.duration:
                print("Duration elapsed.")
                break

            T = bridge.getTcpPose()
            if T is None:
                print("TCP pose unavailable, stopping effort control.")
                break

            p = T[:3, 3]
            if not is_inside_box(p, lower_bound, upper_bound):
                print(f"TCP {np.round(p, 3)} left the box, stopping effort control.")
                break

            bridge.sendEffortCommand(tau)
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            bridge.switch_to_trajectory_controller_service()
        finally:
            shutdown()


if __name__ == "__main__":
    main()
