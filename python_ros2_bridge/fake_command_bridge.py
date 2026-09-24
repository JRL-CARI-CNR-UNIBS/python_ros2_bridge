#!/usr/bin/env python3
from __future__ import annotations

import time
from typing import Iterable, Tuple, Optional, Callable

import numpy as np
import pinocchio as pin

from python_ros2_bridge.base_command_bridge_abc import BaseCommandBridgeABC
from python_ros2_bridge.human_pose_reader import PoseReader  # provides PoseReader(csv, Tworld_to_cam, ...)


class FakeCommandBridge(BaseCommandBridgeABC):
    """
    Topic-free, ROS-free bridge that simulates human poses from a CSV using PoseReader.
    Implements the BaseCommandBridgeABC interface:

      - _do_publish: stores the last command, sets it as the current joint position
        (ideal tracking), and calls the optional callback
      - getObstacles(max_age_sec): returns (pos[K,3], vel[K,3], acc[K,3]) at the current simulated time
      - getTcpPose(): forward kinematics of the current joint positions (requires `urdf_path`)

    Time evolution:
      - Let T = human_trj_time.
      - We advance with a slowdown_factor (default 0.4).
      - t_raw = slowdown_factor * (now - t0).
      - t_human = triangular-wave mirror over [0, 2T]:
            if 0 <= (t_raw mod 2T) <= T  -> forward
            else                          -> backward with mirrored time and NEGATED slowdown
    """

    def __init__(
        self,
        ordered_joint_names: Iterable[str],
        *,
        threshold: float = 0.05,
        csv_path: str = "a01_s10_e02_skeleton3D_with_savgol_vel_acc.csv",
        Tworld_to_cam: Optional[pin.SE3] = None,
        slowdown_factor: float = 0.4,
        on_publish: Optional[Callable[[np.ndarray], None]] = None,
        auto_diff_if_missing: bool = False,
        t0: Optional[float] = None,
        urdf_path: Optional[str] = None,
        tcp_frame: str = "tool0",
    ) -> None:
        super().__init__(ordered_joint_names, threshold=threshold)

        # --- Build default Tworld->cam if none provided ---
        if Tworld_to_cam is None:
            R = pin.utils.rotate('z', 1.9) @ pin.utils.rotate('x', 1.57)  # ~90° about X, 1.9 rad about Z
            Tworld_to_cam = pin.SE3(R, np.array([-1.85, -0.9, 0.9]))

        # --- Pose reader & trajectory duration ---
        self._reader = PoseReader(csv_path, Tworld_to_cam, auto_diff_if_missing=auto_diff_if_missing)
        self._human_T = float(self._reader.getTotalTime())

        # --- Simulation settings ---
        self._slowdown = float(slowdown_factor)
        self._t0 = time.monotonic() if t0 is None else float(t0)
        self._on_publish = on_publish
        self.last_command: Optional[np.ndarray] = None
        self.actual_joint_positions_ = np.array([90.0, -140.0, 140.0, -90.0, 90.0, 0.0]) * np.pi / 180.0
        self.last_command = self.actual_joint_positions_.copy()
        self.actual_joint_velocities_ = np.zeros(6, dtype=float)
        self.actual_joint_accelerations_ = np.zeros(6, dtype=float)

        # --- Optional kinematic model for getTcpPose (URDF root frame = world) ---
        self._pin_model: Optional[pin.Model] = None
        if urdf_path is not None:
            self._pin_model = pin.buildModelFromUrdf(urdf_path)
            self._pin_data = self._pin_model.createData()
            if not self._pin_model.existFrame(tcp_frame):
                raise ValueError(f"TCP frame '{tcp_frame}' not found in {urdf_path}")
            self._tcp_frame_id = self._pin_model.getFrameId(tcp_frame)
            self._q_idx = []
            for name in self.ordered_joint_names_:
                if not self._pin_model.existJointName(name):
                    raise ValueError(f"Joint '{name}' not found in {urdf_path}")
                self._q_idx.append(self._pin_model.joints[self._pin_model.getJointId(name)].idx_q)

    # --------------------- ABC: command publish ---------------------
    def _do_publish(self, q: np.ndarray) -> None:
        self.last_command = q.copy()
        # Ideal tracking: the simulated robot reaches the commanded position instantly
        with self._state_lock:
            self.actual_joint_positions_[:] = q
        if self._on_publish is not None:
            try:
                self._on_publish(q.copy())
            except Exception:
                pass

    # --------------------- ABC: obstacles provider ------------------
    def getObstacles(
        self,
        elapsed: Optional[float] = None,
        max_age_sec: float = 0.5,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Returns (pos[K,3], vel[K,3], acc[K,3]) for the current simulated time.

        The CSV may contain any number of keypoints K; arrays are shaped (K,3).
        """
        if elapsed is None:
            elapsed_time = time.monotonic() - self._t0
        else:
            elapsed_time = elapsed - self._t0
        t_raw = self._slowdown * elapsed_time

        T = self._human_T
        # wrap into [0, 2T)
        tw = t_raw % (2.0 * T) if T > 0 else 0.0

        # mirror (triangular wave) + sign flip of slowdown on the backward leg
        if tw <= T:
            t_human = tw
            eff_slowdown = self._slowdown
        else:
            t_human = 2.0 * T - tw
            eff_slowdown = -self._slowdown

        # sample pose from reader (camera frame): lists of length K, each (3,)
        pos_list, vel_list, acc_list = self._reader.getHumanPose(t_human, eff_slowdown)

        # convert to (K,3) arrays
        pos = np.asarray(pos_list, dtype=float).reshape(-1, 3)
        vel = np.asarray(vel_list, dtype=float).reshape(-1, 3)
        acc = np.asarray(acc_list, dtype=float).reshape(-1, 3)
        return pos, vel, acc

    # --------------------- TCP pose (forward kinematics) ---------------------
    def getTcpPose(self) -> np.ndarray:
        """TCP pose in the world (URDF root) frame as a 4x4 homogeneous matrix (world_T_tcp)."""
        if self._pin_model is None:
            raise RuntimeError("getTcpPose requires FakeCommandBridge(urdf_path=...)")
        q = pin.neutral(self._pin_model)
        q[self._q_idx] = self.getPositions()
        pin.framesForwardKinematics(self._pin_model, self._pin_data, q)
        return self._pin_data.oMf[self._tcp_frame_id].homogeneous.copy()

    # ---------------------------- Getters ----------------------------
    def getPositions(self) -> np.ndarray:
        with self._state_lock:
            return self.actual_joint_positions_.copy()

    def getVelocities(self) -> np.ndarray:
        with self._state_lock:
            return self.actual_joint_velocities_.copy()

    def getEfforts(self) -> np.ndarray:
        with self._state_lock:
            return self.actual_joint_efforts_.copy()

    # --------------------- Convenience controls ---------------------
    def reset_time(self) -> None:
        """Restart the simulated time origin."""
        self._t0 = time.monotonic()

    def set_slowdown(self, value: float) -> None:
        """Change how fast we traverse the trajectory (sign controls direction)."""
        self._slowdown = float(value)

    def set_world_to_cam(self, T: pin.SE3) -> None:
        """Swap the camera frame on the fly (e.g., if you move the sensor)."""
        # Recreate the reader to apply a different transform without re-parsing CSV
        # (keeps timing/arrays intact by copying internals).
        self._reader = PoseReader(self._reader._csv_path, T)  # relies on PoseReader's cached CSV path
