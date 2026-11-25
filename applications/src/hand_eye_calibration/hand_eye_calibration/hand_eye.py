"""Pure math for eye-in-hand hand-eye calibration.

Conventions (REP-103):
- base frame: x forward, y left, z up.
- camera optical frame: x right, y down, z forward (OpenCV/REP-103 optical).
- ``T_a_b`` means "pose of frame b expressed in frame a".
- ``calibrateHandEye`` returns ``T_tcp_cam`` (camera in gripper/TCP frame).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Sequence

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


# OpenCV method names -> constants.
HAND_EYE_METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def make_transform(rotation: np.ndarray, translation: Sequence[float]) -> np.ndarray:
    """Build a 4x4 matrix from a 3x3 rotation and 3-vector translation."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(rotation, dtype=np.float64)
    T[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """Fast inverse of a rigid 4x4 transform."""
    R = T[:3, :3]
    t = T[:3, 3]
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def pose_to_matrix(pose) -> np.ndarray:
    """geometry_msgs/Pose or PoseStamped.pose -> 4x4."""
    if hasattr(pose, "pose"):
        pose = pose.pose
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_quat(q).as_matrix()
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return T


def transform_to_matrix(transform) -> np.ndarray:
    """geometry_msgs/TransformStamped -> 4x4."""
    tr = transform.transform
    q = [tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w]
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_quat(q).as_matrix()
    T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    return T


def matrix_to_quaternion(T: np.ndarray) -> list[float]:
    """Return quaternion as [x, y, z, w]."""
    return Rotation.from_matrix(T[:3, :3]).as_quat().tolist()


def look_at(camera_position: Sequence[float], target: Sequence[float]) -> np.ndarray:
    """Pose of the camera optical frame looking from camera_position at target.

    Camera z is forward (towards target), x is right, y is down.
    """
    cam = np.asarray(camera_position, dtype=np.float64)
    tgt = np.asarray(target, dtype=np.float64)
    z = tgt - cam
    norm = np.linalg.norm(z)
    if norm < 1e-9:
        raise ValueError("camera_position and target are the same point")
    z /= norm
    x = np.cross(z, np.array([0.0, 0.0, 1.0]))
    if np.linalg.norm(x) < 1e-9:  # looking straight up/down
        x = np.array([1.0, 0.0, 0.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    y /= np.linalg.norm(y)
    T = np.eye(4, dtype=np.float64)
    T[:3, 0] = x
    T[:3, 1] = y
    T[:3, 2] = z
    T[:3, 3] = cam
    return T


def rotation_angle(R: np.ndarray) -> float:
    """Angle of a rotation matrix, in radians."""
    value = (np.trace(R) - 1.0) / 2.0
    return float(math.acos(max(-1.0, min(1.0, value))))


def _span(values: Sequence[float]) -> float:
    return float(max(values) - min(values)) if len(values) else 0.0


@dataclass
class Sample:
    """One calibration sample."""

    T_base_tcp: np.ndarray
    T_cam_marker: np.ndarray
    stamp_diff_s: float = 0.0
    reproj_error_px: float = 0.0
    image_stamp: float = 0.0
    tcp_stamp: float = 0.0
    camera_position: Sequence[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    note: str = ""


@dataclass
class HandEyeResult:
    T_tcp_cam: np.ndarray
    T_cam_tcp: np.ndarray
    method: str
    samples: int
    metrics: dict

    def as_dict(self) -> dict:
        T = self.T_tcp_cam
        return {
            "type": "eye_in_hand",
            "parent_frame": "tcp",
            "child_frame": "camera_optical_frame",
            "method": self.method,
            "samples": self.samples,
            "translation_m": T[:3, 3].tolist(),
            "rotation_quaternion_xyzw": matrix_to_quaternion(T),
            "rotation_matrix": T[:3, :3].tolist(),
            "transform_tcp_camera": T.tolist(),
            "inverse_transform_camera_tcp": self.T_cam_tcp.tolist(),
            "metrics": self.metrics,
        }


def solve_eye_in_hand(samples: Iterable[Sample], method: str = "TSAI") -> HandEyeResult:
    """Solve AX = XB for an eye-in-hand camera.

    Args:
        samples: each contains ``T_base_tcp`` (gripper pose in base) and
            ``T_cam_marker`` (marker pose in camera).
        method: one of TSAI / PARK / HORAUD / ANDREFF / DANIILIDIS.
    """
    samples = list(samples)
    if len(samples) < 3:
        raise ValueError(f"need >= 3 samples, got {len(samples)}")
    key = method.upper()
    if key not in HAND_EYE_METHODS:
        raise ValueError(f"unknown method {method!r}, choices: {sorted(HAND_EYE_METHODS)}")

    R_gripper2base = [s.T_base_tcp[:3, :3] for s in samples]
    t_gripper2base = [s.T_base_tcp[:3, 3].reshape(3, 1) for s in samples]
    R_target2cam = [s.T_cam_marker[:3, :3] for s in samples]
    t_target2cam = [s.T_cam_marker[:3, 3].reshape(3, 1) for s in samples]

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base,
        t_gripper2base,
        R_target2cam,
        t_target2cam,
        method=HAND_EYE_METHODS[key],
    )
    T_tcp_cam = make_transform(R_cam2gripper, t_cam2gripper.ravel())
    T_cam_tcp = invert_transform(T_tcp_cam)

    # Consistency: marker in base should be the same for every sample.
    marker_in_base = [s.T_base_tcp @ T_tcp_cam @ s.T_cam_marker for s in samples]
    positions = np.array([T[:3, 3] for T in marker_in_base])
    mean_position = positions.mean(axis=0)
    pos_errors = np.linalg.norm(positions - mean_position, axis=1)
    R_ref = marker_in_base[0][:3, :3]
    rot_errors = [rotation_angle(T[:3, :3].T @ R_ref) for T in marker_in_base]

    tcp_positions = np.array([s.T_base_tcp[:3, 3] for s in samples])
    tcp_rotations = [s.T_base_tcp[:3, :3] for s in samples]
    tcp_rot_span = max(
        rotation_angle(tcp_rotations[i].T @ tcp_rotations[j])
        for i in range(len(samples))
        for j in range(i + 1, len(samples))
    ) if len(samples) > 1 else 0.0

    metrics = {
        "marker_position_mean_m": mean_position.tolist(),
        "marker_position_std_m": float(pos_errors.std()),
        "marker_position_max_error_m": float(pos_errors.max()),
        "marker_rotation_max_error_rad": float(max(rot_errors)),
        "sample_position_errors_m": pos_errors.tolist(),
        "sample_rotation_errors_rad": [float(v) for v in rot_errors],
        "stamp_diff_max_s": float(max(s.stamp_diff_s for s in samples)),
        "reproj_error_mean_px": float(np.mean([s.reproj_error_px for s in samples])),
        "reproj_error_max_px": float(np.max([s.reproj_error_px for s in samples])),
        "tcp_translation_span_m": float(
            max(np.linalg.norm(tcp_positions[i] - tcp_positions[j])
                for i in range(len(samples)) for j in range(i + 1, len(samples)))
        ) if len(samples) > 1 else 0.0,
        "tcp_rotation_span_rad": float(tcp_rot_span),
        "tcp_position_span_m": [
            _span(tcp_positions[:, 0]),
            _span(tcp_positions[:, 1]),
            _span(tcp_positions[:, 2]),
        ],
    }
    return HandEyeResult(
        T_tcp_cam=T_tcp_cam,
        T_cam_tcp=T_cam_tcp,
        method=key,
        samples=len(samples),
        metrics=metrics,
    )
