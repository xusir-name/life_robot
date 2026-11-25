"""刚体变换与位姿矩阵工具（纯函数，可单测）。"""

from __future__ import annotations

from typing import Tuple

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def transform_from_quat(translation, quat_xyzw) -> np.ndarray:
    """由平移和 xyzw 四元数构造 4x4 变换矩阵。"""
    T = np.eye(4, dtype=float)
    T[:3, :3] = Rotation.from_quat(np.asarray(quat_xyzw, dtype=float)).as_matrix()
    T[:3, 3] = np.asarray(translation, dtype=float)
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """4x4 刚体变换求逆。"""
    T = np.asarray(T, dtype=float)
    R = T[:3, :3]
    t = T[:3, 3]
    out = np.eye(4, dtype=float)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def translation_transform(x: float, y: float, z: float) -> np.ndarray:
    """纯平移 4x4 变换。"""
    T = np.eye(4, dtype=float)
    T[:3, 3] = (x, y, z)
    return T


def matrix_to_quat_xyzw(R: np.ndarray) -> list[float]:
    """旋转矩阵 -> xyzw 四元数（归一化）。"""
    q = Rotation.from_matrix(np.asarray(R, dtype=float)).as_quat()
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError("invalid rotation matrix: zero quaternion")
    return (q / norm).tolist()


def pose_msg_to_matrix(pose) -> np.ndarray:
    """geometry_msgs/Pose -> 4x4。"""
    q = (
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )
    t = (float(pose.position.x), float(pose.position.y), float(pose.position.z))
    return transform_from_quat(t, q)


def matrix_to_pose(T: np.ndarray):
    """4x4 -> (position tuple, quaternion xyzw tuple)。"""
    T = np.asarray(T, dtype=float)
    return T[:3, 3].tolist(), matrix_to_quat_xyzw(T[:3, :3])


def validate_transform(T: np.ndarray, name: str = "transform") -> None:
    """检查矩阵形状、最后一行和旋转正交性。"""
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4):
        raise ValueError(f"{name}: expected 4x4, got {T.shape}")
    if not np.allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
        raise ValueError(f"{name}: last row must be [0, 0, 0, 1]")
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-5):
        raise ValueError(f"{name}: rotation is not orthonormal")
    if np.linalg.det(R) < 0:
        raise ValueError(f"{name}: rotation determinant is negative")


def load_extrinsics_yaml(path: str) -> Tuple[np.ndarray, str, str]:
    """读取标定外参，返回 (T_parent_child, parent_frame, child_frame)。

    支持 hand_eye_calibration 的三种写法：
    - 纯外参文件: ``transform``
    - 完整报告: ``transform_tcp_camera``
    - 分字段: ``translation_m`` + ``rotation_quaternion_xyzw`` / ``rotation_matrix``
    """
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a YAML mapping")

    if "transform" in data:
        T = np.asarray(data["transform"], dtype=float)
    elif "transform_tcp_camera" in data:
        T = np.asarray(data["transform_tcp_camera"], dtype=float)
    elif "translation_m" in data:
        t = np.asarray(data["translation_m"], dtype=float).reshape(3)
        if "rotation_quaternion_xyzw" in data:
            q = np.asarray(data["rotation_quaternion_xyzw"], dtype=float).reshape(4)
            T = transform_from_quat(t, q)
        elif "rotation_matrix" in data:
            R = np.asarray(data["rotation_matrix"], dtype=float).reshape(3, 3)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t
        else:
            raise ValueError(f"{path}: missing rotation")
    else:
        raise ValueError(f"{path}: no transform / translation_m found")

    validate_transform(T, path)
    parent = str(data.get("parent_frame", "tcp"))
    child = str(data.get("child_frame", "camera_optical_frame"))
    return T, parent, child
