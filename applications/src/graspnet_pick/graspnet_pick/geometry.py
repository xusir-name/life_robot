"""RGB-D -> 点云 -> 6-DoF 抓取候选（纯 NumPy/SciPy，可单测）。

这里的“GraspNet”后端是几何版：用平面去除 + 聚类 + PCA OBB 生成
力闭合/反平行候选，输出与 GraspNet 相同的抓取约定：
    z = approach（指向物体）
    y = 闭合轴
    x = y × z
后续接入官方 GraspNet 网络时，只需替换 ``sample_grasps``。

全部函数不依赖 ROS，方便单测和仿真/真机复用。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

import numpy as np
from scipy.spatial import cKDTree


@dataclass
class Grasp:
    """相机坐标系下的单个抓取候选。"""

    score: float
    width_m: float
    position: np.ndarray
    rotation: np.ndarray  # 列 = [x, y, z]，z=approach, y=闭合轴
    cluster_id: int = -1

    def transform(self) -> np.ndarray:
        T = np.eye(4, dtype=float)
        T[:3, :3] = self.rotation
        T[:3, 3] = self.position
        return T


def depth_to_points(
    depth: np.ndarray,
    K: np.ndarray,
    roi: Optional[Sequence[int]] = None,
    stride: int = 1,
    min_depth_m: float = 0.05,
    max_depth_m: float = 3.0,
) -> tuple[np.ndarray, np.ndarray]:
    """深度图反投影为相机坐标系点云。

    Returns:
        points: (N, 3) 相机光学系点，单位 m
        uv: (N, 2) 对应的像素坐标 (u, v)
    """
    depth = np.asarray(depth)
    K = np.asarray(K, dtype=float).reshape(3, 3)
    h, w = depth.shape[:2]
    if roi is None:
        x1, y1, x2, y2 = 0, 0, w, h
    else:
        x1, y1, x2, y2 = (int(v) for v in roi)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
    if x2 <= x1 or y2 <= y1:
        return np.empty((0, 3), dtype=float), np.empty((0, 2), dtype=int)

    stride = max(1, int(stride))
    d = depth[y1:y2:stride, x1:x2:stride]
    ys, xs = np.mgrid[y1:y2:stride, x1:x2:stride]
    mask = np.isfinite(d) & (d > min_depth_m) & (d < max_depth_m)
    if not np.any(mask):
        return np.empty((0, 3), dtype=float), np.empty((0, 2), dtype=int)

    z = d[mask].astype(float)
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    points = np.stack(
        [(xs[mask] - cx) * z / fx, (ys[mask] - cy) * z / fy, z], axis=1
    )
    uv = np.stack([xs[mask], ys[mask]], axis=1)
    return points, uv


def voxel_downsample(points: np.ndarray, voxel_m: float) -> np.ndarray:
    """体素下采样（按体素内均值）。voxel_m <= 0 时原样返回。"""
    points = np.asarray(points, dtype=float)
    if voxel_m <= 0 or len(points) == 0:
        return points
    keys = np.floor(points / float(voxel_m)).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    sums = np.zeros((len(counts), 3), dtype=float)
    np.add.at(sums, inverse, points)
    return sums / counts[:, None]


def remove_dominant_plane(
    points: np.ndarray,
    distance_threshold_m: float = 0.006,
    min_inlier_ratio: float = 0.25,
    min_plane_points: int = 80,
    iterations: int = 200,
    seed: int = 7,
    expected_normal: Optional[np.ndarray] = None,
    max_normal_angle_rad: float = np.deg2rad(30.0),
) -> tuple[np.ndarray, Optional[tuple[np.ndarray, float]]]:
    """RANSAC 去除最大平面（桌面），返回 (剩余点, (法向, 截距))。

    给出 ``expected_normal`` 时，只接受法向与它（同向或反向）夹角不超过
    ``max_normal_angle_rad`` 的平面，避免把物体侧面当成桌面删掉。
    """
    points = np.asarray(points, dtype=float)
    if len(points) < max(50, min_plane_points):
        return points, None

    expected = None
    if expected_normal is not None:
        expected = np.asarray(expected_normal, dtype=float)
        norm = np.linalg.norm(expected)
        if norm > 1e-9:
            expected = expected / norm
        else:
            expected = None
    cos_limit = float(np.cos(max_normal_angle_rad))

    rng = np.random.default_rng(seed)
    best_inliers: Optional[np.ndarray] = None
    best_plane: Optional[tuple[np.ndarray, float]] = None
    n = len(points)
    for _ in range(max(1, int(iterations))):
        ids = rng.choice(n, 3, replace=False)
        p0, p1, p2 = points[ids]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal = normal / norm
        if expected is not None and abs(float(normal @ expected)) < cos_limit:
            continue
        offset = -float(normal @ p0)
        dist = np.abs(points @ normal + offset)
        inliers = np.where(dist < distance_threshold_m)[0]
        if best_inliers is None or len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_plane = (normal, offset)

    if best_inliers is None:
        return points, None
    if len(best_inliers) < max(min_plane_points, int(min_inlier_ratio * n)):
        return points, None
    keep = np.ones(n, dtype=bool)
    keep[best_inliers] = False
    return points[keep], best_plane


def cluster_points(
    points: np.ndarray, eps_m: float = 0.015, min_points: int = 20
) -> np.ndarray:
    """欧氏聚类（cKDTree + BFS），噪声标签为 -1。"""
    points = np.asarray(points, dtype=float)
    n = len(points)
    labels = np.full(n, -1, dtype=int)
    if n == 0:
        return labels
    if eps_m <= 0:
        return np.zeros(n, dtype=int)

    tree = cKDTree(points)
    neighbor_cache: dict[int, list[int]] = {}
    label = 0
    for i in range(n):
        if labels[i] != -1:
            continue
        labels[i] = label
        stack = [i]
        members = []
        while stack:
            j = stack.pop()
            members.append(j)
            neighbors = neighbor_cache.get(j)
            if neighbors is None:
                neighbors = tree.query_ball_point(points[j], eps_m)
                neighbor_cache[j] = neighbors
            for k in neighbors:
                if labels[k] == -1:
                    labels[k] = label
                    stack.append(k)
        if len(members) < min_points:
            labels[np.asarray(members, dtype=int)] = -1
        else:
            label += 1
    return labels


def pca_obb(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """PCA 近似有向包围盒，返回 (中心, 旋转矩阵列=局部轴, 三边长)。"""
    points = np.asarray(points, dtype=float)
    mean = points.mean(axis=0)
    centered = points - mean
    cov = centered.T @ centered / max(len(points), 1)
    _, eigenvectors = np.linalg.eigh(cov)
    axes = eigenvectors[:, ::-1]  # 方差从大到小
    if np.linalg.det(axes) < 0:
        axes[:, 2] *= -1.0
    projections = centered @ axes
    lo = projections.min(axis=0)
    hi = projections.max(axis=0)
    center = mean + axes @ ((lo + hi) * 0.5)
    extents = np.maximum(hi - lo, 1e-6)
    return center, axes, extents


def _convex_hull_2d(points: np.ndarray) -> np.ndarray:
    """Andrew monotone chain，返回逆时针凸包顶点。"""
    pts = np.unique(np.asarray(points, dtype=float), axis=0)
    if len(pts) <= 2:
        return pts
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in pts[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.asarray(lower[:-1] + upper[:-1], dtype=float)


def _min_area_rect_2d(points: np.ndarray):
    """旋转卡壳式最小面积矩形，返回 (center2, u2, v2, extent_u, extent_v)。"""
    hull = _convex_hull_2d(points)
    if len(hull) < 2:
        return np.zeros(2), np.array([1.0, 0.0]), np.array([0.0, 1.0]), 0.0, 0.0

    best = None
    for i in range(len(hull)):
        p0 = hull[i]
        p1 = hull[(i + 1) % len(hull)]
        edge = p1 - p0
        norm = np.linalg.norm(edge)
        if norm < 1e-9:
            continue
        u = edge / norm
        v = np.array([-u[1], u[0]])
        proj = hull @ np.column_stack([u, v])
        lo = proj.min(axis=0)
        hi = proj.max(axis=0)
        area = float((hi[0] - lo[0]) * (hi[1] - lo[1]))
        if best is None or area < best[0]:
            center = u * ((lo[0] + hi[0]) * 0.5) + v * ((lo[1] + hi[1]) * 0.5)
            best = (area, center, u, v, float(hi[0] - lo[0]), float(hi[1] - lo[1]))
    if best is None:
        return np.zeros(2), np.array([1.0, 0.0]), np.array([0.0, 1.0]), 0.0, 0.0
    return best[1], best[2], best[3], best[4], best[5]


def tabletop_obb(
    points: np.ndarray, up: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """桌面场景 OBB：用桌面法向约束一个轴，平面内取最小面积矩形。

    对正方体这类 PCA 特征值退化的物体，比纯 PCA 稳定得多。
    ``up`` 会被统一到“从物体指向相机”的方向。
    """
    points = np.asarray(points, dtype=float)
    mean = points.mean(axis=0)
    up = np.asarray(up, dtype=float)
    up_norm = np.linalg.norm(up)
    if up_norm < 1e-9:
        return pca_obb(points)
    up = up / up_norm
    to_camera = -mean / max(np.linalg.norm(mean), 1e-9)
    if float(up @ to_camera) < 0.0:
        up = -up

    helper = np.array([1.0, 0.0, 0.0])
    if abs(float(up @ helper)) > 0.9:
        helper = np.array([0.0, 1.0, 0.0])
    e1 = np.cross(up, helper)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(up, e1)

    proj2 = points @ np.column_stack([e1, e2])
    _, u2, v2, _, _ = _min_area_rect_2d(proj2)
    u3 = e1 * u2[0] + e2 * u2[1]
    v3 = e1 * v2[0] + e2 * v2[1]
    axes = np.column_stack([u3, v3, up])
    if np.linalg.det(axes) < 0:
        axes[:, 1] *= -1.0

    projections = points @ axes
    lo = projections.min(axis=0)
    hi = projections.max(axis=0)
    center = axes @ ((lo + hi) * 0.5)
    extents = np.maximum(hi - lo, 1e-6)
    return center, axes, extents


def _make_grasp(
    approach: np.ndarray,
    closing: np.ndarray,
    center: np.ndarray,
    width_m: float,
    score: float,
    cluster_id: int,
) -> Optional[Grasp]:
    z = np.asarray(approach, dtype=float)
    z_norm = np.linalg.norm(z)
    if z_norm < 1e-9:
        return None
    z = z / z_norm

    y = np.asarray(closing, dtype=float)
    y = y - float(y @ z) * z
    y_norm = np.linalg.norm(y)
    if y_norm < 1e-9:
        return None
    y = y / y_norm

    x = np.cross(y, z)
    x_norm = np.linalg.norm(x)
    if x_norm < 1e-9:
        return None
    x = x / x_norm
    y = np.cross(z, x)
    R = np.column_stack([x, y, z])
    if np.linalg.det(R) < 0.0:
        R[:, 0] *= -1.0
    return Grasp(score=float(score), width_m=float(width_m),
                 position=np.asarray(center, dtype=float), rotation=R,
                 cluster_id=int(cluster_id))


def rotation_angle_rad(R1: np.ndarray, R2: np.ndarray) -> float:
    """两个旋转矩阵之间的最小夹角（rad）。"""
    cos = (np.trace(np.asarray(R1).T @ np.asarray(R2)) - 1.0) * 0.5
    return float(np.arccos(np.clip(cos, -1.0, 1.0)))


def nms_grasps(
    grasps: Iterable[Grasp],
    translation_threshold_m: float = 0.02,
    rotation_threshold_rad: float = np.deg2rad(30.0),
    max_grasps: int = 200,
) -> list[Grasp]:
    """贪心 NMS：平移和旋转都接近时只保留分数最高的。"""
    ordered = sorted(grasps, key=lambda g: g.score, reverse=True)
    kept: list[Grasp] = []
    for g in ordered:
        duplicate = False
        for k in kept:
            if (
                np.linalg.norm(g.position - k.position) < translation_threshold_m
                and rotation_angle_rad(g.rotation, k.rotation) < rotation_threshold_rad
            ):
                duplicate = True
                break
        if not duplicate:
            kept.append(g)
        if len(kept) >= max_grasps:
            break
    return kept


def sample_grasps(
    points: np.ndarray,
    min_width_m: float = 0.005,
    max_width_m: float = 0.09,
    preferred_width_m: float = 0.04,
    cluster_eps_m: float = 0.015,
    cluster_min_points: int = 20,
    nms_translation_m: float = 0.02,
    nms_rotation_rad: float = np.deg2rad(30.0),
    max_grasps: int = 200,
    up: Optional[np.ndarray] = None,
) -> list[Grasp]:
    """从物体点云生成抓取候选（按分数降序，已 NMS）。

    ``up`` 为桌面法向（相机系）时使用桌面约束 OBB，否则退回 PCA OBB。
    """
    points = np.asarray(points, dtype=float)
    if len(points) < cluster_min_points:
        return []

    if not (min_width_m < max_width_m):
        raise ValueError("min_width_m must be smaller than max_width_m")
    preferred = float(np.clip(preferred_width_m, min_width_m, max_width_m))
    width_span = max(max_width_m - min_width_m, 1e-6)

    up_hint = None
    if up is not None:
        up_hint = np.asarray(up, dtype=float)
        norm = np.linalg.norm(up_hint)
        up_hint = up_hint / norm if norm > 1e-9 else None

    labels = cluster_points(points, eps_m=cluster_eps_m,
                            min_points=cluster_min_points)
    candidates: list[Grasp] = []
    for label in sorted(int(v) for v in np.unique(labels) if v >= 0):
        cluster = points[labels == label]
        if len(cluster) < cluster_min_points:
            continue
        if up is not None:
            center, axes, extents = tabletop_obb(cluster, up)
        else:
            center, axes, extents = pca_obb(cluster)
        radius = float(np.linalg.norm(center))
        if radius < 1e-6:
            continue
        to_object = center / radius  # 相机原点 -> 物体
        up_local = up_hint
        if up_local is not None and float(up_local @ to_object) > 0.0:
            up_local = -up_local

        approach_axes = [axes[:, i] for i in range(3)]
        approach_axes.append(to_object)
        for approach in approach_axes:
            a = np.asarray(approach, dtype=float)
            if float(a @ to_object) < 0.0:
                a = -a
            for i_close in range(3):
                width = float(extents[i_close])
                if not (min_width_m <= width <= max_width_m):
                    continue
                width_score = max(0.0, 1.0 - abs(width - preferred) / width_span)
                approach_score = max(0.0, float(a @ to_object))
                vertical_score = (
                    max(0.0, float(a @ (-up_local)))
                    if up_local is not None
                    else 0.0
                )
                score = (
                    0.55 * width_score
                    + 0.35 * approach_score
                    + 0.10 * vertical_score
                )
                grasp = _make_grasp(
                    a, axes[:, i_close], center, width, score, label
                )
                if grasp is not None:
                    candidates.append(grasp)

    return nms_grasps(
        candidates,
        translation_threshold_m=nms_translation_m,
        rotation_threshold_rad=nms_rotation_rad,
        max_grasps=max_grasps,
    )


def select_best_grasp(
    grasps: Iterable[Grasp], min_score: float = 0.0
) -> Optional[Grasp]:
    """返回分数最高且不低于阈值的候选。"""
    best: Optional[Grasp] = None
    for g in grasps:
        if g.score < min_score:
            continue
        if best is None or g.score > best.score:
            best = g
    return best
