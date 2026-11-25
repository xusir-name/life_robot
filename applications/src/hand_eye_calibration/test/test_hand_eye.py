import numpy as np
from scipy.spatial.transform import Rotation

from hand_eye_calibration.hand_eye import (
    Sample,
    invert_transform,
    look_at,
    make_transform,
    rotation_angle,
    solve_eye_in_hand,
)


def test_look_at_points_to_target():
    cam = np.array([0.2, 0.0, 0.3])
    target = np.array([0.45, 0.0, 0.127])
    T = look_at(cam, target)
    forward = T[:3, 2]
    expected = (target - cam) / np.linalg.norm(target - cam)
    assert np.allclose(forward, expected, atol=1e-9)
    assert np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-9)
    assert np.allclose(T[:3, 3], cam)


def test_invert_transform():
    T = make_transform(
        Rotation.from_euler("xyz", [0.2, -0.1, 0.3]).as_matrix(),
        [0.4, -0.2, 0.1],
    )
    assert np.allclose(T @ invert_transform(T), np.eye(4), atol=1e-12)


def test_solve_eye_in_hand_recovers_known_transform():
    rng = np.random.default_rng(7)
    T_tcp_cam_true = make_transform(
        Rotation.from_euler("xyz", [0.15, -0.25, 0.35]).as_matrix(),
        [0.06, -0.03, 0.11],
    )
    T_base_marker = make_transform(
        Rotation.from_euler("xyz", [1.2, 0.2, -0.4]).as_matrix(),
        [0.45, 0.0, 0.127],
    )

    samples = []
    for _ in range(12):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        angle = rng.uniform(0.1, 0.7)
        R_base_tcp = Rotation.from_rotvec(axis * angle).as_matrix()
        t_base_tcp = rng.uniform([0.1, -0.15, 0.2], [0.35, 0.15, 0.45])
        T_base_tcp = make_transform(R_base_tcp, t_base_tcp)
        T_cam_marker = invert_transform(T_base_tcp @ T_tcp_cam_true) @ T_base_marker
        samples.append(Sample(T_base_tcp=T_base_tcp, T_cam_marker=T_cam_marker))

    result = solve_eye_in_hand(samples, method="TSAI")
    assert np.linalg.norm(result.T_tcp_cam[:3, 3] - T_tcp_cam_true[:3, 3]) < 1e-4
    assert rotation_angle(result.T_tcp_cam[:3, :3].T @ T_tcp_cam_true[:3, :3]) < 1e-3
    assert result.samples == len(samples)
    assert result.metrics["marker_position_std_m"] < 1e-6
