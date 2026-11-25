import numpy as np

from graspnet_pick.geometry import (
    Grasp,
    cluster_points,
    depth_to_points,
    nms_grasps,
    pca_obb,
    remove_dominant_plane,
    rotation_angle_rad,
    sample_grasps,
    select_best_grasp,
    tabletop_obb,
)
from graspnet_pick.transforms import (
    invert_transform,
    load_extrinsics_yaml,
    matrix_to_quat_xyzw,
    pose_msg_to_matrix,
    transform_from_quat,
    translation_transform,
    validate_transform,
)


def _box_points(center, size, rotation, n=3000, seed=0):
    rng = np.random.default_rng(seed)
    local = (rng.random((n, 3)) - 0.5) * np.asarray(size)
    return local @ np.asarray(rotation).T + np.asarray(center)


def test_depth_to_points_center_pixel():
    K = np.array([[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]])
    depth = np.full((80, 100), 0.5, dtype=np.float32)
    points, uv = depth_to_points(depth, K, roi=(45, 35, 56, 46), stride=1)
    assert len(points) == 121
    assert np.allclose(points[:, 2], 0.5)
    assert np.all(uv[:, 0] >= 45) and np.all(uv[:, 0] < 56)
    assert np.all(uv[:, 1] >= 35) and np.all(uv[:, 1] < 46)


def test_depth_to_points_rejects_invalid():
    K = np.eye(3)
    depth = np.zeros((10, 10), dtype=np.float32)
    points, uv = depth_to_points(depth, K, min_depth_m=0.1)
    assert points.shape == (0, 3)
    assert uv.shape == (0, 2)


def test_remove_dominant_plane():
    rng = np.random.default_rng(1)
    plane = np.zeros((1500, 3))
    plane[:, :2] = (rng.random((1500, 2)) - 0.5) * 0.6
    object_points = _box_points((0.0, 0.0, 0.05), (0.05, 0.05, 0.05), np.eye(3), 300)
    points = np.vstack([plane, object_points])
    rest, model = remove_dominant_plane(points, distance_threshold_m=0.003)
    assert model is not None
    assert len(rest) == len(object_points)
    assert rest[:, 2].min() > 0.02


def test_pca_obb_recovers_box():
    yaw = np.deg2rad(30.0)
    Rz = np.array(
        [[np.cos(yaw), -np.sin(yaw), 0.0], [np.sin(yaw), np.cos(yaw), 0.0],
         [0.0, 0.0, 1.0]]
    )
    center = np.array([0.1, -0.2, 0.4])
    size = np.array([0.08, 0.04, 0.03])
    points = _box_points(center, size, Rz)
    got_center, axes, extents = pca_obb(points)
    assert np.allclose(got_center, center, atol=2e-3)
    assert np.allclose(np.sort(extents), np.sort(size), atol=5e-3)
    assert np.allclose(axes.T @ axes, np.eye(3), atol=1e-8)
    assert np.linalg.det(axes) > 0.0


def test_remove_plane_respects_expected_normal():
    rng = np.random.default_rng(3)
    # 竖直平面（x=0），不应被“桌面法向 +z”接受
    plane = np.zeros((800, 3))
    plane[:, 1:] = (rng.random((800, 2)) - 0.5) * 0.4
    rest, model = remove_dominant_plane(
        plane, expected_normal=np.array([0.0, 0.0, 1.0]),
        max_normal_angle_rad=np.deg2rad(20.0),
    )
    assert model is None
    assert len(rest) == len(plane)


def test_tabletop_obb_square_side():
    rng = np.random.default_rng(4)
    points = (rng.random((3000, 3)) - 0.5) * np.array([0.04, 0.04, 0.04])
    points[:, 2] += 0.5
    center, axes, extents = tabletop_obb(points, np.array([0.0, 0.0, -1.0]))
    assert np.allclose(center, [0.0, 0.0, 0.5], atol=2e-3)
    assert np.allclose(np.sort(extents), [0.04, 0.04, 0.04], atol=3e-3)
    assert np.allclose(axes.T @ axes, np.eye(3), atol=1e-8)


def test_cluster_points_two_clusters():
    a = _box_points((-0.1, 0.0, 0.5), (0.02, 0.02, 0.02), np.eye(3), 100)
    b = _box_points((0.1, 0.0, 0.5), (0.02, 0.02, 0.02), np.eye(3), 100)
    labels = cluster_points(np.vstack([a, b]), eps_m=0.01, min_points=10)
    assert len(set(labels.tolist())) == 2


def test_sample_grasps_on_box():
    center = np.array([0.0, 0.0, 0.5])
    size = np.array([0.04, 0.04, 0.04])
    points = _box_points(center, size, np.eye(3), 4000)
    grasps = sample_grasps(
        points,
        min_width_m=0.005,
        max_width_m=0.09,
        preferred_width_m=0.04,
        cluster_eps_m=0.01,
        up=np.array([0.0, 0.0, -1.0]),
    )
    assert grasps
    best = select_best_grasp(grasps, min_score=0.0)
    assert best is not None
    assert np.allclose(best.position, center, atol=3e-3)
    assert 0.03 <= best.width_m <= 0.05
    to_object = center / np.linalg.norm(center)
    assert float(best.rotation[:, 2] @ to_object) > 0.5
    assert np.allclose(best.rotation.T @ best.rotation, np.eye(3), atol=1e-8)
    assert np.linalg.det(best.rotation) > 0.0
    assert np.allclose(best.rotation[:, 1] @ best.rotation[:, 2], 0.0, atol=1e-8)


def test_nms_and_rotation_angle():
    R = np.eye(3)
    g1 = Grasp(0.9, 0.04, np.array([0.0, 0.0, 0.5]), R)
    g2 = Grasp(0.8, 0.04, np.array([0.001, 0.0, 0.5]), R)
    g3 = Grasp(0.7, 0.04, np.array([0.05, 0.0, 0.5]), R)
    kept = nms_grasps([g1, g2, g3], translation_threshold_m=0.02,
                      rotation_threshold_rad=np.deg2rad(30))
    assert len(kept) == 2
    assert abs(rotation_angle_rad(R, R)) < 1e-8


def test_transform_helpers_roundtrip():
    T = transform_from_quat((0.1, 0.2, 0.3), (0.0, 0.0, np.sin(0.2), np.cos(0.2)))
    validate_transform(T)
    T_inv = invert_transform(T)
    assert np.allclose(T @ T_inv, np.eye(4), atol=1e-12)
    q = matrix_to_quat_xyzw(T[:3, :3])
    assert np.allclose(transform_from_quat((0, 0, 0), q)[:3, :3], T[:3, :3],
                       atol=1e-12)


def test_load_extrinsics_variants(tmp_path):
    T = transform_from_quat((0.01, -0.02, 0.03), (0.0, 0.0, 0.0, 1.0))
    path = tmp_path / "extrinsics.yaml"
    rows = "\n".join(
        "  - [" + ", ".join(f"{v:.12f}" for v in row) + "]" for row in T
    )
    path.write_text(
        "parent_frame: tcp\nchild_frame: camera_optical_frame\n"
        f"transform:\n{rows}\n",
        encoding="utf-8",
    )
    loaded, parent, child = load_extrinsics_yaml(str(path))
    assert parent == "tcp"
    assert child == "camera_optical_frame"
    assert np.allclose(loaded, T)


def test_hand_eye_chain_math():
    T_base_tcp = transform_from_quat((0.3, 0.0, 0.4), (0.0, 0.0, 0.0, 1.0))
    T_tcp_cam = transform_from_quat((0.0, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0))
    T_cam_grasp = translation_transform(0.0, 0.0, 0.5)
    T_base_grasp = T_base_tcp @ T_tcp_cam @ T_cam_grasp
    assert np.allclose(T_base_grasp[:3, 3], (0.3, 0.0, 1.0), atol=1e-12)


def test_pose_msg_to_matrix_identity():
    class P:
        class Q:
            x = y = z = 0.0
            w = 1.0

        class V:
            x = y = z = 0.0

        position = V()
        orientation = Q()

    assert np.allclose(pose_msg_to_matrix(P()), np.eye(4))
