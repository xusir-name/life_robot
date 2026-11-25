"""GraspNet 抓取位姿估计节点。

订阅 RGB-D 与图像框（像素），在框内生成 6-DoF 抓取候选，发布最高分位姿。
真正的运动由 ``grasp_executor`` 通过服务触发。
"""

from __future__ import annotations

import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest

from interfaces.msg import GraspCandidate

from .geometry import (
    depth_to_points,
    remove_dominant_plane,
    sample_grasps,
    select_best_grasp,
    voxel_downsample,
)
from .transforms import matrix_to_quat_xyzw


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
BOX_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class GraspEstimator(Node):
    def __init__(self) -> None:
        super().__init__("grasp_estimator")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        self.image_topic = self.declare_parameter(
            "image_topic", "/camera/image_raw"
        ).value
        self.depth_topic = self.declare_parameter(
            "depth_topic", "/camera/depth/image_raw"
        ).value
        self.camera_info_topic = self.declare_parameter(
            "camera_info_topic", "/camera/camera_info"
        ).value
        self.box_topic = self.declare_parameter(
            "box_topic", "/grasp/target_box"
        ).value
        self.candidate_topic = self.declare_parameter(
            "candidate_topic", "/grasp/best_grasp"
        ).value
        self.pose_topic = self.declare_parameter(
            "pose_topic", "/grasp/best_pose"
        ).value
        self.camera_frame = self.declare_parameter(
            "camera_frame", "camera_optical_frame"
        ).value
        self.base_frame = self.declare_parameter(
            "base_frame", "base_link"
        ).value

        self.depth_scale_m = float(
            self.declare_parameter("depth_scale_m", 0.001).value
        )
        self.stride = int(self.declare_parameter("stride", 2).value)
        self.min_depth_m = float(
            self.declare_parameter("min_depth_m", 0.05).value
        )
        self.max_depth_m = float(
            self.declare_parameter("max_depth_m", 3.0).value
        )
        self.max_pair_dt_s = float(
            self.declare_parameter("max_pair_dt_s", 0.15).value
        )
        self.sensor_timeout_s = float(
            self.declare_parameter("sensor_timeout_s", 1.0).value
        )
        self.box_timeout_s = float(
            self.declare_parameter("box_timeout_s", 10.0).value
        )
        self.estimate_rate_hz = float(
            self.declare_parameter("estimate_rate_hz", 5.0).value
        )
        self.backend = str(self.declare_parameter("backend", "geometric").value)

        self.remove_plane = bool(
            self.declare_parameter("remove_plane", True).value
        )
        self.plane_distance_m = float(
            self.declare_parameter("plane_distance_m", 0.012).value
        )
        self.plane_min_ratio = float(
            self.declare_parameter("plane_min_ratio", 0.25).value
        )
        self.plane_min_points = int(
            self.declare_parameter("plane_min_points", 80).value
        )
        self.plane_iterations = int(
            self.declare_parameter("plane_iterations", 200).value
        )
        self.plane_max_normal_angle_rad = float(
            self.declare_parameter("plane_max_normal_angle_rad", 0.436).value
        )
        self.voxel_m = float(self.declare_parameter("voxel_m", 0.003).value)
        self.cluster_eps_m = float(
            self.declare_parameter("cluster_eps_m", 0.008).value
        )
        self.cluster_min_points = int(
            self.declare_parameter("cluster_min_points", 20).value
        )
        self.min_width_m = float(
            self.declare_parameter("min_width_m", 0.005).value
        )
        self.max_width_m = float(
            self.declare_parameter("max_width_m", 0.09).value
        )
        self.preferred_width_m = float(
            self.declare_parameter("preferred_width_m", 0.04).value
        )
        self.nms_translation_m = float(
            self.declare_parameter("nms_translation_m", 0.02).value
        )
        self.nms_rotation_rad = float(
            self.declare_parameter("nms_rotation_rad", 0.5236).value
        )
        self.max_grasps = int(self.declare_parameter("max_grasps", 200).value)
        self.min_score = float(self.declare_parameter("min_score", 0.25).value)

        self._validate_params()
        if self.backend != "geometric":
            raise ValueError(
                f"unsupported backend '{self.backend}': only 'geometric' is available "
                "until the official GraspNet checkpoint/runtime is installed"
            )

        self.bridge = CvBridge()
        self._depth: np.ndarray | None = None
        self._depth_stamp: Time | None = None
        self._rgb_stamp: Time | None = None
        self._K: np.ndarray | None = None
        self._box: RegionOfInterest | None = None
        self._box_stamp: Time | None = None
        self._last_estimate_key: tuple | None = None

        self.candidate_pub = self.create_publisher(
            GraspCandidate, self.candidate_topic, 10
        )
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=False
        )
        self.create_subscription(
            Image, self.image_topic, self._on_image, SENSOR_QOS
        )
        self.create_subscription(
            Image, self.depth_topic, self._on_depth, SENSOR_QOS
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, SENSOR_QOS
        )
        self.create_subscription(
            RegionOfInterest, self.box_topic, self._on_box, BOX_QOS
        )
        period = 1.0 / max(self.estimate_rate_hz, 0.1)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f"grasp_estimator ready: backend={self.backend}, "
            f"depth={self.depth_topic}, box={self.box_topic}, "
            f"candidate={self.candidate_topic}"
        )

    def _validate_params(self) -> None:
        if self.stride < 1:
            raise ValueError("stride must be >= 1")
        if self.max_depth_m <= self.min_depth_m:
            raise ValueError("max_depth_m must be greater than min_depth_m")
        if self.estimate_rate_hz <= 0.0:
            raise ValueError("estimate_rate_hz must be > 0")
        if not (0.0 < self.min_width_m < self.max_width_m):
            raise ValueError("require 0 < min_width_m < max_width_m")
        if not (self.min_width_m <= self.preferred_width_m <= self.max_width_m):
            raise ValueError("preferred_width_m must be within [min_width_m, max_width_m]")
        if self.cluster_min_points < 3:
            raise ValueError("cluster_min_points must be >= 3")

    # ------------------------------------------------------------------ callbacks
    def _on_image(self, msg: Image) -> None:
        self._rgb_stamp = Time.from_msg(msg.header.stamp)

    def _on_depth(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:  # cv_bridge raises several types
            self.get_logger().error(f"depth conversion failed: {exc}")
            return
        depth = np.asarray(depth)
        if depth.ndim != 2:
            self.get_logger().error(f"unsupported depth shape: {depth.shape}")
            return
        if np.issubdtype(depth.dtype, np.integer):
            depth = depth.astype(np.float32) * self.depth_scale_m
        else:
            depth = depth.astype(np.float32)
        self._depth = depth
        self._depth_stamp = Time.from_msg(msg.header.stamp)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        K = np.asarray(msg.k, dtype=float).reshape(3, 3)
        if K[0, 0] <= 0.0 or K[1, 1] <= 0.0:
            self.get_logger().error("camera_info has invalid fx/fy")
            return
        self._K = K

    def _on_box(self, msg: RegionOfInterest) -> None:
        if msg.width == 0 or msg.height == 0:
            self.get_logger().warn("ignore empty ROI")
            return
        self._box = msg
        self._box_stamp = self.get_clock().now()

    # ------------------------------------------------------------------ estimate
    def _tick(self) -> None:
        if self._depth is None or self._K is None or self._box is None:
            return
        if self._depth_stamp is None or self._box_stamp is None:
            return

        now = self.get_clock().now()
        if (now - self._box_stamp).nanoseconds * 1e-9 > self.box_timeout_s:
            return
        if (now - self._depth_stamp).nanoseconds * 1e-9 > self.sensor_timeout_s:
            self.get_logger().warn(
                "depth image is stale", throttle_duration_sec=2.0
            )
            return
        if self._rgb_stamp is not None:
            pair_dt = abs(
                (self._rgb_stamp - self._depth_stamp).nanoseconds
            ) * 1e-9
            if pair_dt > self.max_pair_dt_s:
                self.get_logger().warn(
                    f"RGB/depth stamp mismatch {pair_dt:.3f}s",
                    throttle_duration_sec=2.0,
                )
                return

        box = self._box
        roi = (
            int(box.x_offset),
            int(box.y_offset),
            int(box.x_offset + box.width),
            int(box.y_offset + box.height),
        )
        key = (self._depth_stamp.nanoseconds, roi)
        if key == self._last_estimate_key:
            return

        try:
            self._estimate(roi)
            self._last_estimate_key = key
        except Exception as exc:
            self.get_logger().error(f"grasp estimation failed: {exc}")

    def _table_normal_camera(self, stamp: Time) -> np.ndarray | None:
        """返回 base_link +Z 在相机系中的方向（桌面法向）。"""
        tf = None
        for query_stamp in (stamp, Time()):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, self.camera_frame, query_stamp
                )
                break
            except Exception:
                continue
        if tf is None:
            self.get_logger().warn(
                "TF unavailable, fallback to PCA OBB", throttle_duration_sec=5.0
            )
            return None
        q = tf.transform.rotation
        # R_base_cam 的第三行 = base +Z 在相机系的坐标
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )
        up = R[2, :].copy()
        norm = np.linalg.norm(up)
        return up / norm if norm > 1e-9 else None

    def _estimate(self, roi: tuple[int, int, int, int]) -> None:
        points, _ = depth_to_points(
            self._depth,
            self._K,
            roi=roi,
            stride=self.stride,
            min_depth_m=self.min_depth_m,
            max_depth_m=self.max_depth_m,
        )
        if len(points) < self.cluster_min_points:
            self.get_logger().warn(
                f"ROI {roi} has too few depth points ({len(points)})",
                throttle_duration_sec=2.0,
            )
            return

        up = self._table_normal_camera(self._depth_stamp)
        if self.remove_plane:
            points, _ = remove_dominant_plane(
                points,
                distance_threshold_m=self.plane_distance_m,
                min_inlier_ratio=self.plane_min_ratio,
                min_plane_points=self.plane_min_points,
                iterations=self.plane_iterations,
                expected_normal=up,
                max_normal_angle_rad=self.plane_max_normal_angle_rad,
            )
        points = voxel_downsample(points, self.voxel_m)
        if len(points) < self.cluster_min_points:
            self.get_logger().warn(
                f"ROI {roi} has too few object points after filtering "
                f"({len(points)})",
                throttle_duration_sec=2.0,
            )
            return

        grasps = sample_grasps(
            points,
            min_width_m=self.min_width_m,
            max_width_m=self.max_width_m,
            preferred_width_m=self.preferred_width_m,
            cluster_eps_m=self.cluster_eps_m,
            cluster_min_points=self.cluster_min_points,
            nms_translation_m=self.nms_translation_m,
            nms_rotation_rad=self.nms_rotation_rad,
            max_grasps=self.max_grasps,
            up=up,
        )
        best = select_best_grasp(grasps, min_score=self.min_score)
        if best is None:
            self.get_logger().warn(
                f"no valid grasp in ROI {roi} "
                f"(points={len(points)}, candidates={len(grasps)})",
                throttle_duration_sec=2.0,
            )
            return

        q = matrix_to_quat_xyzw(best.rotation)
        candidate = GraspCandidate()
        candidate.header.stamp = self._depth_stamp.to_msg()
        candidate.header.frame_id = self.camera_frame
        candidate.pose.position.x = float(best.position[0])
        candidate.pose.position.y = float(best.position[1])
        candidate.pose.position.z = float(best.position[2])
        (
            candidate.pose.orientation.x,
            candidate.pose.orientation.y,
            candidate.pose.orientation.z,
            candidate.pose.orientation.w,
        ) = q
        candidate.score = float(best.score)
        candidate.width_m = float(best.width_m)
        candidate.source_box = self._box
        candidate.backend = self.backend
        self.candidate_pub.publish(candidate)

        pose_msg = PoseStamped()
        pose_msg.header = candidate.header
        pose_msg.pose = candidate.pose
        self.pose_pub.publish(pose_msg)

        self.get_logger().info(
            f"best grasp score={best.score:.3f} width={best.width_m * 1000:.1f}mm "
            f"p={best.position.round(3).tolist()} "
            f"candidates={len(grasps)} points={len(points)}",
            throttle_duration_sec=1.0,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = GraspEstimator()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
