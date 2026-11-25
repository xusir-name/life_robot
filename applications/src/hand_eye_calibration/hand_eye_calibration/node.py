"""ROS 2 node: move an arm through target poses, detect ArUco, solve hand-eye."""

from __future__ import annotations

import datetime
import os
import time
from collections import deque
from typing import Optional

import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import tf2_ros

from .aruco import detect_marker
from .hand_eye import (
    Sample,
    invert_transform,
    look_at,
    pose_to_matrix,
    rotation_angle,
    solve_eye_in_hand,
    transform_to_matrix,
)


QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
)


def _stamp_s(msg) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def _nearest(buffer, stamp: Time, max_diff_s: float):
    """Return (message, abs_diff_s) nearest to stamp, or (None, None)."""
    if not buffer:
        return None, None
    target_ns = stamp.nanoseconds
    best = min(
        buffer,
        key=lambda m: abs(
            Time.from_msg(m.header.stamp, clock_type=stamp.clock_type).nanoseconds
            - target_ns
        ),
    )
    diff = abs(
        Time.from_msg(best.header.stamp, clock_type=stamp.clock_type).nanoseconds
        - target_ns
    ) * 1e-9
    if diff > max_diff_s:
        return None, diff
    return best, diff


class HandEyeCalibrator(Node):
    def __init__(self) -> None:
        super().__init__("hand_eye_calibrator")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        # Frames / topics
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.camera_frame = self.declare_parameter("camera_frame", "camera_optical_frame").value
        self.tcp_frame = self.declare_parameter("tcp_frame", "tcp").value
        self.motion_target_frame = self.declare_parameter("motion_target_frame", "link6").value
        self.image_topic = self.declare_parameter("image_topic", "/camera/image_raw").value
        self.camera_info_topic = self.declare_parameter(
            "camera_info_topic", "/camera/camera_info"
        ).value
        self.tcp_topic = self.declare_parameter("tcp_topic", "/arm_tcp_state").value
        self.motion_topic = self.declare_parameter("motion_topic", "/target_pose").value
        self.motion_status_topic = self.declare_parameter(
            "motion_status_topic", "/target_pose_status"
        ).value

        # Marker
        self.marker_dict = self.declare_parameter("marker_dict", "DICT_6X6_250").value
        self.marker_id = int(self.declare_parameter("marker_id", 0).value)
        self.marker_size_m = float(self.declare_parameter("marker_size_m", 0.16).value)
        self.max_reproj_error_px = float(
            self.declare_parameter("max_reproj_error_px", 3.0).value
        )

        # Procedure
        self.poses_file = self.declare_parameter("poses_file", "").value
        self.output_yaml = self.declare_parameter(
            "output_yaml",
            os.path.expanduser(
                "~/life_robot/config/calibration/hand_eye_calibration.yaml"
            ),
        ).value
        self.settle_time_s = float(self.declare_parameter("settle_time_s", 1.0).value)
        self.move_timeout_s = float(self.declare_parameter("move_timeout_s", 30.0).value)
        self.capture_timeout_s = float(self.declare_parameter("capture_timeout_s", 6.0).value)
        self.capture_attempts = int(self.declare_parameter("capture_attempts", 3).value)
        self.max_time_diff_s = float(self.declare_parameter("max_time_diff_s", 0.15).value)
        self.stable_pos_tol_m = float(self.declare_parameter("stable_pos_tol_m", 0.002).value)
        self.stable_rot_tol_rad = float(
            self.declare_parameter("stable_rot_tol_rad", 0.02).value
        )
        self.settle_retries = int(self.declare_parameter("settle_retries", 2).value)
        self.min_samples = int(self.declare_parameter("min_samples", 6).value)
        self.hand_eye_method = self.declare_parameter("hand_eye_method", "TSAI").value
        self.max_consistency_translation_std_m = float(
            self.declare_parameter("max_consistency_translation_std_m", 0.01).value
        )
        self.max_consistency_rotation_rad = float(
            self.declare_parameter("max_consistency_rotation_rad", 0.05).value
        )
        self.max_sample_error_m = float(
            self.declare_parameter("max_sample_error_m", 0.005).value
        )
        self.max_sample_rotation_error_rad = float(
            self.declare_parameter("max_sample_rotation_error_rad", 0.03).value
        )
        self.max_outlier_removals = int(
            self.declare_parameter("max_outlier_removals", 3).value
        )

        self.bridge = CvBridge()
        self.image: Optional[Image] = None
        self.camera_info_buffer = deque(maxlen=50)
        self.tcp_buffer = deque(maxlen=300)
        self.status: Optional[str] = None
        self.status_history: list[str] = []

        self.create_subscription(Image, self.image_topic, self._on_image, QOS)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._on_info, QOS)
        self.create_subscription(PoseStamped, self.tcp_topic, self._on_tcp, QOS)
        self.create_subscription(String, self.motion_status_topic, self._on_status, QOS)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.motion_pub = self.create_publisher(PoseStamped, self.motion_topic, QOS)

        self.T_motion_cam: Optional[object] = None

    # ------------------------------------------------------------------ callbacks
    def _on_image(self, msg: Image) -> None:
        self.image = msg

    def _on_info(self, msg: CameraInfo) -> None:
        self.camera_info_buffer.append(msg)

    def _on_tcp(self, msg: PoseStamped) -> None:
        self.tcp_buffer.append(msg)

    def _on_status(self, msg: String) -> None:
        self.status = msg.data
        self.status_history.append(msg.data)

    # ------------------------------------------------------------------ helpers
    def _spin_until(self, predicate, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    def _wait_sim_seconds(self, seconds: float) -> None:
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_ready(self) -> None:
        self.get_logger().info("waiting for camera, camera_info and TCP feedback...")
        if not self._spin_until(
            lambda: self.image is not None
            and len(self.camera_info_buffer) > 0
            and len(self.tcp_buffer) > 0,
            30.0,
        ):
            raise RuntimeError("camera / camera_info / tcp feedback not available")

        self.get_logger().info(
            f"looking up TF {self.motion_target_frame} -> {self.camera_frame}"
        )
        deadline = time.monotonic() + 15.0
        last_error = None
        while time.monotonic() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.motion_target_frame, self.camera_frame, Time()
                )
                self.T_motion_cam = transform_to_matrix(tf)
                self.get_logger().info(
                    f"T_{self.motion_target_frame}_{self.camera_frame} = "
                    f"{self.T_motion_cam.round(4).tolist()}"
                )
                return
            except Exception as exc:  # tf2 lookup can raise several types
                last_error = exc
                rclpy.spin_once(self, timeout_sec=0.2)
        raise RuntimeError(f"TF lookup failed: {last_error}")

    def _load_poses(self) -> list[dict]:
        if not self.poses_file:
            raise RuntimeError("parameter 'poses_file' is required")
        path = os.path.expanduser(self.poses_file)
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        default_look_at = data.get("look_at")
        poses = []
        if "camera_positions" in data:
            if default_look_at is None:
                raise ValueError("poses file needs 'look_at' or per-pose 'look_at'")
            for i, pos in enumerate(data["camera_positions"]):
                if len(pos) != 3:
                    raise ValueError(f"camera_positions[{i}] must have 3 values")
                poses.append(
                    {
                        "index": i,
                        "position": [float(v) for v in pos],
                        "look_at": [float(v) for v in default_look_at],
                    }
                )
        elif "poses" in data:
            for i, item in enumerate(data["poses"]):
                look_at_point = item.get("look_at", default_look_at)
                if look_at_point is None:
                    raise ValueError(f"poses[{i}] missing look_at")
                poses.append(
                    {
                        "index": i,
                        "position": [float(v) for v in item["position"]],
                        "look_at": [float(v) for v in look_at_point],
                    }
                )
        else:
            raise ValueError("poses file needs 'camera_positions' or 'poses'")
        if len(poses) < self.min_samples:
            self.get_logger().warn(
                f"only {len(poses)} target poses configured, min_samples={self.min_samples}"
            )
        self.get_logger().info(f"loaded {len(poses)} target camera poses from {path}")
        return poses

    def _move_to_camera_pose(self, camera_position, look_at_point) -> bool:
        T_base_cam = look_at(camera_position, look_at_point)
        T_base_motion = T_base_cam @ invert_transform(self.T_motion_cam)
        q = Rotation.from_matrix(T_base_motion[:3, :3]).as_quat()
        msg = PoseStamped()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = T_base_motion[:3, 3]
        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ) = q

        self.status = None
        self.get_logger().info(
            f"move target {self.motion_target_frame}: "
            f"p={T_base_motion[:3, 3].round(3).tolist()}"
        )
        self.motion_pub.publish(msg)

        ok = self._spin_until(
            lambda: self.status is not None
            and (
                self.status.startswith("succeeded")
                or self.status.startswith("failed")
                or self.status.startswith("rejected")
                or self.status.startswith("busy")
            ),
            self.move_timeout_s,
        )
        if not ok:
            self.get_logger().error("motion timeout")
            return False
        if not self.status.startswith("succeeded"):
            self.get_logger().error(f"motion failed: {self.status}")
            return False
        return True

    def _wait_settle(self) -> bool:
        start = self.get_clock().now()
        for attempt in range(self.settle_retries + 1):
            self._wait_sim_seconds(self.settle_time_s)
            recent = [
                m
                for m in self.tcp_buffer
                if Time.from_msg(m.header.stamp, clock_type=start.clock_type) >= start
            ]
            if len(recent) < 2:
                continue
            poses = [pose_to_matrix(m.pose) for m in recent]
            max_pos = 0.0
            max_rot = 0.0
            for i in range(len(poses)):
                for j in range(i + 1, len(poses)):
                    max_pos = max(max_pos, float(abs(poses[i][:3, 3] - poses[j][:3, 3]).max()))
                    max_rot = max(max_rot, rotation_angle(poses[i][:3, :3].T @ poses[j][:3, :3]))
            self.get_logger().debug(
                f"settle attempt {attempt}: max_pos={max_pos:.4f}, max_rot={max_rot:.4f}"
            )
            if max_pos <= self.stable_pos_tol_m and max_rot <= self.stable_rot_tol_rad:
                return True
            start = self.get_clock().now()
        self.get_logger().warn("TCP did not settle within tolerance")
        return False

    def _fresh_image(self, start: Time) -> Optional[Image]:
        self.image = None
        if not self._spin_until(
            lambda: self.image is not None
            and Time.from_msg(
                self.image.header.stamp, clock_type=start.clock_type
            ) >= start,
            self.capture_timeout_s,
        ):
            return None
        return self.image

    def _capture_sample(self, pose_info: dict) -> Optional[Sample]:
        start = self.get_clock().now()
        for attempt in range(self.capture_attempts):
            image = self._fresh_image(start)
            if image is None:
                self.get_logger().warn(f"pose {pose_info['index']}: no fresh image")
                start = self.get_clock().now()
                continue
            image_stamp = Time.from_msg(
                image.header.stamp, clock_type=self.get_clock().clock_type
            )
            info, info_dt = _nearest(
                self.camera_info_buffer, image_stamp, self.max_time_diff_s
            )
            tcp, tcp_dt = _nearest(self.tcp_buffer, image_stamp, self.max_time_diff_s)
            if info is None or tcp is None:
                self.get_logger().warn(
                    f"pose {pose_info['index']}: timestamp alignment failed "
                    f"(info_dt={info_dt}, tcp_dt={tcp_dt})"
                )
                start = Time(
                    nanoseconds=image_stamp.nanoseconds + 1,
                    clock_type=image_stamp.clock_type,
                )
                continue
            try:
                arr = self.bridge.imgmsg_to_cv2(image, "bgr8")
                obs = detect_marker(
                    arr,
                    info,
                    dictionary_name=self.marker_dict,
                    marker_id=self.marker_id,
                    marker_size_m=self.marker_size_m,
                    max_reproj_error_px=self.max_reproj_error_px,
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"pose {pose_info['index']}: marker detection failed: {exc}"
                )
                start = Time(
                    nanoseconds=image_stamp.nanoseconds + 1,
                    clock_type=image_stamp.clock_type,
                )
                continue

            sample = Sample(
                T_base_tcp=pose_to_matrix(tcp.pose),
                T_cam_marker=obs.T_cam_marker,
                stamp_diff_s=max(info_dt, tcp_dt),
                reproj_error_px=obs.reproj_error_px,
                image_stamp=_stamp_s(image),
                tcp_stamp=_stamp_s(tcp),
                camera_position=pose_info["position"],
                note=f"variant={obs.variant}",
            )
            self.get_logger().info(
                f"pose {pose_info['index']}: sample ok, "
                f"dt={sample.stamp_diff_s:.3f}s, reproj={sample.reproj_error_px:.2f}px"
            )
            return sample
        return None

    def _solve_robust(self, samples: list[Sample]):
        """Solve, then remove the worst sample while its consistency error is too large."""
        working = list(samples)
        removed = []
        result = solve_eye_in_hand(working, method=self.hand_eye_method)
        for _ in range(self.max_outlier_removals):
            pos_errors = result.metrics["sample_position_errors_m"]
            rot_errors = result.metrics["sample_rotation_errors_rad"]
            worst = max(
                range(len(working)),
                key=lambda i: max(
                    pos_errors[i] / max(self.max_sample_error_m, 1e-9),
                    rot_errors[i] / max(self.max_sample_rotation_error_rad, 1e-9),
                ),
            )
            bad = (
                pos_errors[worst] > self.max_sample_error_m
                or rot_errors[worst] > self.max_sample_rotation_error_rad
            )
            if not bad or len(working) <= max(3, self.min_samples):
                break
            removed.append(working.pop(worst))
            self.get_logger().warn(
                f"remove outlier sample {worst}: "
                f"pos_err={pos_errors[worst]:.4f}m rot_err={rot_errors[worst]:.4f}rad"
            )
            result = solve_eye_in_hand(working, method=self.hand_eye_method)
        return result, working, removed

    def _write_yaml(self, result, samples: list[Sample], poses: list[dict], removed=None) -> str:
        data = result.as_dict()
        data["parent_frame"] = self.tcp_frame
        data["child_frame"] = self.camera_frame
        data["created_at"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        data["source"] = {
            "base_frame": self.base_frame,
            "motion_target_frame": self.motion_target_frame,
            "image_topic": self.image_topic,
            "camera_info_topic": self.camera_info_topic,
            "tcp_topic": self.tcp_topic,
            "motion_topic": self.motion_topic,
            "poses_file": os.path.expanduser(self.poses_file),
            "marker_dictionary": self.marker_dict,
            "marker_id": self.marker_id,
            "marker_size_m": self.marker_size_m,
        }
        data["samples_detail"] = [
            {
                "index": i,
                "camera_position_m": s.camera_position,
                "image_stamp_s": s.image_stamp,
                "tcp_stamp_s": s.tcp_stamp,
                "stamp_diff_s": s.stamp_diff_s,
                "reproj_error_px": s.reproj_error_px,
                "tcp_position_m": s.T_base_tcp[:3, 3].tolist(),
                "tcp_quaternion_xyzw": Rotation.from_matrix(
                    s.T_base_tcp[:3, :3]
                ).as_quat().tolist(),
                "note": s.note,
            }
            for i, s in enumerate(samples)
        ]
        data["pose_count"] = len(poses)
        data["removed_outliers"] = len(removed or [])
        data["removed_outlier_positions_m"] = [
            s.camera_position for s in (removed or [])
        ]

        path = os.path.expanduser(self.output_yaml)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(
                {"hand_eye_calibration": data},
                f,
                sort_keys=False,
                allow_unicode=True,
            )
        return path

    # ------------------------------------------------------------------ run
    def run(self) -> int:
        self._wait_ready()
        poses = self._load_poses()
        samples: list[Sample] = []
        for pose in poses:
            if not self._move_to_camera_pose(pose["position"], pose["look_at"]):
                self.get_logger().warn(f"skip pose {pose['index']}: motion failed")
                continue
            if not self._wait_settle():
                self.get_logger().warn(f"skip pose {pose['index']}: not settled")
                continue
            sample = self._capture_sample(pose)
            if sample is not None:
                samples.append(sample)

        self.get_logger().info(f"collected {len(samples)} valid samples")
        if len(samples) < self.min_samples:
            self.get_logger().error(
                f"not enough samples: {len(samples)} < {self.min_samples}"
            )
            return 1
        if len(samples) < 3:
            self.get_logger().error("hand-eye calibration needs at least 3 samples")
            return 1

        result, used_samples, removed = self._solve_robust(samples)
        metrics = result.metrics
        self.get_logger().info(
            "T_tcp_cam translation=%s quat_xyzw=%s"
            % (
                result.T_tcp_cam[:3, 3].round(5).tolist(),
                Rotation.from_matrix(result.T_tcp_cam[:3, :3]).as_quat().round(5).tolist(),
            )
        )
        self.get_logger().info(f"metrics: {metrics}")

        quality_ok = (
            metrics["marker_position_std_m"] <= self.max_consistency_translation_std_m
            and metrics["marker_rotation_max_error_rad"] <= self.max_consistency_rotation_rad
        )
        if not quality_ok:
            self.get_logger().warn(
                "calibration consistency is poor; check marker visibility / timestamp alignment"
            )

        path = self._write_yaml(result, used_samples, poses, removed)
        self.get_logger().info(
            f"wrote {path} ({len(used_samples)} samples, {len(removed)} outliers removed)"
        )
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandEyeCalibrator()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    except Exception as exc:
        node.get_logger().error(f"hand-eye calibration failed: {exc}")
        code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)
