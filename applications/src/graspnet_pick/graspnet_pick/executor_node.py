"""抓取执行节点：读手眼标定，把相机系抓取位姿换算到基座系并执行。

流程：服务触发 -> 取最近候选 -> 用候选时间戳对齐 TCP -> 手眼变换
     -> 开夹爪 -> 预抓取 -> 抓取 -> 闭夹爪 -> 抬起退出。
"""

from __future__ import annotations

import threading
import time
from collections import deque

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from interfaces.msg import GraspCandidate
from interfaces.srv import ExecuteGrasp

from .transforms import (
    invert_transform,
    load_extrinsics_yaml,
    matrix_to_pose,
    pose_msg_to_matrix,
    transform_from_quat,
    translation_transform,
    validate_transform,
)


def _parse_joint_names(value) -> list[str]:
    if isinstance(value, str):
        return [v.strip() for v in value.split(",") if v.strip()]
    return [str(v) for v in value]


class GraspExecutor(Node):
    def __init__(self) -> None:
        super().__init__("grasp_executor")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        self.extrinsics_yaml = str(
            self.declare_parameter(
                "extrinsics_yaml",
                "~/life_robot/config/calibration/camera_tcp_extrinsics.yaml",
            ).value
        )
        self.candidate_topic = self.declare_parameter(
            "candidate_topic", "/grasp/best_grasp"
        ).value
        self.service_name = self.declare_parameter(
            "service_name", "/grasp/execute"
        ).value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.ee_frame = self.declare_parameter("ee_frame", "link6").value
        self.camera_frame = self.declare_parameter(
            "camera_frame", "camera_optical_frame"
        ).value
        self.tcp_topic = self.declare_parameter(
            "tcp_topic", "/arm_tcp_state"
        ).value
        self.motion_topic = self.declare_parameter(
            "motion_topic", "/target_pose"
        ).value
        self.motion_status_topic = self.declare_parameter(
            "motion_status_topic", "/target_pose_status"
        ).value
        self.gripper_topic = self.declare_parameter(
            "gripper_topic", "/gripper_controller/joint_trajectory"
        ).value
        self.gripper_joints = _parse_joint_names(
            self.declare_parameter(
                "gripper_joints", "gripper_joint1,gripper_joint2"
            ).value
        )

        self.max_candidate_age_s = float(
            self.declare_parameter("max_candidate_age_s", 3.0).value
        )
        self.max_stamp_diff_s = float(
            self.declare_parameter("max_stamp_diff_s", 0.15).value
        )
        self.motion_timeout_s = float(
            self.declare_parameter("motion_timeout_s", 30.0).value
        )
        self.tf_timeout_s = float(
            self.declare_parameter("tf_timeout_s", 10.0).value
        )
        self.settle_time_s = float(
            self.declare_parameter("settle_time_s", 0.4).value
        )
        self.finger_length_m = float(
            self.declare_parameter("finger_length_m", 0.0).value
        )
        self.declare_parameter("grasp_height_offset_m", 0.0)
        self.pregrasp_distance_m = float(
            self.declare_parameter("pregrasp_distance_m", 0.08).value
        )
        self.retreat_distance_m = float(
            self.declare_parameter("retreat_distance_m", 0.08).value
        )
        self.lift_height_m = float(
            self.declare_parameter("lift_height_m", 0.05).value
        )
        self.gripper_open_m = float(
            self.declare_parameter("gripper_open_m", 0.08).value
        )
        self.gripper_margin_m = float(
            self.declare_parameter("gripper_margin_m", 0.004).value
        )
        self.gripper_max_opening_m = float(
            self.declare_parameter("gripper_max_opening_m", 0.095).value
        )
        self.gripper_joint_limit_m = float(
            self.declare_parameter("gripper_joint_limit_m", 0.05).value
        )
        self.gripper_move_time_s = float(
            self.declare_parameter("gripper_move_time_s", 1.0).value
        )
        self.gripper_settle_s = float(
            self.declare_parameter("gripper_settle_s", 1.5).value
        )
        self.min_grasp_opening_m = float(
            self.declare_parameter("min_grasp_opening_m", 0.005).value
        )
        self.prefer_vertical_approach = bool(
            self.declare_parameter("prefer_vertical_approach", True).value
        )
        self.joint_states_topic = self.declare_parameter(
            "joint_states_topic", "/joint_states"
        ).value
        self._validate_params()

        path = self.extrinsics_yaml
        self.T_tcp_cam, self.extrinsics_parent, self.extrinsics_child = (
            load_extrinsics_yaml(path)
        )
        self.get_logger().info(
            f"loaded hand-eye {self.extrinsics_parent} <- {self.extrinsics_child} "
            f"from {path}: p={self.T_tcp_cam[:3, 3].round(4).tolist()}"
        )

        self._candidate: GraspCandidate | None = None
        self._tcp_buffer: deque = deque(maxlen=1000)
        self._motion_status: str | None = None
        self._joint_positions: dict[str, float] = {}
        self._busy = False
        self._lock = threading.Lock()
        self._cond = threading.Condition()

        group = ReentrantCallbackGroup()
        self.motion_pub = self.create_publisher(PoseStamped, self.motion_topic, 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, self.gripper_topic, 10
        )
        self.create_subscription(
            GraspCandidate, self.candidate_topic, self._on_candidate, 10,
            callback_group=group,
        )
        self.create_subscription(
            PoseStamped, self.tcp_topic, self._on_tcp, 50, callback_group=group
        )
        self.create_subscription(
            String, self.motion_status_topic, self._on_motion_status, 10,
            callback_group=group,
        )
        self.create_subscription(
            JointState, self.joint_states_topic, self._on_joint_states, 50,
            callback_group=group,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=False
        )
        self.T_ee_tcp = self._lookup_ee_tcp()
        self.get_logger().info(
            f"T_{self.ee_frame}_{self.extrinsics_parent} = "
            f"{self.T_ee_tcp.round(4).tolist()}"
        )
        self.create_service(
            ExecuteGrasp, self.service_name, self._on_execute,
            callback_group=group,
        )
        self.get_logger().info(
            f"grasp_executor ready: service={self.service_name}, "
            f"motion={self.motion_topic}, gripper={self.gripper_topic}"
        )

    def _validate_params(self) -> None:
        if self.max_candidate_age_s <= 0.0:
            raise ValueError("max_candidate_age_s must be > 0")
        if self.max_stamp_diff_s <= 0.0:
            raise ValueError("max_stamp_diff_s must be > 0")
        if self.motion_timeout_s <= 0.0:
            raise ValueError("motion_timeout_s must be > 0")
        if len(self.gripper_joints) != 2:
            raise ValueError("gripper_joints must contain exactly two joints")
        if not (0.0 < self.gripper_max_opening_m <= 2.0 * self.gripper_joint_limit_m):
            raise ValueError(
                "require 0 < gripper_max_opening_m <= 2 * gripper_joint_limit_m"
            )
        if self.gripper_open_m > self.gripper_max_opening_m:
            raise ValueError("gripper_open_m must not exceed gripper_max_opening_m")
        if self.gripper_settle_s < self.gripper_move_time_s:
            raise ValueError(
                "gripper_settle_s must be >= gripper_move_time_s; "
                "otherwise joint feedback is read before the gripper stops"
            )

    def _lookup_ee_tcp(self) -> np.ndarray:
        deadline = time.monotonic() + self.tf_timeout_s
        last_error = None
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.ee_frame, self.camera_frame, Time()
                )
                T_ee_cam = transform_from_quat(
                    (
                        tf.transform.translation.x,
                        tf.transform.translation.y,
                        tf.transform.translation.z,
                    ),
                    (
                        tf.transform.rotation.x,
                        tf.transform.rotation.y,
                        tf.transform.rotation.z,
                        tf.transform.rotation.w,
                    ),
                )
                T_ee_tcp = T_ee_cam @ invert_transform(self.T_tcp_cam)
                validate_transform(T_ee_tcp, "T_ee_tcp")
                return T_ee_tcp
            except Exception as exc:  # tf2 lookup can raise several types
                last_error = exc
                rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(
            f"TF lookup {self.ee_frame} <- {self.camera_frame} failed: {last_error}"
        )

    # ------------------------------------------------------------------ callbacks
    def _on_candidate(self, msg: GraspCandidate) -> None:
        self._candidate = msg

    def _on_tcp(self, msg: PoseStamped) -> None:
        with self._cond:
            self._tcp_buffer.append(msg)
            self._cond.notify_all()

    def _on_motion_status(self, msg: String) -> None:
        with self._cond:
            self._motion_status = msg.data
            self._cond.notify_all()

    def _on_joint_states(self, msg: JointState) -> None:
        with self._cond:
            for name, position in zip(msg.name, msg.position):
                if name in self.gripper_joints:
                    self._joint_positions[name] = float(position)

    # ------------------------------------------------------------------ service
    def _on_execute(self, request, response):
        with self._lock:
            if self._busy:
                response.success = False
                response.error_code = ExecuteGrasp.Response.ERROR_BUSY
                response.message = "another grasp is running"
                return response
            self._busy = True
        try:
            return self._execute(request, response)
        except Exception as exc:
            self.get_logger().error(f"grasp execution crashed: {exc}")
            response.success = False
            response.error_code = ExecuteGrasp.Response.ERROR_INVALID_GRASP
            response.message = f"internal error: {exc}"
            return response
        finally:
            with self._lock:
                self._busy = False

    def _fail(self, response, code: int, message: str):
        response.success = False
        response.error_code = code
        response.message = message
        self.get_logger().error(f"grasp failed: {message}")
        return response

    def _execute(self, request, response):
        candidate = self._candidate
        if candidate is None:
            return self._fail(
                response, ExecuteGrasp.Response.ERROR_NO_CANDIDATE,
                "no GraspCandidate received yet",
            )

        max_age = float(request.max_age_s) or self.max_candidate_age_s
        timeout = float(request.timeout_s) or self.motion_timeout_s
        stamp = Time.from_msg(candidate.header.stamp)
        if stamp.nanoseconds == 0:
            return self._fail(
                response, ExecuteGrasp.Response.ERROR_INVALID_GRASP,
                "candidate has zero timestamp",
            )
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        if age > max_age:
            return self._fail(
                response, ExecuteGrasp.Response.ERROR_STALE_CANDIDATE,
                f"candidate age {age:.3f}s exceeds {max_age:.3f}s",
            )
        if age < -1.0:
            self.get_logger().warn(
                f"candidate stamp is {-age:.3f}s in the future; "
                "check use_sim_time / clock sources"
            )

        tcp_msg, tcp_dt = self._nearest_tcp(stamp)
        if tcp_msg is None:
            return self._fail(
                response, ExecuteGrasp.Response.ERROR_TRANSFORM,
                f"no TCP feedback within {self.max_stamp_diff_s:.3f}s of "
                f"candidate stamp",
            )

        try:
            T_cam_grasp = pose_msg_to_matrix(candidate.pose)
            validate_transform(T_cam_grasp, "T_cam_grasp")
            T_base_tcp_image = pose_msg_to_matrix(tcp_msg.pose)
            T_base_tcp_fingertip = (
                T_base_tcp_image
                @ self.T_tcp_cam
                @ T_cam_grasp
            )
            # /arm_tcp_state 的 TCP 与两指指根重合（仿真里即接触中心）。
            # finger_length_m 仅用于真实夹爪 TCP 与指腹有额外偏移的情况。
            T_base_tcp_grasp = T_base_tcp_fingertip @ translation_transform(
                0.0, 0.0, -self.finger_length_m
            )
            height_offset = float(
                self.get_parameter("grasp_height_offset_m").value
            )
            T_base_tcp_grasp[2, 3] += height_offset
        except Exception as exc:
            return self._fail(
                response, ExecuteGrasp.Response.ERROR_TRANSFORM,
                f"transform failed: {exc}",
            )

        width = float(candidate.width_m)
        if not (0.0 < width <= self.gripper_max_opening_m):
            return self._fail(
                response, ExecuteGrasp.Response.ERROR_INVALID_GRASP,
                f"grasp width {width:.4f}m is outside gripper range",
            )

        open_width = min(
            self.gripper_max_opening_m,
            max(self.gripper_open_m, width + self.gripper_margin_m),
        )
        self.get_logger().info(
            f"execute grasp: score={candidate.score:.3f} width={width * 1000:.1f}mm "
            f"open={open_width * 1000:.1f}mm tcp_dt={tcp_dt:.3f}s age={age:.3f}s"
        )

        if not self._command_gripper(open_width, "open"):
            return self._fail(
                response, ExecuteGrasp.Response.ERROR_GRIPPER,
                "failed to command gripper open",
            )

        # 同一抓取点可能有多个等价/近似的腕部朝向。先试估计器给出的朝向，
        # 规划失败再试“保持当前腕部偏航”的朝向，避免垂直抓取被腕部限位拒绝。
        rotations = self._orientation_candidates(
            T_base_tcp_grasp[:3, :3], T_base_tcp_image[:3, :3]
        )
        chosen_tcp = None
        for index, R in enumerate(rotations):
            T_base_tcp = T_base_tcp_grasp.copy()
            T_base_tcp[:3, :3] = R
            T_base_ee = T_base_tcp @ invert_transform(self.T_ee_tcp)
            if self.pregrasp_distance_m > 0.0:
                T_base_tcp_pre = T_base_tcp @ translation_transform(
                    0.0, 0.0, -self.pregrasp_distance_m
                )
                T_base_ee_pre = T_base_tcp_pre @ invert_transform(self.T_ee_tcp)
                if not self._move_to(
                    T_base_ee_pre, f"pregrasp[{index}]", timeout
                ):
                    continue
            else:
                if not self._move_to(T_base_ee, f"grasp[{index}]", timeout):
                    continue
            chosen_tcp = T_base_tcp
            break

        if chosen_tcp is None:
            self._command_gripper(open_width, "safe-open")
            return self._fail(
                response, ExecuteGrasp.Response.ERROR_MOTION,
                "pregrasp motion failed for all orientation candidates",
            )

        T_base_ee_grasp = chosen_tcp @ invert_transform(self.T_ee_tcp)
        if not self._move_to(T_base_ee_grasp, "grasp", timeout):
            self._command_gripper(open_width, "safe-open")
            return self._fail(
                response, ExecuteGrasp.Response.ERROR_MOTION,
                "grasp motion failed or timed out",
            )
        if not self._command_gripper(0.0, "close"):
            return self._fail(
                response, ExecuteGrasp.Response.ERROR_GRIPPER,
                "failed to command gripper close",
            )
        opening = self._gripper_opening()
        if opening is not None:
            self.get_logger().info(
                f"gripper opening after close: {opening * 1000:.1f}mm"
            )
        if opening is not None and opening < self.min_grasp_opening_m:
            self.get_logger().error(
                f"gripper closed empty (opening={opening * 1000:.1f}mm)"
            )
            self._command_gripper(open_width, "safe-open")
            return self._fail(
                response, ExecuteGrasp.Response.ERROR_GRIPPER,
                f"gripper closed empty (opening={opening * 1000:.1f}mm)",
            )
        T_base_tcp_retreat = chosen_tcp @ translation_transform(
            0.0, 0.0, -max(0.0, self.retreat_distance_m)
        )
        T_base_tcp_retreat[2, 3] += max(0.0, self.lift_height_m)
        T_base_ee_retreat = T_base_tcp_retreat @ invert_transform(self.T_ee_tcp)
        if not self._move_to(T_base_ee_retreat, "retreat", timeout):
            return self._fail(
                response, ExecuteGrasp.Response.ERROR_MOTION,
                "retreat motion failed; object may still be held",
            )

        response.success = True
        response.error_code = ExecuteGrasp.Response.ERROR_NONE
        response.message = (
            f"grasp succeeded: score={candidate.score:.3f} "
            f"width={width:.4f}m"
        )
        response.score = float(candidate.score)
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def _rotation_with_approach(approach, closing):
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
        return R

    def _orientation_candidates(self, R_grasp, R_current):
        candidates = []
        # 侧向抓取：从机器人前方水平接近，指面水平闭合，避免 45° 下压撞桌面。
        for approach in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, -1.0, 0.0)):
            R = self._rotation_with_approach(approach, (0.0, 1.0, 0.0))
            if R is not None:
                candidates.append(R)
        candidates.append(np.asarray(R_grasp, dtype=float))
        approach = R_grasp[:, 2]
        for closing in (R_current[:, 1], R_current[:, 0]):
            R = self._rotation_with_approach(approach, closing)
            if R is not None:
                candidates.append(R)
        R = self._rotation_with_approach(R_current[:, 2], R_grasp[:, 1])
        if R is not None:
            candidates.append(R)
        candidates.append(np.asarray(R_current, dtype=float))
        if self.prefer_vertical_approach:
            for closing in (R_current[:, 1], R_current[:, 0], R_grasp[:, 1]):
                R = self._rotation_with_approach((0.0, 0.0, -1.0), closing)
                if R is not None:
                    candidates.append(R)
        candidates.append(np.column_stack([
            -R_grasp[:, 0], -R_grasp[:, 1], R_grasp[:, 2]
        ]))
        unique = []
        for R in candidates:
            if any(
                np.linalg.norm(R - U) < 1e-6
                or self._rotation_angle(R, U) < np.deg2rad(5.0)
                for U in unique
            ):
                continue
            unique.append(R)
        return unique

    def _gripper_opening(self) -> float | None:
        with self._cond:
            values = [self._joint_positions.get(name) for name in self.gripper_joints]
        if any(v is None for v in values):
            self.get_logger().warn(
                "gripper joint feedback unavailable; skip grasp verification"
            )
            return None
        return abs(float(values[0]) - float(values[1]))

    @staticmethod
    def _rotation_angle(R1, R2):
        cos = (np.trace(np.asarray(R1).T @ np.asarray(R2)) - 1.0) * 0.5
        return float(np.arccos(np.clip(cos, -1.0, 1.0)))

    def _nearest_tcp(self, target: Time):
        best = None
        best_dt = None
        target_ns = target.nanoseconds
        for msg in reversed(self._tcp_buffer):
            msg_stamp = Time.from_msg(msg.header.stamp)
            if msg_stamp.nanoseconds == 0:
                continue
            dt = abs(msg_stamp.nanoseconds - target_ns) * 1e-9
            if best_dt is None or dt < best_dt:
                best, best_dt = msg, dt
        if best is None or best_dt is None or best_dt > self.max_stamp_diff_s:
            return None, best_dt
        return best, best_dt

    def _wait_motion_done(self, timeout_s: float) -> str | None:
        deadline = time.monotonic() + timeout_s
        terminal = ("succeeded", "failed", "rejected", "busy")
        with self._cond:
            while True:
                status = self._motion_status
                if status is not None and status.startswith(terminal):
                    return status
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._cond.wait(timeout=min(0.05, remaining))

    def _move_to(self, T_base_ee: np.ndarray, label: str, timeout_s: float) -> bool:
        position, quat = matrix_to_pose(T_base_ee)
        msg = PoseStamped()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ) = quat

        with self._cond:
            self._motion_status = None
        self.get_logger().info(
            f"{label}: p=[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]"
        )
        self.motion_pub.publish(msg)
        status = self._wait_motion_done(timeout_s)
        if status is None:
            self.get_logger().error(f"{label}: motion timeout after {timeout_s:.1f}s")
            return False
        if not status.startswith("succeeded"):
            self.get_logger().error(f"{label}: motion status '{status}'")
            return False
        time.sleep(max(0.0, self.settle_time_s))
        return True

    def _command_gripper(self, opening_m: float, label: str) -> bool:
        opening = float(np.clip(opening_m, 0.0, self.gripper_max_opening_m))
        half = opening * 0.5
        if half > self.gripper_joint_limit_m + 1e-9:
            self.get_logger().error(
                f"gripper {label}: opening {opening:.4f}m exceeds joint limit"
            )
            return False

        traj = JointTrajectory()
        traj.joint_names = list(self.gripper_joints)
        point = JointTrajectoryPoint()
        point.positions = [half, -half]
        point.time_from_start = Duration(
            seconds=max(0.1, self.gripper_move_time_s)
        ).to_msg()
        traj.points = [point]
        self.gripper_pub.publish(traj)
        self.get_logger().info(f"gripper {label}: opening={opening * 1000:.1f}mm")
        time.sleep(max(0.0, self.gripper_settle_s))
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = GraspExecutor()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
