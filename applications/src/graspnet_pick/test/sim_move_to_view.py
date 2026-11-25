#!/usr/bin/env python3
"""仿真测试用：把相机移到能看见桌面的观察位姿。

用法：
    python3 test/sim_move_to_view.py \
        --camera-position 0.20,0.0,0.40 --look-at 0.45,0.0,0.10

它只发布 /target_pose（仿真 MoveIt 桥），不发布 /control/*，不是正式应用节点。
"""

import argparse
import sys

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from scipy.spatial.transform import Rotation


def look_at(camera_position, target):
    cam = np.asarray(camera_position, dtype=float)
    tgt = np.asarray(target, dtype=float)
    z = tgt - cam
    z /= np.linalg.norm(z)
    x = np.cross(z, np.array([0.0, 0.0, 1.0]))
    if np.linalg.norm(x) < 1e-9:
        x = np.array([1.0, 0.0, 0.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, cam
    return T


def invert(T):
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


class ViewMover(Node):
    def __init__(self, args):
        super().__init__("sim_move_to_view")
        self.base_frame = args.base_frame
        self.ee_frame = args.ee_frame
        self.camera_frame = args.camera_frame
        self.pub = self.create_publisher(PoseStamped, args.motion_topic, 10)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.target = args
        self.sent = False
        self.done = False
        self.status = None
        self.create_subscription(
            String, args.status_topic, self._on_status, 10
        )
        self.create_timer(0.5, self._tick)

    def _on_status(self, msg: String) -> None:
        self.status = msg.data
        if self.sent and (
            msg.data.startswith("succeeded")
            or msg.data.startswith("failed")
            or msg.data.startswith("rejected")
            or msg.data.startswith("busy")
        ):
            self.get_logger().info(f"motion status: {msg.data}")
            self.done = True

    def _tick(self):
        if self.sent:
            return
        try:
            tf = self.buffer.lookup_transform(
                self.ee_frame, self.camera_frame, Time()
            )
        except Exception:
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        T_ee_cam = np.eye(4)
        T_ee_cam[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T_ee_cam[:3, 3] = [t.x, t.y, t.z]
        T_base_cam = look_at(self.target.camera_position, self.target.look_at)
        T_base_ee = T_base_cam @ invert(T_ee_cam)
        q_target = Rotation.from_matrix(T_base_ee[:3, :3]).as_quat()
        msg = PoseStamped()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
            T_base_ee[:3, 3]
        )
        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ) = q_target
        self.pub.publish(msg)
        self.sent = True
        self.get_logger().info(
            f"sent view pose link6 p={T_base_ee[:3, 3].round(3).tolist()}"
        )


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera-position", default="0.20,0.0,0.40")
    parser.add_argument("--look-at", default="0.45,0.0,0.10")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--ee-frame", default="link6")
    parser.add_argument("--camera-frame", default="camera_optical_frame")
    parser.add_argument("--motion-topic", default="/target_pose")
    parser.add_argument("--status-topic", default="/target_pose_status")
    args, _ = parser.parse_known_args(argv)
    args.camera_position = [float(v) for v in args.camera_position.split(",")]
    args.look_at = [float(v) for v in args.look_at.split(",")]
    if len(args.camera_position) != 3 or len(args.look_at) != 3:
        parser.error("camera-position and look-at must be x,y,z")

    rclpy.init(args=argv)
    node = ViewMover(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
