#!/usr/bin/env python3
"""仿真测试用：用颜色阈值代替 YOLO，发布目标框到 /grasp/target_box。

这不是正式感知节点，只用于在 Gazebo 里验证
    RGB -> 目标框 -> GraspNet -> 抓取执行
的链路。正式运行请由 YOLO/检测节点发布同样的 RegionOfInterest。
"""

import argparse
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, RegionOfInterest


COLOR_RANGES = {
    "red": [((0, 80, 50), (10, 255, 255)), ((170, 80, 50), (180, 255, 255))],
    "orange": [((11, 80, 50), (25, 255, 255))],
    "yellow": [((26, 80, 50), (35, 255, 255))],
    "green": [((36, 70, 50), (85, 255, 255))],
    "blue": [((86, 70, 50), (130, 255, 255))],
    "purple": [((131, 70, 50), (169, 255, 255))],
}


class ColorBoxPublisher(Node):
    def __init__(self, image_topic: str, box_topic: str, color: str,
                 min_area: int, margin_px: int, fixed_box):
        super().__init__("sim_color_box_publisher")
        self.color = color
        self.min_area = min_area
        self.margin_px = margin_px
        self.fixed_box = fixed_box
        self.pub = self.create_publisher(RegionOfInterest, box_topic, 10)
        self.create_subscription(Image, image_topic, self._on_image, 10)
        self.get_logger().info(
            f"publishing '{color}' box from {image_topic} to {box_topic}"
        )

    def _mask(self, bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        if self.color == "auto":
            ranges = [r for rs in COLOR_RANGES.values() for r in rs]
        else:
            ranges = COLOR_RANGES[self.color]
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _on_image(self, msg: Image) -> None:
        if self.fixed_box is not None:
            x, y, w, h = self.fixed_box
        else:
            try:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as exc:
                self.get_logger().error(f"image conversion failed: {exc}")
                return
            contours, _ = cv2.findContours(
                self._mask(bgr), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                return
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) < self.min_area:
                return
            x, y, w, h = cv2.boundingRect(largest)

        image_h, image_w = msg.height, msg.width
        m = self.margin_px
        x1 = max(0, x - m)
        y1 = max(0, y - m)
        x2 = min(image_w, x + w + m)
        y2 = min(image_h, y + h + m)
        roi = RegionOfInterest()
        roi.x_offset = int(x1)
        roi.y_offset = int(y1)
        roi.width = int(max(1, x2 - x1))
        roi.height = int(max(1, y2 - y1))
        self.pub.publish(roi)

    @property
    def bridge(self):
        # 延迟创建，避免在 __init__ 里依赖 cv_bridge 的导入顺序
        if not hasattr(self, "_bridge"):
            from cv_bridge import CvBridge

            self._bridge = CvBridge()
        return self._bridge


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--box-topic", default="/grasp/target_box")
    parser.add_argument(
        "--color", default="red", choices=sorted(COLOR_RANGES) + ["auto"]
    )
    parser.add_argument("--min-area", type=int, default=300)
    parser.add_argument("--margin-px", type=int, default=2)
    parser.add_argument(
        "--box",
        default="",
        help="固定框 x,y,w,h（像素）；给出后不再做颜色检测",
    )
    args, _ = parser.parse_known_args(argv)

    fixed = None
    if args.box:
        parts = [int(v) for v in args.box.split(",")]
        if len(parts) != 4:
            parser.error("--box must be x,y,w,h")
        fixed = tuple(parts)

    rclpy.init(args=argv)
    node = ColorBoxPublisher(
        args.image_topic, args.box_topic, args.color, args.min_area,
        args.margin_px, fixed,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
