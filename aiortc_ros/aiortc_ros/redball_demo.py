"""Below was smth @Interpause wrote for fun, can use to test rqt's image view"""

from __future__ import annotations
from dataclasses import dataclass, field
import sys
from time import time

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from nicepynode import Job, JobCfg

NODE_NAME = "RED_BALL_DEMO"

cv_bridge = CvBridge()


@dataclass
class RedBallCfg(JobCfg):
    topic: str = "topic"
    """Topic to publish to."""
    spdX: float = 100.0
    """Horizontal speed of ball in pixels per second."""
    spdY: float = 100.0
    """Vertical speed of ball in pixels per second."""
    iniX: int = 250
    """Initial X coordinate in pixels."""
    iniY: int = 250
    """Initial Y coordinate in pixels."""
    height: int = 500
    """Height of canvas in pixels."""
    width: int = 500
    """Height of canvas in pixels."""
    size: int = 10
    """Size of ball in pixels."""


@dataclass
class RedBallImgPublisher(Job[RedBallCfg]):
    """Receives WebRTC Video & publishes it as sensor_msgs/Image."""

    ini_cfg: RedBallCfg = field(default_factory=RedBallCfg)

    def attach_params(self, node, cfg: RedBallCfg):
        super(RedBallImgPublisher, self).attach_params(node, cfg)

        node.declare_parameter("topic", cfg.topic)
        node.declare_parameter("spdX", cfg.spdX)
        node.declare_parameter("spdY", cfg.spdY)
        node.declare_parameter("iniX", cfg.iniX)
        node.declare_parameter("iniY", cfg.iniY)
        node.declare_parameter("height", cfg.height)
        node.declare_parameter("width", cfg.width)
        node.declare_parameter("size", cfg.size)

    def on_params_change(self, node, changes):
        self.log.info(f"Config changed: {changes}.")
        if any(
            n in changes
            for n in ("max_rate", "topic", "iniX", "iniY", "height", "width")
        ):
            self.log.info(f"Config change requires restart. Restarting...")
            self.restart()
        return True

    def attach_behaviour(self, node, cfg: RedBallCfg):
        super(RedBallImgPublisher, self).attach_behaviour(node, cfg)

        self._pos = (cfg.iniX, cfg.iniY)

        self._image = np.zeros((cfg.height, cfg.width, 3), np.uint8)

        self._publisher = node.create_publisher(Image, cfg.topic, 30)
        try:
            self._timer = node.create_timer(1.0 / cfg.max_rate, self._timer_cb)
        except ZeroDivisionError:
            pass
        self.log.info(f'Publishing to "{cfg.topic}" at {cfg.max_rate}Hz.')

    def detach_behaviour(self, node):
        super(RedBallImgPublisher, self).detach_behaviour(node)

        node.destroy_publisher(self._publisher)
        node.destroy_timer(self._timer)

    def _timer_cb(self):
        now = time()
        try:
            delta = now - self._prev
        except AttributeError:
            delta = 0.0
        self._prev = now
        self._pos = (
            (self._pos[0] + self.cfg.spdX * delta) % self.cfg.width,
            (self._pos[1] + self.cfg.spdY * delta) % self.cfg.height,
        )

        # reset image canvas
        self._image[...] = (0, 255, 0)
        cv2.circle(
            self._image,
            (int(self._pos[0]), int(self._pos[1])),
            self.cfg.size,
            (0, 0, 255),
            -1,
        )

        msg = cv_bridge.cv2_to_imgmsg(self._image, encoding="bgr8")
        self._publisher.publish(msg)


def main(args=None):
    if __name__ == "__main__" and args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = Node(NODE_NAME)

    cfg = RedBallCfg(max_rate=120)
    RedBallImgPublisher(node, cfg)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
