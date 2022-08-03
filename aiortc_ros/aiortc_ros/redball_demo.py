"""Below was smth @Interpause wrote for fun, can use to test rqt's image view"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nicepynode import Job, JobCfg
from nicepynode.utils import RT_PUB_PROFILE, declare_parameters_from_dataclass
from rclpy.node import Node
from sensor_msgs.msg import Image

NODE_NAME = "redball_demo"

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

        declare_parameters_from_dataclass(node, cfg)

    def on_params_change(self, node, changes):
        self.log.info(f"Config changed: {changes}.")
        if any(
            n in changes for n in ("rate", "topic", "iniX", "iniY", "height", "width")
        ):
            self.log.info(f"Config change requires restart.")
            return True
        return False

    def attach_behaviour(self, node, cfg: RedBallCfg):
        super(RedBallImgPublisher, self).attach_behaviour(node, cfg)

        self._pos = (cfg.iniX, cfg.iniY)

        self._image = np.zeros((cfg.height, cfg.width, 3), np.uint8)

        self._publisher = node.create_publisher(Image, cfg.topic, RT_PUB_PROFILE)
        self.log.info(f'Publishing to "{cfg.topic}" at {cfg.rate}Hz.')

    def detach_behaviour(self, node):
        super(RedBallImgPublisher, self).detach_behaviour(node)

        node.destroy_publisher(self._publisher)

    def step(self, delta):
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

    cfg = RedBallCfg(rate=120)
    RedBallImgPublisher(node, cfg)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
