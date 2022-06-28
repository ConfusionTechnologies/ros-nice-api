from __future__ import annotations
from dataclasses import dataclass, field
import sys
from time import time

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

from aiortc import MediaStreamTrack
from sensor_msgs.msg import Image

from nicepynode import Job, JobCfg

# Currently on version 1.3.2 of aiortc
# See https://github.com/aiortc/aiortc/blob/1.3.2/src/aiortc/mediastreams.py
# PeerConnection emits "track" event when browser sends track
# track from client is of type MediaStreamTrack, most impt is recv() method
# which returns either av.audio.frame.AudioFrame or av.video.frame.VideoFrame
# depending on audio or video track

NODE_NAME = "TEST"

cv_bridge = CvBridge()


class VideoRecvTrack(MediaStreamTrack):

    kind = "video"

    def __init__(self, track, transf) -> None:
        super().__init__()


@dataclass
class RTCRecvConfig(JobCfg):
    topic: str = "topic"
    """Topic to publish to."""


@dataclass
class RTCReceiver(Job[RTCRecvConfig]):
    """Receives WebRTC Video & publishes it as sensor_msgs/Image."""

    ini_cfg: RTCRecvConfig = field(default_factory=RTCRecvConfig)

    def attach_params(self, node, cfg: RTCRecvConfig):
        super(RTCReceiver, self).attach_params(node, cfg)

        node.declare_parameter("topic", cfg.topic)

    def attach_behaviour(self, node, cfg: RTCRecvConfig):
        super(RTCReceiver, self).attach_behaviour(node, cfg)

        self._spd = (250, 10)  # (x,y) px per second
        self._pos = (20, 0)  # in px
        self._size = 10

        self._image = np.zeros((480, 640, 3), np.uint8)

        self._publisher = node.create_publisher(Image, cfg.topic, 30)
        try:
            self._timer = node.create_timer(1.0 / cfg.max_rate, self._timer_cb)
        except ZeroDivisionError:
            pass
        self.log.info(f'Publishing to "{cfg.topic}" at {cfg.max_rate}Hz.')

    def detach_behaviour(self, node):
        super(RTCReceiver, self).detach_behaviour(node)

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
            (self._pos[0] + self._spd[0] * delta) % self._image.shape[1],
            (self._pos[1] + self._spd[1] * delta) % self._image.shape[0],
        )

        # reset image canvas
        self._image[...] = (0, 255, 0)
        cv2.circle(
            self._image,
            (int(self._pos[0]), int(self._pos[1])),
            self._size,
            (0, 0, 255),
            -1,
        )

        msg = cv_bridge.cv2_to_imgmsg(self._image, encoding="bgr8")
        self._publisher.publish(msg)

    def on_params_change(self, node, changes):
        self.log.info(f"Config changed: {changes}.")
        if any(n in changes for n in ("max_rate", "topic", "pub_size")):
            self.log.info(f"Config change requires restart. Restarting...")
            self.restart()
        return True


def main(args=None):
    if __name__ == "__main__" and args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = Node(NODE_NAME)

    cfg = RTCRecvConfig(max_rate=120)
    RTCReceiver(node, cfg)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
