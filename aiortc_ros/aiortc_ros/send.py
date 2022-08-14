from __future__ import annotations

import asyncio
import json
import sys
from collections import defaultdict
from dataclasses import dataclass, field

import numpy as np
import rclpy
from aiortc import VideoStreamTrack
from aiortc_ros_msgs.srv import Handshake, ListCams
from av import VideoFrame
from cv_bridge import CvBridge
from nicepynode.aioutils import to_thread
from nicepynode.utils import RT_SUB_PROFILE
from rclpy.node import Node
from ros2topic.api import get_msg_class
from sensor_msgs.msg import Image

from aiortc_ros.cfg import RTCSendConfig
from aiortc_ros.rtc_manager import RTCManager
from aiortc_ros.rtc_node import RTCNode

NODE_NAME = "rtc_sender"

cv_bridge = CvBridge()


class LiveStreamTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.counter = 0
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[..., :3] = (0, 255, 0)
        self.frame = VideoFrame.from_ndarray(img, format="bgr24")

    def send_frame(self, img):
        self.frame = VideoFrame.from_ndarray(img, format="bgr24")

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        frame = self.frame
        frame.pts = pts
        frame.time_base = time_base
        self.counter += 1
        return frame


@dataclass
class RTCSender(RTCNode[RTCSendConfig]):
    """Receives WebRTC Video & publishes it as sensor_msgs/Image."""

    ini_cfg: RTCSendConfig = field(default_factory=RTCSendConfig)

    def attach_params(self, node, cfg: RTCSendConfig):
        super(RTCSender, self).attach_params(node, cfg)

        self._tracks = defaultdict(LiveStreamTrack)
        """Map of frame_id to track responsible for frame_id"""
        self._last_seen = {}
        """When frame_id was last seen"""

        self._winname = f"[{self.node.get_name()}] Livefeed"
        self._winshown = False

    def on_params_change(self, node, changes):
        self.log.info(f"Config changed: {changes}.")
        if not all(n in ("expiry_duration",) for n in changes):
            self.log.info(f"Config change requires restart.")
            return True
        return False

    def attach_behaviour(self, node, cfg: RTCSendConfig):
        super(RTCSender, self).attach_behaviour(node, cfg)

        # TODO: make this isomorphic realtime image subscriber a utility
        self.log.info(f"Waiting for publisher@{cfg.frames_in_topic}...")
        self._frames_sub = node.create_subscription(
            # blocks until image publisher is up!
            get_msg_class(node, cfg.frames_in_topic, blocking=True),
            cfg.frames_in_topic,
            self._on_input,
            RT_SUB_PROFILE,
        )

        self._cam_srv = node.create_service(
            ListCams, cfg.list_cam_service, self._on_list_cams
        )

        self.log.info("WebRTC Sender Ready.")

    def detach_behaviour(self, node):
        super(RTCSender, self).detach_behaviour(node)

        node.destroy_publisher(self._frames_sub)
        node.destroy_service(self._cam_srv)

    def step(self, delta):
        self._remove_stale()

    def _remove_stale(self):
        now = self.get_timestamp()

        to_remove = [
            k
            for k in self._tracks
            if now.sec - self._last_seen[k].sec >= self.cfg.expiry_duration
        ]

        for frame_id in to_remove:
            self._tracks.pop(frame_id)
            self._last_seen.pop(frame_id)

    def _on_input(self, msg: Image):
        # update list of frame_id available before returning if no connections to send to
        frame_id = msg.header.frame_id
        track = self._tracks[frame_id]
        self._last_seen[frame_id] = self.get_timestamp()

        if len(self.rtc_manager._conns) < 1:
            return

        if isinstance(msg, Image):
            img = cv_bridge.imgmsg_to_cv2(msg, "rgb8")
        else:
            img = cv_bridge.compressed_imgmsg_to_cv2(msg, "rgb8")
        if 0 in img.shape:
            self.log.debug("Image has invalid shape!")
            return
        track.send_frame(img)

    def _on_connection(self, req: Handshake.Request, res: Handshake.Response):
        res = super()._on_connection(req, res)

        try:
            obj = json.loads(req.payload)
            frame_id = obj["frame_id"]
            conn = self.rtc_manager._conns[res.conn_uuid]
            conn.addTrack(self._tracks[frame_id])
        except:
            self.log.warning(f"Invalid payload: {req.payload}")

        return res

    def _on_list_cams(self, req: ListCams.Request, res: ListCams.Response):
        # try:
        #     obj = json.loads(req.payload)
        # except:
        #     self.log.warning(f"Invalid payload: {req.payload}")
        res.frame_ids = list(self._tracks.keys())

        return res


def main(args=None):
    if __name__ == "__main__" and args is None:
        args = sys.argv
    try:
        rclpy.init(args=args)

        node = Node(NODE_NAME)
        cfg = RTCSendConfig()

        async def loop():
            manager = RTCManager(loop=asyncio.get_running_loop(), log=node.get_logger())
            RTCSender(node, cfg, manager)
            rostask = to_thread(rclpy.spin, node)
            await rostask

        # TODO: will sometimes freeze when trying to interrupt, my guess is some
        # sort of deadlock in rtc_manager.py but i dont even know how to trace this.
        asyncio.run(loop())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
