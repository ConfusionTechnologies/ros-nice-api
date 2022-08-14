from __future__ import annotations

import asyncio
import json
import sys
from dataclasses import dataclass, field
from threading import Lock

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

        self._tracks: dict[str, LiveStreamTrack] = {}
        """Map of frame_id to track responsible for frame_id"""
        self._track_lock = Lock()
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
        # TODO: _remove_stale caused me A LOT of debugging trouble
        # Account for duration spent in _on_connection to avoid
        # inadvertently deleting & creating a fresh LiveStreamTrack
        # NOTE: increasing the expiry_duration > 10s is a workaround
        self._remove_stale()
        pass

    def _remove_stale(self):
        now = self.get_timestamp()

        with self._track_lock:
            to_remove = [
                k
                for k in self._tracks
                if now.sec - self._last_seen.get(k, now.sec) >= self.cfg.expiry_duration
            ]

            for frame_id in to_remove:
                self._tracks.pop(frame_id, None)
                self._last_seen.pop(frame_id, None)

    def _get_track(self, frame_id):
        with self._track_lock:
            track = self._tracks.get(frame_id, None)
            # not really the best place to put this
            self._last_seen[frame_id] = self.get_timestamp().sec
            if not track is None:
                return track
            else:
                self._tracks[frame_id] = LiveStreamTrack()
                return self._tracks[frame_id]

    def _on_input(self, msg: Image):
        # update list of frame_id available before returning if no connections to send to
        frame_id = msg.header.frame_id
        track = self._get_track(frame_id)

        if len(self.rtc_manager._conns) < 1:
            return

        if isinstance(msg, Image):
            img = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        else:
            img = cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        if 0 in img.shape:
            self.log.debug("Image has invalid shape!")
            return
        track.send_frame(img)

    def _on_connection(self, req: Handshake.Request, res: Handshake.Response):
        try:
            obj = json.loads(req.payload)
            frame_id = obj["frame_id"]
            track = self._get_track(frame_id)
            res = super()._on_connection(req, res, track)
            self.log.info(f"[{res.conn_uuid}] Added frame_id: {frame_id}")
        except Exception as e:
            self.log.error(f"Invalid payload: {req.payload}")
            raise e

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

        asyncio.run(loop())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
