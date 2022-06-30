from __future__ import annotations
from dataclasses import dataclass, field
import sys
import asyncio
from uuid import UUID, uuid4

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

from aiortc import MediaStreamTrack, RTCSessionDescription, RTCPeerConnection
from sensor_msgs.msg import Image

from nicepynode import Job, JobCfg
from nicepynode.aioutils import to_thread, wait_coro
from nicepynode.utils import Symbol
from aiortc_ros_msgs.srv import Handshake

# Currently on version 1.3.2 of aiortc
# See https://github.com/aiortc/aiortc/blob/1.3.2/src/aiortc/mediastreams.py
# or specifically RemoteStreamTrack in https://github.com/aiortc/aiortc/blob/main/src/aiortc/rtcrtpreceiver.py
# PeerConnection emits "track" event when browser sends track
# track from client is of type MediaStreamTrack, most impt is recv() method
# which returns either av.audio.frame.AudioFrame or av.video.frame.VideoFrame
# depending on audio or video track

NODE_NAME = "TEST"

WAITING = Symbol("WAITING")

cv_bridge = CvBridge()


class VideoRecvTrack(MediaStreamTrack):

    kind = "video"

    def __init__(self, track, transf) -> None:
        super().__init__()


@dataclass
class RTCRecvConfig(JobCfg):
    frames_out_topic: str = "~/{node}/frames_out"
    """Topic to publish received video frames to."""
    connect_service: str = "~/{node}/connect"
    """Service for exchanging WebRTC SDPs to establish connection."""
    ice_candidate_topic: str = "~/{node}/ice_candidates"
    """Topic to send trickled ICE candidates to."""
    ice_info_service: str = "~/{node}/get_ice_servers"
    """Get ICE servers available."""


@dataclass
class RTCReceiver(Job[RTCRecvConfig]):
    """Receives WebRTC Video & publishes it as sensor_msgs/Image."""

    ini_cfg: RTCRecvConfig = field(default_factory=RTCRecvConfig)

    def attach_params(self, node, cfg: RTCRecvConfig):
        super(RTCReceiver, self).attach_params(node, cfg)

        node.declare_parameter("frames_out_topic", cfg.frames_out_topic)
        node.declare_parameter("connect_service", cfg.connect_service)
        node.declare_parameter("ice_candidate_topic", cfg.ice_candidate_topic)
        node.declare_parameter("ice_info_service", cfg.ice_info_service)

    def on_params_change(self, node, changes):
        self.log.info(f"Config changed: {changes}.")
        self.log.info(f"Config change requires restart. Restarting...")
        self.restart()
        return True

    def attach_behaviour(self, node, cfg: RTCRecvConfig):
        super(RTCReceiver, self).attach_behaviour(node, cfg)

        self._conn_srv = node.create_service(
            Handshake, cfg.connect_service, self._conn_cb
        )

    def detach_behaviour(self, node):
        super(RTCReceiver, self).detach_behaviour(node)

        node.destroy_service(self._conn_srv)

    def __post_init__(self):
        super(RTCReceiver, self).__post_init__()

        self._conns: dict[UUID, RTCPeerConnection] = {}
        # asyncio event loop
        self.loop: asyncio.AbstractEventLoop = None

    def _assert_loop(self):
        assert (
            not self.loop is None
        ), f"{type(self).__name__} must be run within an asyncio loop context!"

    def _conn_cb(self, req: Handshake.Request, res: Handshake.Response):
        self._assert_loop()
        conn_id = uuid4()
        self._conns[conn_id] = WAITING

        conn = RTCPeerConnection()  # TODO: configure iceServers
        client_sdp = RTCSessionDescription(req.offer.sdp, req.offer.type)

        wait_coro(conn.setRemoteDescription(client_sdp))
        server_sdp = wait_coro(conn.createAnswer())
        wait_coro(conn.setLocalDescription(server_sdp))

        res.answer.sdp = conn.localDescription.sdp
        res.answer.type = conn.localDescription.type

        self._conns[conn_id] = conn

        return res


async def loop(job: RTCReceiver):
    job.loop = asyncio.get_running_loop()
    rostask = to_thread(rclpy.spin, job.node)
    await rostask


def main(args=None):
    if __name__ == "__main__" and args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = Node(NODE_NAME)

    cfg = RTCRecvConfig(max_rate=120)

    asyncio.run(loop(RTCReceiver(node, cfg)))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
