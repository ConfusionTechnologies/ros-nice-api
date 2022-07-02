from __future__ import annotations
from dataclasses import dataclass, field
import json
import sys
import asyncio
from uuid import UUID, uuid4
import traceback

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

from aiortc import (
    RTCSessionDescription,
    RTCPeerConnection,
    RTCIceCandidate,
    RTCDataChannel,
)
from sensor_msgs.msg import Image

from nicepynode import Job, JobCfg
from nicepynode.aioutils import to_thread, wait_coro
from nicepynode.utils import Symbol
from aiortc_ros_msgs.srv import Handshake
from aiortc_ros_msgs.msg import IceCandidate, SDP

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


@dataclass
class RTCManager:
    loop: asyncio.AbstractEventLoop = None

    def __post_init__(self):
        self._conns: dict[UUID, RTCPeerConnection] = {}

    def _assert_loop(self):
        """Ensure that asyncio loop for synchronous calls has been specified."""
        assert (
            not self.loop is None
        ), f"{type(self).__name__} requires asyncio loop for sync methods!"

    def _create_connection(self, conn_id) -> RTCPeerConnection:
        """Create peer connection & attach event handlers and stuff."""
        pc = RTCPeerConnection()  # TODO: configure IceServers

        def on_datachannel(chn: RTCDataChannel):
            chn.send(json.dumps(dict(code=501)))
            chn.close()

        pc.add_listener("datachannel", on_datachannel)

        return pc

    async def handshake(self, client_sdp: SDP) -> SDP:
        """Handle RTC connection SDP handshake."""
        conn_id = uuid4()
        # intermediate WAITING state to prevent adding ice candidates before connection is ready
        self._conns[conn_id] = WAITING

        conn = self._create_connection(conn_id)
        client_sdp = RTCSessionDescription(client_sdp.sdp, client_sdp.type)

        await conn.setRemoteDescription(client_sdp)
        await conn.setLocalDescription(await conn.createAnswer())

        server_sdp = SDP()
        server_sdp.sdp = conn.localDescription.sdp
        server_sdp.type = conn.localDescription.type

        self._conns[conn_id] = conn
        return server_sdp

    async def add_ice_candidate(self, candidate: IceCandidate):
        """Handle trickled ICE candidates from client."""
        conn_id = candidate.conn_uuid
        # wait for connection to be ready for additional ice candidates
        while self._conns.get(conn_id, None) == WAITING:
            await asyncio.sleep(0)
        conn = self._conns.get(conn_id, None)
        # ensure connection exists, not some errant client sending invalid candidates
        if conn:
            await conn.addIceCandidate(
                RTCIceCandidate(
                    component=candidate.component,
                    foundation=candidate.foundation,
                    ip=candidate.ip,
                    port=candidate.port,
                    priority=candidate.priority,
                    protocol=candidate.protocol,
                    type=candidate.type,
                    relatedAddress=candidate.related_address,
                    relatedPort=candidate.related_port,
                    sdpMid=candidate.sdp_mid,
                    sdpMLineIndex=candidate.sdp_mline_index,
                    tcpType=candidate.tcp_type,
                )
            )

    def handshake_sync(self, client_sdp: SDP) -> SDP:
        """Handle RTC connection SDP handshake synchronously."""
        self._assert_loop()
        return wait_coro(self.handshake(client_sdp), self.loop)

    def add_ice_candidate_sync(self, candidate: IceCandidate):
        """Handle adding trickled ICE candidate synchronously"""
        self._assert_loop()
        return wait_coro(self.add_ice_candidate(candidate), self.loop)


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
    rtc_manager: RTCManager = None

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
            Handshake, cfg.connect_service, self._on_connection
        )
        self._ice_sub = node.create_subscription(
            IceCandidate, cfg.ice_candidate_topic, self._on_ice_candidate
        )

    def detach_behaviour(self, node):
        super(RTCReceiver, self).detach_behaviour(node)

        node.destroy_service(self._conn_srv)
        node.destroy_subscription(self._ice_sub)

    def _on_connection(self, req: Handshake.Request, res: Handshake.Response):
        """Handle connect_service exchange of SDP."""
        try:
            res.answer = self.rtc_manager.handshake_sync(req.offer)
        except:
            self.log.warning(traceback.format_exc())
        return res

    def _on_ice_candidate(self, msg: IceCandidate):
        """Handle trickled ice candidates."""
        try:
            self.rtc_manager.add_ice_candidate_sync(msg)
        except:
            self.log.warning(traceback.format_exc())


def main(args=None):
    if __name__ == "__main__" and args is None:
        args = sys.argv
    try:
        rclpy.init(args=args)

        node = Node(NODE_NAME)
        cfg = RTCRecvConfig(max_rate=120)

        async def loop():
            manager = RTCManager(asyncio.get_running_loop())
            RTCReceiver(node, cfg, manager)
            rostask = to_thread(rclpy.spin, node)
            await rostask

        asyncio.run(loop())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
