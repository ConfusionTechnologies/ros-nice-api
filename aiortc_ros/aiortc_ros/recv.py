from __future__ import annotations
from dataclasses import dataclass, field
import sys
import asyncio
import traceback

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from nicepynode import Job, JobCfg
from nicepynode.aioutils import to_thread
from aiortc_ros_msgs.srv import Handshake
from aiortc_ros_msgs.msg import IceCandidate
from aiortc_ros.rtc_manager import RTCManager

NODE_NAME = "TEST"

cv_bridge = CvBridge()


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
