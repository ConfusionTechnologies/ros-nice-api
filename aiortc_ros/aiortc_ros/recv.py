from __future__ import annotations
from dataclasses import dataclass, field
import sys
from array import array
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
    frames_out_topic: str = "~/frames_out"
    """Topic to publish received video frames to."""
    connect_service: str = "~/connect"
    """Service for exchanging WebRTC SDPs to establish connection."""
    ice_candidate_topic: str = "~/ice_candidates"
    """Topic to send trickled ICE candidates to."""
    ice_info_service: str = "~/get_ice_servers"
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
        self.log.info(f"Config change requires restart.")
        self.restart()
        return True

    def attach_behaviour(self, node, cfg: RTCRecvConfig):
        super(RTCReceiver, self).attach_behaviour(node, cfg)

        self._conn_srv = node.create_service(
            Handshake, cfg.connect_service, self._on_connection
        )
        self._ice_sub = node.create_subscription(
            IceCandidate, cfg.ice_candidate_topic, self._on_ice_candidate, 10
        )
        self._frame_pub = node.create_publisher(Image, cfg.frames_out_topic, 30)

        self.log.info("WebRTC Receiver Ready.")

    def detach_behaviour(self, node):
        super(RTCReceiver, self).detach_behaviour(node)

        node.destroy_service(self._conn_srv)
        node.destroy_subscription(self._ice_sub)

    def step(self, delta):
        frames = self.rtc_manager.get_frames()
        # self.log.info(str(frames))

        for uuid, frame in frames.items():
            img = Image()
            img.height = frame.height
            img.width = frame.width
            # see https://github.com/ros2/common_interfaces/blob/foxy/sensor_msgs/include/sensor_msgs/image_encodings.hpp
            img.encoding = "bgr8"
            # adapted from https://github.com/PyAV-Org/PyAV/blob/main/av/video/frame.pyx
            # specifically its to_image() func. prevents additional buffer copy.
            plane = frame.reformat(format="bgr24").planes[0]
            # plane supports Buffer Protocol, this is more efficient than cv_bridge which uses numpy for it
            img.data = array("B", memoryview(plane))

            img.is_bigendian = False
            img.step = len(img.data) // img.height

            # conn_id ~= camera_id ~= coordinate frame id
            img.header.frame_id = uuid
            img.header.stamp.sec = int(frame.time)
            img.header.stamp.nanosec = int((frame.time % 1) * 10 ** 9)

            self._frame_pub.publish(img)

    def _on_connection(self, req: Handshake.Request, res: Handshake.Response):
        """Handle connect_service exchange of SDP."""
        try:
            self.log.debug(f"Incoming SDP: {req.offer}")
            res.answer, res.conn_uuid = self.rtc_manager.handshake_sync(req.offer)
        except:
            self.log.warning(traceback.format_exc())
        return res

    def _on_ice_candidate(self, msg: IceCandidate):
        """Handle trickled ice candidates."""
        try:
            self.log.debug(f"Incoming candidate: {msg}")
            self.rtc_manager.add_ice_candidate_sync(msg)
        except:
            self.log.warning(traceback.format_exc())


def main(args=None):
    if __name__ == "__main__" and args is None:
        args = sys.argv
    try:
        rclpy.init(args=args)

        node = Node(NODE_NAME)
        cfg = RTCRecvConfig(rate=30)

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
