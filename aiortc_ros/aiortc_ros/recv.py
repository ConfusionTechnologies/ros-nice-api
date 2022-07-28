from __future__ import annotations
from dataclasses import dataclass, field
import sys
from copy import copy
import asyncio
import traceback

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from sensor_msgs.msg import Image, CompressedImage

from nicepynode import Job, JobCfg
from nicepynode.aioutils import to_thread
from aiortc_ros_msgs.srv import Handshake
from aiortc_ros_msgs.msg import IceCandidate
from aiortc_ros.rtc_manager import RTCManager

NODE_NAME = "rtc_receiver"

# Realtime Profile: don't wait for slow subscribers
rt_profile = copy(QoSPresetProfiles.SENSOR_DATA.value)
rt_profile.depth = 10


@dataclass
class RTCRecvConfig(JobCfg):
    frames_out_topic: str = "~/frames_out"
    """Topic to publish received video frames to."""
    connect_service: str = "~/connect"
    """Service for exchanging WebRTC SDPs to establish connection."""
    ice_candidate_topic: str = "~/ice_candidates"
    """Topic to send trickled ICE candidates to."""
    ice_info_service: str = "~/get_ice_servers"
    """Get ICE servers available. TODO: implement this."""
    rate: float = 144.0
    use_compression: bool = True
    """Only necessary if sending high res (>480p) over rosbridge."""


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
        return True

    def attach_behaviour(self, node, cfg: RTCRecvConfig):
        super(RTCReceiver, self).attach_behaviour(node, cfg)

        self._conn_srv = node.create_service(
            Handshake, cfg.connect_service, self._on_connection
        )
        self._ice_sub = node.create_subscription(
            IceCandidate, cfg.ice_candidate_topic, self._on_ice_candidate, 10
        )
        self._frame_pub = node.create_publisher(
            CompressedImage if cfg.use_compression else Image,
            cfg.frames_out_topic,
            rt_profile,
        )

        self.log.info("WebRTC Receiver Ready.")

    def detach_behaviour(self, node):
        super(RTCReceiver, self).detach_behaviour(node)

        node.destroy_service(self._conn_srv)
        node.destroy_subscription(self._ice_sub)

    def step(self, delta):
        if self._frame_pub.get_subscription_count() < 1:
            return

        frames = self.rtc_manager.get_frames()
        # self.log.info(str(frames))

        for uuid, frame in frames.items():
            if self.cfg.use_compression:
                _, enc = cv2.imencode(
                    ".jpg",
                    frame.to_ndarray(format="bgr24"),
                    [cv2.IMWRITE_JPEG_QUALITY, 100],
                )
                img = CompressedImage()
                img.format = "jpeg"
                img.data.frombytes(memoryview(enc))

            else:
                # see https://github.com/PyAV-Org/PyAV/blob/972f3ca096ef30c063744bdcd3c3380325408ec3/av/video/frame.pyx#L77
                # pyAV's default format (yuv420p) not supported by ROS, so reformat is needed
                plane = frame.reformat(format="bgr24").planes[0]
                img = Image(
                    height=frame.height,
                    width=frame.width,
                    # see https://github.com/ros2/common_interfaces/blob/foxy/sensor_msgs/include/sensor_msgs/image_encodings.hpp
                    encoding="bgr8",
                    is_bigendian=False,
                )
                # plane supports Python's Buffer Protocol, this is more efficient than cv_bridge which uses numpy for it
                # See: https://github.com/ros-perception/vision_opencv/issues/443
                img.data.frombytes(memoryview(plane))
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
        cfg = RTCRecvConfig()

        async def loop():
            manager = RTCManager(loop=asyncio.get_running_loop(), log=node.get_logger())
            RTCReceiver(node, cfg, manager)
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
