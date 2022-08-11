from __future__ import annotations

import asyncio
import sys
import traceback
from copy import copy
from dataclasses import dataclass, field

import cv2
import rclpy
from aiortc_ros_msgs.msg import IceCandidate
from aiortc_ros_msgs.srv import Handshake
from nicepynode import Job, JobCfg
from nicepynode.aioutils import to_thread
from nicepynode.utils import (
    RT_PUB_PROFILE,
    declare_parameters_from_dataclass,
    letterbox,
)
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from aiortc_ros.rtc_manager import RTCManager

NODE_NAME = "rtc_receiver"

RT_PUB_PROFILE = copy(RT_PUB_PROFILE)
RT_PUB_PROFILE.depth = 2


@dataclass
class RTCRecvConfig(JobCfg):
    rate: float = 120.0
    frames_out_topic: str = "~/frames_out"
    """Topic to publish received video frames to."""
    connect_service: str = "~/connect"
    """Service for exchanging WebRTC SDPs to establish connection."""
    ice_candidate_topic: str = "~/ice_candidates"
    """Topic to send trickled ICE candidates to."""
    ice_info_service: str = "~/get_ice_servers"
    """Get ICE servers available. TODO: implement this."""
    use_compression: bool = True
    """Only necessary if sending high res (>720p) over rosbridge."""
    compression_level: int = 75
    """Compression level for image frames. Compression only necessary if sending high res over rosbridge."""
    downscale_wh: int = 0
    """When using compression, downscaling the image may be necessary due to CPU usage from image decompression."""
    debug_window: bool = False
    """Displays the video feed for debug purpposes."""


@dataclass
class RTCReceiver(Job[RTCRecvConfig]):
    """Receives WebRTC Video & publishes it as sensor_msgs/Image."""

    ini_cfg: RTCRecvConfig = field(default_factory=RTCRecvConfig)
    rtc_manager: RTCManager = None

    def attach_params(self, node, cfg: RTCRecvConfig):
        super(RTCReceiver, self).attach_params(node, cfg)

        declare_parameters_from_dataclass(node, cfg)

        self._winname = "[Livefeed] aiortc_ros"
        self._winshown = False

    def on_params_change(self, node, changes):
        self.log.info(f"Config changed: {changes}.")
        if not all(n in ("compression_level", "debug_window") for n in changes):
            self.log.info(f"Config change requires restart.")
            return True
        return False

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
            RT_PUB_PROFILE,
        )

        self.log.info("WebRTC Receiver Ready.")

    def detach_behaviour(self, node):
        super(RTCReceiver, self).detach_behaviour(node)

        node.destroy_service(self._conn_srv)
        node.destroy_subscription(self._ice_sub)
        node.destroy_publisher(self._frame_pub)

    def step(self, delta):
        if self._frame_pub.get_subscription_count() < 1:
            return

        frames = self.rtc_manager.get_frames()
        # self.log.info(str(frames))

        for uuid, frame in frames.items():
            if self.cfg.debug_window:
                cv2img = frame.to_ndarray(format="bgr24")
                cv2.namedWindow(self._winname, cv2.WINDOW_NORMAL)
                self._winshown = True
                cv2.imshow(self._winname, cv2img)
                cv2.waitKey(1)
            elif self._winshown:
                cv2.destroyWindow(self._winname)
                self._winshown = False

            if self.cfg.use_compression:
                cv2img = frame.to_ndarray(format="bgr24")
                if self.cfg.downscale_wh > 0:
                    cv2img = letterbox(
                        cv2img, (self.cfg.downscale_wh, self.cfg.downscale_wh)
                    )[0]
                _, enc = cv2.imencode(
                    ".jpg",
                    cv2img,
                    (cv2.IMWRITE_JPEG_QUALITY, self.cfg.compression_level),
                )
                img = CompressedImage()
                img.format = "jpeg"
                img.data.frombytes(memoryview(enc))

            elif self.cfg.downscale_wh > 0:
                cv2img = frame.to_ndarray(format="bgr24")
                cv2img = letterbox(
                    cv2img, (self.cfg.downscale_wh, self.cfg.downscale_wh)
                )
                img = Image(
                    height=img.shape[0],
                    width=img.shape[1],
                    encoding="bgr8",
                    is_bigendian=False,
                )
                img.data.frombytes(memoryview(cv2img))
                img.step = len(img.data) // img.height
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
