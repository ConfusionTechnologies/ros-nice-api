from __future__ import annotations

import asyncio
import sys
from copy import copy
from dataclasses import dataclass, field

import cv2
import rclpy
from aiortc_ros.cfg import RTCRecvConfig
from aiortc_ros.rtc_manager import RTCManager
from aiortc_ros.rtc_node import RTCNode
from nicepynode.aioutils import to_thread
from nicepynode.utils import RT_PUB_PROFILE, letterbox
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

NODE_NAME = "rtc_receiver"

RT_PUB_PROFILE = copy(RT_PUB_PROFILE)
RT_PUB_PROFILE.depth = 2

# did you know RTCReceiver is horizontally scalable? You can start up more instances
# of the RTCReceiver (with unique names ofc) to handle more connections.


@dataclass
class RTCReceiver(RTCNode[RTCRecvConfig]):
    """Receives WebRTC Video & publishes it as sensor_msgs/Image."""

    ini_cfg: RTCRecvConfig = field(default_factory=RTCRecvConfig)

    def attach_params(self, node, cfg: RTCRecvConfig):
        super(RTCReceiver, self).attach_params(node, cfg)

        self._winname = f"[{self.node.get_name()}] Livefeed"
        self._winshown = False

    def on_params_change(self, node, changes):
        self.log.info(f"Config changed: {changes}.")
        if not all(
            n in ("compression_level", "downscale_wh", "debug_window") for n in changes
        ):
            self.log.info(f"Config change requires restart.")
            return True
        return False

    def attach_behaviour(self, node, cfg: RTCRecvConfig):
        super(RTCReceiver, self).attach_behaviour(node, cfg)

        self._frame_pub = node.create_publisher(
            CompressedImage if cfg.use_compression else Image,
            cfg.frames_out_topic,
            RT_PUB_PROFILE,
        )

        self.log.info("WebRTC Receiver Ready.")

    def detach_behaviour(self, node):
        super(RTCReceiver, self).detach_behaviour(node)

        node.destroy_publisher(self._frame_pub)

    def _update_window(self, frame):
        """Updates the debug window based on cfg (i.e. whether it should be shown) and video frame."""

        # TODO: support multiple incoming streams
        if self.cfg.debug_window:
            img = frame.to_ndarray(format="bgr24")
            cv2.namedWindow(self._winname, cv2.WINDOW_NORMAL)
            self._winshown = True
            cv2.imshow(self._winname, img)
            cv2.waitKey(1)
        elif self._winshown:
            cv2.destroyWindow(self._winname)
            self._winshown = False

    def step(self, delta):
        if self._frame_pub.get_subscription_count() < 1:
            return

        frames = self.rtc_manager.get_frames()
        # self.log.info(str(frames))

        for uuid, frame in frames.items():
            self._update_window(frame)

            if self.cfg.use_compression or self.cfg.downscale_wh > 0:
                cv2img = frame.to_ndarray(format="bgr24")

                if self.cfg.downscale_wh > 0:
                    cv2img = letterbox(
                        cv2img, (self.cfg.downscale_wh, self.cfg.downscale_wh)
                    )[0]

                if self.cfg.use_compression:
                    _, cv2img = cv2.imencode(
                        ".jpg",
                        cv2img,
                        (cv2.IMWRITE_JPEG_QUALITY, self.cfg.compression_level),
                    )
                    img = CompressedImage()
                    img.format = "jpeg"
                else:
                    img = Image(
                        height=cv2img.shape[0],
                        width=cv2img.shape[1],
                        step=cv2img.size // cv2img.shape[0],
                        encoding="bgr8",
                        is_bigendian=False,
                    )
                img.data.frombytes(memoryview(cv2img))

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


def main(args=None):
    if __name__ == "__main__" and args is None:
        args = sys.argv
    try:
        rclpy.init(args=args)

        node = Node(NODE_NAME)
        cfg = RTCRecvConfig()

        async def loop():
            manager = RTCManager(loop=asyncio.get_running_loop(), log=node.get_logger())
            RTCReceiver(node=node, ini_cfg=cfg, rtc_manager=manager)
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
