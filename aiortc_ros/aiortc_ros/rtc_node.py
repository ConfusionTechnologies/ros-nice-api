from __future__ import annotations

import traceback
from dataclasses import dataclass, field
from typing import Generic, TypeVar

from aiortc_ros_msgs.msg import IceCandidate
from aiortc_ros_msgs.srv import Handshake
from nicepynode import Job
from nicepynode.utils import declare_parameters_from_dataclass
from rclpy.node import Node

from aiortc_ros.cfg import RTCConfig
from aiortc_ros.rtc_manager import RTCManager

CT = TypeVar("CT", bound=RTCConfig)
"""Generic for class that extends RTCConfig."""


@dataclass
class RTCNode(Job[CT], Generic[CT]):
    """Common implementation of ICE."""

    ini_cfg: CT = field(default_factory=RTCConfig)
    rtc_manager: RTCManager = None

    def attach_params(self, node: Node, cfg: CT):
        super(RTCNode, self).attach_params(node, cfg)

        declare_parameters_from_dataclass(node, cfg)

    def on_params_change(self, node: Node, changes: dict):
        return True

    def attach_behaviour(self, node: Node, cfg: CT):
        super(RTCNode, self).attach_behaviour(node, cfg)

        self._conn_srv = node.create_service(
            Handshake, cfg.connect_service, self._on_connection
        )
        self._ice_sub = node.create_subscription(
            IceCandidate, cfg.ice_candidate_topic, self._on_ice_candidate, 10
        )

    def detach_behaviour(self, node: Node):
        super(RTCNode, self).detach_behaviour(node)

        node.destroy_service(self._conn_srv)
        node.destroy_subscription(self._ice_sub)

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
