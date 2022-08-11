from __future__ import annotations

import asyncio
import json
import threading
from dataclasses import dataclass
from typing import Any
from uuid import uuid4

from aiortc import (
    RTCConfiguration,
    RTCDataChannel,
    RTCIceCandidate,
    RTCIceServer,
    RTCPeerConnection,
    RTCSessionDescription,
)
from aiortc.rtcrtpreceiver import RemoteStreamTrack
from aiortc_ros_msgs.msg import SDP, IceCandidate
from av import VideoFrame
from nicepynode.aioutils import wait_coro
from nicepynode.utils import Symbol

from aiortc_ros.utils import have_internet

# Currently on version 1.3.2 of aiortc
# See https://github.com/aiortc/aiortc/blob/1.3.2/src/aiortc/mediastreams.py
# or specifically RemoteStreamTrack in https://github.com/aiortc/aiortc/blob/main/src/aiortc/rtcrtpreceiver.py
# PeerConnection emits "track" event when browser sends track
# track from client is of type MediaStreamTrack, most impt is recv() method
# which returns either av.audio.frame.AudioFrame or av.video.frame.VideoFrame
# depending on audio or video track

WAITING = Symbol("WAITING")


@dataclass
class RTCManager:
    loop: asyncio.AbstractEventLoop = None
    log: Any = None

    def __post_init__(self):
        self._conns: dict[str, RTCPeerConnection] = {}
        self._tracks: dict[str, RemoteStreamTrack] = {}
        self._track_lock = threading.Lock()

        # TODO: dont hardcode
        self._ice_servers = [
            RTCIceServer(urls="stun:localhost:3478"),
            # RTCIceServer(urls="stun:openrelay.metered.ca:80"),
            # RTCIceServer(urls="stun:stun.l.google.com:19302"),
            # RTCIceServer(urls="stun:global.stun.twilio.com:3478?transport=udp"),
            # RTCIceServer(
            #     urls="turn:openrelay.metered.ca:80",
            #     username="openrelayproject",
            #     credential="openrelayproject",
            # ),
            # RTCIceServer(
            #     urls="turn:openrelay.metered.ca:443",
            #     username="openrelayproject",
            #     credential="openrelayproject",
            # ),
            # RTCIceServer(
            #     urls="turn:openrelay.metered.ca:443?transport=tcp",
            #     username="openrelayproject",
            #     credential="openrelayproject",
            # ),
        ]

    def _assert_loop(self):
        """Ensure that asyncio loop for synchronous calls has been specified."""
        assert (
            not self.loop is None
        ), f"{type(self).__name__} requires asyncio loop for sync methods!"

    def _create_connection(self, conn_id) -> RTCPeerConnection:
        """Create peer connection & attach event handlers and stuff."""
        # NOTE: aiortc gathers all ICE candidates upfront, connecting to all
        # STUN/TURN servers to do so. Surprisingly, aiortc will fail if it cannot
        # connect to even one server.
        # Below internet connectivity check is not sufficient if STUN/TURN server
        # itself is down.
        # TODO: this is a bug, report it
        ice_servers = self._ice_servers
        if not have_internet():
            self.log.warn(
                "No internet! Removing STUN/TURN temporarily to attempt mDNS connection."
            )
            ice_servers = [RTCIceServer(urls="stun:localhost:3478")]
        pc = RTCPeerConnection(configuration=RTCConfiguration(iceServers=ice_servers))

        # PeerConnection events:
        # - track -> MediaStreamTrack
        # - datachannel -> RTCDataChannel
        # - connectionstatechange -> void
        # - iceconnectionstatechange -> void
        # - icegatheringstatechange -> void
        # - signalingstatechange -> void

        # DataChannel events:
        # - bufferedamountlow -> void
        # - open -> void
        # - close -> void

        # MediaStreamTrack events:
        # - ended -> void

        def on_datachannel(chn: RTCDataChannel):
            chn.send(json.dumps(dict(code=501)))
            chn.close()

        def on_track_end():
            with self._track_lock:
                self._tracks.pop(conn_id, None)
            self.log.info(f"[{conn_id}] Track removed")

        async def on_conn_state():
            if pc.connectionState == "failed":
                self._conns.pop(conn_id, None)
                on_track_end()
                await pc.close()
            self.log.info(f"[{conn_id}] State change: {pc.connectionState}")

        def on_track_received(track: RemoteStreamTrack):
            if track.kind == "video":
                with self._track_lock:
                    self._tracks[conn_id] = track
                self.log.info(f"[{conn_id}] Track added")

            track.add_listener("ended", on_track_end)

        pc.add_listener("datachannel", on_datachannel)
        pc.add_listener("connectionstatechange", on_conn_state)
        pc.add_listener("track", on_track_received)
        return pc

    async def handshake(self, client_sdp: SDP) -> tuple[SDP, str]:
        """Handle RTC connection SDP handshake."""
        conn_id = uuid4().hex
        # intermediate WAITING state to prevent adding ice candidates before connection is ready
        self._conns[conn_id] = WAITING

        self.log.info(f"[{conn_id}] Beginning connection")
        conn = self._create_connection(conn_id)
        client_sdp = RTCSessionDescription(client_sdp.sdp, client_sdp.type)

        await conn.setRemoteDescription(client_sdp)
        await conn.setLocalDescription(await conn.createAnswer())

        server_sdp = SDP()
        server_sdp.sdp = conn.localDescription.sdp
        server_sdp.type = conn.localDescription.type

        self._conns[conn_id] = conn
        self.log.info(f"[{conn_id}] Connection ready")
        return server_sdp, conn_id

    async def add_ice_candidate(self, candidate: IceCandidate) -> None:
        """Handle trickled ICE candidates from client."""
        conn_id = candidate.conn_uuid
        # wait for connection to be ready for additional ice candidates
        while self._conns.get(conn_id, None) == WAITING:
            await asyncio.sleep(0)
        conn = self._conns.get(conn_id, None)
        # ensure connection exists, not some errant client sending invalid candidates
        if conn:
            self.log.debug(f"[{conn_id}] Candidate added: {candidate}")
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

    def get_frames(self) -> dict[str, VideoFrame]:
        """
        Get current frames from all tracks. Frames are pyAV's VideoFrame.
        See https://pyav.org/docs/develop/api/video.html#av.video.frame.VideoFrame\
        and https://pyav.org/docs/develop/api/frame.html#av.frame.Frame
        """
        # see https://github.com/aiortc/aiortc/blob/1.3.2/src/aiortc/rtcrtpreceiver.py
        # no choice but to use internal API track._queue
        frames = {}
        stopped: list[RemoteStreamTrack] = []
        with self._track_lock:
            for conn_id, track in self._tracks.items():
                frame = None
                # if there are frames, get the latest frame & discard the rest
                # discarding might occur when sampling rate < track rate
                while not track._queue.empty():
                    frame = track._queue.get_nowait()
                    # following aiortc's internal code for detecting track end
                    if frame is None:
                        stopped.append(track)

                if frame:  # self._tracks should contain only videos
                    frames[conn_id] = frame

        # tracks must be stopped outside of self._track_lock to avoid
        # calling track removal handler, resulting in deadlock
        for t in stopped:
            t.stop()
        return frames

    def handshake_sync(self, client_sdp: SDP) -> tuple[SDP, str]:
        """Handle RTC connection SDP handshake synchronously."""
        self._assert_loop()
        return wait_coro(self.handshake(client_sdp), self.loop, timeout=20)

    def add_ice_candidate_sync(self, candidate: IceCandidate):
        """Handle adding trickled ICE candidate synchronously"""
        self._assert_loop()
        return wait_coro(self.add_ice_candidate(candidate), self.loop)
