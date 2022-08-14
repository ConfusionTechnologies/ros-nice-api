from __future__ import annotations

from dataclasses import dataclass

from nicepynode import JobCfg


@dataclass
class RTCConfig(JobCfg):
    rate: float = 60.0
    """Sampling/publishing rate for video feed."""
    connect_service: str = "~/connect"
    """Service for exchanging WebRTC SDPs to establish connection."""
    ice_candidate_topic: str = "~/ice_candidates"
    """Topic to send trickled ICE candidates to."""
    ice_info_service: str = "~/get_ice_servers"
    """Get ICE servers available. TODO: implement this."""


@dataclass
class RTCRecvConfig(RTCConfig):
    frames_out_topic: str = "~/frames_out"
    """Topic to publish received video frames to."""
    use_compression: bool = True
    """Only necessary if sending high res (>720p) over rosbridge."""
    compression_level: int = 75
    """Compression level for image frames. Compression only necessary if sending high res over rosbridge."""
    downscale_wh: int = 0
    """When using compression, downscaling the image may be necessary due to CPU usage from image decompression."""
    debug_window: bool = False
    """Displays the video feed for debug purpposes."""


@dataclass
class RTCSendConfig(RTCConfig):
    frames_in_topic: str = "~/frames_in"
    """Topic to receive frames from."""
    list_cam_service: str = "~/list_cams"
    """Service to return list of cameras that can be connected to."""
    expiry_duration: int = 5
    """Duration in seconds before frame_id considered to be expired (track removed due to inactivity)."""
    rate: float = 3

