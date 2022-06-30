# See: https://aiortc.readthedocs.io/en/latest/api.html#aiortc.RTCIceCandidate
# And: https://developer.mozilla.org/en-US/docs/Web/API/RTCIceCandidate
# from Handshake.srv: unique id assigned by server to identify connection
string conn_uuid
# rtp=1, rtcp=2
uint8 component
# unique candidate id across transports
string foundation
# either ipv4 or ipv6 (WITHOUT SQUARE BRACKETS!)
string ip
# port is unsigned 16 bit number
uint16 port
# long unsigned integer
uint32 priority
# tcp or udp
string protocol
# host, srflx, prflx or relay
string type
string related_address
uint16 related_port
string sdp_mid
uint16 sdp_mline_index
string tcp_type