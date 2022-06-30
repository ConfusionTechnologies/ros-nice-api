# See: https://aiortc.readthedocs.io/en/latest/api.html#aiortc.RTCIceServer
# Unified msg type for TURN or STUN servers
# Required for TURN only
# TODO: implement
string credential
string credential_type "password"
string username
# Required for both STUN & TURN
# yes official spec is "urls" but they are overengineering 
string url