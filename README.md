# ros-nice-api

Socket.IO, WebRTC & other API packages

This is implemented using:

- [`aiortc`](https://aiortc.readthedocs.io/en/latest/)

Initial code was self-plagiarized from <https://github.com/Interpause/nicepipe/commit/fd693a6638f8f08549b6a13a93d71b6bea4d6810> by [@Interpause](https://github.com/Interpause).

It might be reimplemented in the future using <https://github.com/RobotWebTools/rclnodejs> to utilize the NodeJS ecosystem, the native ecosystem of many APIs.

## Notes

- rqt image visualizer
  - the image visualizer isn't compatible with all QoS Profiles! the default works though
    - See: <https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html>
  - It only supports `sensor_msgs/Image` not `sensor_msgs/CompressedImage`
    - Alternatively, use `image_transport` to transparently treat them all the same (but it doesn't have Python bindings...)
- <https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html#field-types> is useful for writing interfaces
- <chrome://webrtc-internals/> **is your best friend!**

## Issues

- <https://github.com/RobotWebTools/rosbridge_suite/issues/744>
  - Ocassionally websocket server will seem to completely freeze
  - May ocassionally spam inconsequential errors when client disconnects improperly
