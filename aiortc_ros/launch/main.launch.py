import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

PACKAGE_NAME = "aiortc_ros"
# TODO: consider removing default namespace given this launch file is typically
# composed as part of a main launch file.
# The main launch file can use PushRosNamespace instead:
# https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#namespaces
NAMESPACE = "/rtc"

# See documentation: https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst

# rosbridge will crash when a high-rate publisher it is subscribed to
# gets destroyed. I cant seem to find anybody encountering this issue before
# but we are reliably running into this across container rebuilds.
# https://github.com/ros2/rclpy/issues/827 suggests it might be fixed in later
# distros of ROS (as a side effect of an internal API rewrite).
# Unfortunately, the crash doesn't fully crash the node, leading to a zombie
# node that can't be detected & restarted. This is because they seem to be
# running rclpy.spin inside some sort of callback.
# A fix would likely mean modifying rosbridge itself to crash the node when
# that callback crashes. Alternatively, some messing with the publisher QoS
# might fix it.
# NOTE: Seemingly, using the SENSOR_DATA QoS for literally everything is a workaround.


def generate_launch_description():
    use_compression = LaunchConfiguration("use_compression")
    use_compression_arg = DeclareLaunchArgument("use_compression", default_value="True")
    certfile = LaunchConfiguration("certfile")
    certfile_arg = DeclareLaunchArgument("certfile", default_value="/cert/server.crt")
    keyfile = LaunchConfiguration("keyfile")
    keyfile_arg = DeclareLaunchArgument("keyfile", default_value="/cert/server.key")

    ssl_available = IfCondition(
        PythonExpression(
            ["os.path.exists(", certfile, ") and os.path.exists(", keyfile, ")"]
        )
    )

    log = LogInfo(
        PythonExpression(
            [
                "'Cert & key found, SSL will be enabled.' if ",
                ssl_available,
                " else 'Cert or key not found at ",
                certfile,
                " and ",
                keyfile,
                ". SSL disabled.'",
            ]
        )
    )

    recv_node = Node(
        package=PACKAGE_NAME,
        namespace=NAMESPACE,
        executable="recv",
        name="rtc_receiver",
        respawn=True,
        parameters=[{"use_compression": use_compression}],
    )

    send_node = Node(
        package=PACKAGE_NAME,
        namespace=NAMESPACE,
        executable="send",
        name="rtc_sender",
        respawn=True,
        parameters=[{"frames_in_topic": f"{NAMESPACE}/rtc_receiver/frames_out"}],
    )

    rosbridge_cfg = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            str(
                Path(get_package_share_directory(PACKAGE_NAME))
                / "rosbridge_websocket_launch.xml"
            )
        ),
        launch_arguments=dict(
            port="9090",  # is default; but specify again to make sure
            address="0.0.0.0",  # exposes on all network interfaces
            use_compression="true",  # websocket compression
            # the internal API for parsing this throws this through a YAML parser
            # then attempts topics_glob[1:-1].split(",")
            # see: https://github.com/RobotWebTools/rosbridge_suite/issues/727
            # apparently, the default (which is "") exposes everything due to a glitch...
            # see: https://github.com/RobotWebTools/rosbridge_suite/issues/588#issuecomment-898050685
            # topics_glob="\[*\]",
            # services_glob="\[*\]",
            # params_glob="\[*\]",
            ssl=ssl_available,
            certfile=certfile,
            keyfile=keyfile,
        ).items(),
    )

    launch_desc = LaunchDescription(
        [
            use_compression_arg,
            certfile_arg,
            keyfile_arg,
            log,
            recv_node,
            send_node,
            TimerAction(period=3.0, actions=[rosbridge_cfg]),
        ]
    )

    return launch_desc
