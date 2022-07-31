from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch_ros.actions import Node

# Feels like crappier version of click
PACKAGE_NAME = "aiortc_ros"


# See documentation: https://github.com/ros2/launch/blob/foxy/launch/doc/source/architecture.rst


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    namespace_arg = DeclareLaunchArgument(
        "namespace", description="Set the namespace of the nodes", default_value="rtc"
    )

    recv_node = Node(
        package=PACKAGE_NAME,
        namespace=namespace,
        executable="recv",
        name="rtc_receiver",
        respawn=True,
    )

    rosbridge_cfg = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("rosbridge_server"))
                / "launch"
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
            ssl="false",
            certfile="/cert/server.crt",
            keyfile="/cert/server.key",
        ).items(),
    )

    launch_desc = LaunchDescription(
        [namespace_arg, recv_node, TimerAction(period=3.0, actions=[rosbridge_cfg])]
    )

    return launch_desc

