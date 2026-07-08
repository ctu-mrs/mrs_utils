#!/usr/bin/env python3

import launch
import os

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def get_type(sensor: str) -> int:
    type_map = {
        '': 0,
        'unknown': 0,
        'camera': 1,
        'lidar_1d': 2,
        'lidar_2d': 3,
        'lidar_3d': 4,
    }

    return type_map.get(sensor, 0)


def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "mrs_sensor_info"
    namespace = "sensor_info"

    # #{ uav_name
    uav_name = LaunchConfiguration("uav_name")

    ld.add_action(
        DeclareLaunchArgument(
            "uav_name",
            default_value=os.getenv("UAV_NAME", "uav1"),
            description="Name of the UAV",
        )
    )
    # #} end of uav_name

    # #{ name
    name = LaunchConfiguration("name")

    ld.add_action(
        DeclareLaunchArgument(
            "name",
            default_value="",
            description="Name identifier for the sensor",
        )
    )
    # #} end of name

    # #{ topic
    topic = LaunchConfiguration("topic")

    ld.add_action(
        DeclareLaunchArgument(
            "topic",
            default_value="",
            description="Topic to monitor",
        )
    )
    # #} end of topic

    # #{ expected_rate
    expected_rate = LaunchConfiguration("expected_rate")

    ld.add_action(
        DeclareLaunchArgument(
            "expected_rate",
            default_value="",
            description="Expected publishing rate",
        )
    )
    # #} end of expected_rate

    # #{ type
    sensor_type = LaunchConfiguration("type")

    ld.add_action(
        DeclareLaunchArgument(
            "type",
            default_value="",
            description="Sensor type (unknown, camera, lidar_1d, lidar_2d, lidar_3d)",
        )
    )
    # #} end of type

    # #{ standalone
    standalone = LaunchConfiguration("standalone")

    ld.add_action(
        DeclareLaunchArgument(
            "standalone",
            default_value="true",
            description="Whether to start as standalone or load into existing container",
        )
    )
    # #} end of standalone

    # #{ container_name
    container_name = LaunchConfiguration("container_name")

    ld.add_action(
        DeclareLaunchArgument(
            "container_name",
            default_value="",
            description="Name of existing container to load into (if standalone is false)",
        )
    )
    # #} end of container_name

    # #{ use_sim_time
    use_sim_time = LaunchConfiguration("use_sim_time")

    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=os.getenv("USE_SIM_TIME", "false"),
            description="Should the node subscribe to sim time?",
        )
    )
    # #} end of use_sim_time

    # #{ log_level
    ld.add_action(DeclareLaunchArgument(name="log_level", default_value="info"))
    # #} end of log_level

    # #{ default_node
    default_node = ComposableNode(
        package=pkg_name,
        plugin=pkg_name + "::SensorInfo",
        namespace=uav_name,
        name="sensor_info_" + name,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"name": name},
            {"topic": topic},
            {"expected_rate": expected_rate},
            {"type": get_type(sensor_type)},
        ],
        remappings=[
            ("~/sensor_info_out", "sensor_info"),
        ],
    )

    load_into_existing = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[default_node],
        condition=UnlessCondition(standalone),
    )

    ld.add_action(load_into_existing)
    # #} end of default_node

    # #{ standalone container
    standalone_container = ComposableNodeContainer(
        namespace=uav_name,
        name=namespace + "_container",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        composable_node_descriptions=[default_node],
        condition=IfCondition(standalone),
    )

    ld.add_action(standalone_container)
    # #} end of standalone container

    return ld
