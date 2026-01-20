import launch
import os

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
        LaunchConfiguration,
        IfElseSubstitution,
        PythonExpression,
        PathJoinSubstitution,
        EnvironmentVariable,
        )

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "mrs_odometry_republisher"

    this_pkg_path = get_package_share_directory(pkg_name)
    namespace='mrs_odometry_republisher'

    # #{ node_name

    node_name = LaunchConfiguration('node_name')

    ld.add_action(DeclareLaunchArgument(
        'node_name',
        default_value="odometry_republisher",
        description="The uav name used for namespacing.",
    ))

    # #} end of node_name

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    # #} end of custom_config

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value="",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
            condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
            else_value=custom_config
            )

    # #} end of custom_config

    # #{ new_child_frame

    new_child_frame = LaunchConfiguration('new_child_frame')

    ld.add_action(DeclareLaunchArgument(
        'new_child_frame',
        default_value='child_frame',
        description="",
    ))

    # #} end of new_child_frame

    # #{ new_parent_frame

    new_parent_frame = LaunchConfiguration('new_parent_frame')

    ld.add_action(DeclareLaunchArgument(
        'new_parent_frame',
        default_value='parent_framee',
        description="",
    ))

    # #} end of new_parent_frame

    # #{ tf_parent

    tf_parent = LaunchConfiguration('tf_parent')

    ld.add_action(DeclareLaunchArgument(
        'tf_parent',
        default_value='parent',
        description="When do_transform=true and from_ros=true, then this is the parent of the tf",
    ))

    # #} end of new_child_frame

    # #{ tf_child

    tf_child = LaunchConfiguration('tf_child')

    ld.add_action(DeclareLaunchArgument(
        'tf_child',
        default_value='child',
        description="",
    ))

    # #} end of new_parent_frame

    # #{ topic_in

    topic_in = LaunchConfiguration('topic_in')

    ld.add_action(DeclareLaunchArgument(
        'topic_in',
        default_value='~/odometry_in',
        description="",
    ))

    # #} end of topic_in

    # #{ topic_out

    topic_out = LaunchConfiguration('topic_out')

    ld.add_action(DeclareLaunchArgument(
        'topic_out',
        default_value='~/odometry_out',
        description="",
    ))

    # #} end of topic_in

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', "false"),
        description="Should the node subscribe to sim time?",
    ))

    # #} end of custom_config

    # #{ log_level

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    # #} end of log_level

    ld.add_action(ComposableNodeContainer(

        namespace=uav_name,
        name=[namespace, '_', node_name, '_container'],
        package='rclcpp_components',
        executable='component_container_mt',
        output="screen",
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],

        composable_node_descriptions=[

            ComposableNode(

                package=pkg_name,
                plugin='odometry_republisher::OdometryRepublisher',
                namespace=uav_name,
                name=node_name,

                parameters=[
                    {"uav_name": uav_name},
                    {"new_child_frame": new_child_frame},
                    {"new_parent_frame": new_parent_frame},
                    {"tf_parent": tf_parent},
                    {"tf_child": tf_child},
                    {"use_sim_time": use_sim_time},
                    {"config": this_pkg_path + "/config/default.yaml"},
                    {"custom_config": custom_config},
                ],

                remappings=[
                    # subscribers
                    ("~/odometry_in", topic_in),
                    # publishers
                    ("~/odometry_out", topic_out),
                ],
            )

        ],

    ))

    return ld
