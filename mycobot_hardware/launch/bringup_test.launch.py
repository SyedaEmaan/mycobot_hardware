"""
L2 bringup test against the mycobot URDF, with ros2_control configured for
ONE joint (link1_to_link2) — the base rotation joint. Wire your single
Laifual drive as slave 1 on the EtherCAT bus.

USAGE:
    ros2 launch mycobot_hardware bringup_test.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    pkg_share = get_package_share_directory("mycobot_hardware")
    urdf_file = os.path.join(pkg_share, "urdf", "mycobot_280_l2_test.urdf")
    with open(urdf_file, "r") as f:
        robot_description_content = f.read()

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("mycobot_hardware"),
        "config",
        "controllers.yaml",
    ])

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                controllers_yaml,
            ],
            remappings=[
                ("~/robot_description", "/robot_description"),
            ],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description_content}],
            output="screen",
        ),
    ])
