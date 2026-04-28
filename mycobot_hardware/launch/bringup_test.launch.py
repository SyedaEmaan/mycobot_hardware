"""
L2 bringup test against the mycobot URDF, with ros2_control configured for
ONE joint (link1_to_link2) — the base rotation joint.
USAGE:
    ros2 launch mycobot_hardware bringup_test.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    pkg_share = get_package_share_directory("mycobot_hardware")
    urdf_file = os.path.join(pkg_share, "urdf", "mycobot_280_l2_test.urdf")

    with open(urdf_file, "r") as f:
        robot_description_content = f.read()

    controllers_yaml = os.path.join(pkg_share, "config", "controllers.yaml")

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controllers_yaml,
            {"robot_description": robot_description_content},
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
        output="screen",
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        )],
    )

    forward_position_controller_spawner = TimerAction(
        period=4.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        )],
    )

    return LaunchDescription([
        ros2_control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner,
    ])