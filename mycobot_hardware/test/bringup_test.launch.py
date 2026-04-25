import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution

def generate_launch_description():
    # POINT THIS AT YOUR ACTUAL URDF/XACRO PACKAGE AND FILE:
    your_robot_pkg = "your_robot_description"
    your_robot_xacro = "robot.urdf.xacro"

    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare(your_robot_pkg), "urdf", your_robot_xacro
        ])
    ])

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("mycobot_hardware"), "config", "controllers.yaml"
    ])

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description},
                controllers_yaml,
            ],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
    ])
