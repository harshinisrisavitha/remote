from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("aruco_markers"),
        "config",
        "aruco_params.yaml"
    ])

    return LaunchDescription([
        Node(
            package="aruco_markers",
            executable="aruco_markers",
            name="aruco_markers",
            parameters=[config_file],
            output="screen"
        )
    ])