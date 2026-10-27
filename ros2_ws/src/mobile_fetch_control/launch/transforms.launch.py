""" Publish wheel transform using simulated joint states """

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_share = Path(
        get_package_share_directory("mobile_fetch_control")
    )

    robot_description = (
        package_share / "urdf" / "mobile_base.urdf"
    ).read_text()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": True,
                "publish_frequency": 50.0,
            }],
        ),
    ])