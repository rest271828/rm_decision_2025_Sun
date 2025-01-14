from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    fsm_solution_node = Node(
        package="fsm_solution",
        name="fsm_solution_node",
        executable="fsm_solution_node",
        namespace="",
        output="screen",
    )

    return LaunchDescription([fsm_solution_node])
