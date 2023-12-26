from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='quad_anim',
            executable='anim_window',
        ),
        Node(
            package='quad_sim',
            executable='solve_physics',
        )
    ])