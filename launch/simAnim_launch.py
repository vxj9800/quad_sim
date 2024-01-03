from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='quad_anim',
            executable='anim_window',
            parameters=[{
                'use_sim_time': True
            }],
        ),
        Node(
            package='quad_sim',
            executable='solve_physics',
            parameters=[{
                'use_sim_time': True
            }]
        )
    ])