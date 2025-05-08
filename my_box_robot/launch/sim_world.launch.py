from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    world_path = os.path.join(
        os.getenv('HOME'),
        'ros', 'Gazebo_ws', 'src', 'my_box_robot', 'worlds', 'empty.world'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                world_path,
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        )
    ])
