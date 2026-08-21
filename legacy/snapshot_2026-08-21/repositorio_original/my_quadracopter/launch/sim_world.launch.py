from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    world_path = os.path.join(
        os.getenv('HOME'),
        'ros', 'Gazebo_ws', 'src', 'my_quadracopter', 'worlds', 'empty.world'
    )

    return LaunchDescription([
        # Lanzar Gazebo con el mundo
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                world_path,
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        # Lanzar el nodo que inserta el robot
        Node(
            package='my_quadracopter',
            executable='sim_quadcopter',
            name='spawn_quadcopter_node',
            output='screen'
        )
    ])