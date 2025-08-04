from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    world_path = os.path.join(
        os.getenv('HOME'),
        'Gazebo', 'src', 'fuerza_gazebo', 'worlds', 'empty.world'
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
            package='fuerza_gazebo',
            executable='sim_URDF',
            name='spawn_bot_node',
            output='screen'
        )
    ])