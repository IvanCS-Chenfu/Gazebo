from launch import LaunchDescription
from launch.actions import ExecuteProcess, Node
import os

def generate_launch_description():
    world_path = os.path.join(
        os.getenv('HOME'),
        'ros', 'Gazebo_ws', 'src', 'my_box_robot', 'worlds', 'empty.world'
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
            package='my_box_robot',
            executable='sim_box_bot',
            name='spawn_bot_node',
            output='screen'
        )
    ])
