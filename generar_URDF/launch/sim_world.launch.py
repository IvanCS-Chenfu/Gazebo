from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # Lanzar Gazebo con el mundo
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        # Lanzar el nodo que inserta el robot
        Node(
            package='urdf_gazebo',
            executable='sim_URDF',
            name='spawn_bot_node',
            output='screen'
        )
    ])