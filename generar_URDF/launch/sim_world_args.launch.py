from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    
    pkg = get_package_share_directory('urdf_gazebo')
    xacro_file = os.path.join(pkg, 'urdf', 'macro_joints.xacro')

    robot_description = Command([
                                    'xacro ', xacro_file,
                                    ' px1:=', '2',
                                    ' pz2:=', '2',
                                    ' n_90_grad2:=', '0.5'
                                ])
    
    
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
        
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                # opcional si tu URDF usa "world" en gazebo:
                # 'frame_prefix': 'model/'
            }]
        ),

        # Lanzar el nodo que inserta el robot
        Node(
            package='urdf_gazebo',
            executable='sim_URDF_args',
            name='spawn_bot_node',
            output='screen'
        )
    ])