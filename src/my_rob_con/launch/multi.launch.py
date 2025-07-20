from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    urdf_path1 = os.path.join(
        FindPackageShare('my_rob_con').find('my_rob_con'),
        'task4', 'urdf', 'robot1.urdf'
    )

    urdf_path2 = os.path.join(
        FindPackageShare('my_rob_con').find('my_rob_con'),
        'task4', 'urdf', 'robot2.urdf'
    )

    return LaunchDescription([
        # Launch Gazebo with the required plugins
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    FindPackageShare('gazebo_ros').find('gazebo_ros'),
                    'launch', 'gazebo.launch.py'
                )
            ]),
            launch_arguments={
                'extra_gazebo_args': '-s libgazebo_ros_factory.so -s libgazebo_ros_init.so'
            }.items()
        ),

        # Spawn robot1
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'robot1',
                '-file', urdf_path1,
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        ),

        # Spawn robot2
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'robot2',
                '-file', urdf_path2,
                '-x', '2', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        )
    ])
