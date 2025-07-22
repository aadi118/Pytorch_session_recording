#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_dir = os.path.join(
        os.getenv('HOME'), 'ros2_ws', 'src', 'my_rob_con', 'my_rob_con', 'task4', 'urdf')

    robot1_urdf = os.path.join(urdf_dir, 'robot1.urdf')
    robot2_urdf = os.path.join(urdf_dir, 'robot2.urdf')

    with open(robot1_urdf, 'r') as f:
        robot1_desc = f.read()
    with open(robot2_urdf, 'r') as f:
        robot2_desc = f.read()

    # Launch Gazebo properly using gazebo_ros
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    return LaunchDescription([
        # ✅ Fix black screen by forcing software rendering
        SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1'),

        gazebo_launch,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot1_state_pub',
            parameters=[{'robot_description': robot1_desc, 'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot2_state_pub',
            parameters=[{'robot_description': robot2_desc, 'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_pub_gui',
            output='screen'
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'robot1', '-file', robot1_urdf, '-x', '0', '-y', '0', '-z', '0.1'],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'robot2', '-file', robot2_urdf, '-x', '2', '-y', '0', '-z', '0.1'],
            output='screen'
        ),

        Node(
            package='my_rob_con',
            executable='gesture_to_cmd_vel',
            name='gesture_to_cmd_robot1',
            parameters=[{'robot_name': 'robot1'}],
            remappings=[('hand_action', 'left_hand_action')],
            output='screen'
        ),

        Node(
            package='my_rob_con',
            executable='gesture_to_cmd_vel',
            name='gesture_to_cmd_robot2',
            parameters=[{'robot_name': 'robot2'}],
            remappings=[('hand_action', 'right_hand_action')],
            output='screen'
        ),

        Node(
            package='my_rob_con',
            executable='finger_detection',
            name='finger_detection_node',
            output='screen'
        )
    ])