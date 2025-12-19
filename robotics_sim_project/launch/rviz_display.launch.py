import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Process Xacro file to generate URDF
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('robotics_sim_project'), 
            'urdf', 
            'simple_arm.urdf.xacro'
        ])
    ])
    
    return LaunchDescription([
        # Publish robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': False
            }]
        ),
        
        # GUI to control joints
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        
        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])