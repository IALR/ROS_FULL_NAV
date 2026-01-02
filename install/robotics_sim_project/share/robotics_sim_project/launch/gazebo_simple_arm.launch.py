import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'robotics_sim_project'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Process Xacro to URDF
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare(pkg_name),
            'urdf',
            'simple_arm.urdf.xacro'
        ])
    ])
    
    # Path to world file
    world_file = os.path.join(pkg_share, 'worlds', 'simple_test_world.world')
    
    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )
    
    # Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'simple_arm_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
    ])