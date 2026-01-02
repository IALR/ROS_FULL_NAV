from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

# Path to your saved map
hospital_map = "/home/ilyas/ros2_ws/src/maps/hospital_map.yaml"


def generate_launch_description():
    # =====================
    # World
    # =====================
    world_file = os.path.join(
        FindPackageShare("simple_world").find("simple_world"),
        "worlds",
        "robotic_chair_worlds.sdf"
    )

    # =====================
    # URDF
    # =====================
    desc_share = FindPackageShare("robotic_chair_description").find(
        "robotic_chair_description"
    )
    urdf_file = os.path.join(desc_share, "urdf", "chair.urdf")

    with open(urdf_file, "r") as f:
        robot_description = f.read()

    # =====================
    # Nav2 bringup
    # =====================
    nav2_bringup_dir = FindPackageShare("nav2_bringup").find("nav2_bringup")
    nav2_bringup_launch = os.path.join(
        nav2_bringup_dir, "launch", "bringup_launch.py"
    )

    nav2_params_file = os.path.join(
        FindPackageShare("robotic_chair_nav").find("robotic_chair_nav"),
        "config",
        "nav2_params.yaml"
    )

    return LaunchDescription([
        # =====================
        # Gazebo
        # =====================
        ExecuteProcess(
            cmd=[
                "gazebo",
                "--verbose",
                world_file,
                "-s", "libgazebo_ros_factory.so"
            ],
            output="screen"
        ),

        # =====================
        # Spawn robot
        # =====================
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "robotic_chair",
                "-file", urdf_file
            ],
            output="screen"
        ),

        # =====================
        # Robot State Publisher
        # =====================
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {"use_sim_time": True},
                {"robot_description": robot_description},
            ],
        ),

        # =====================
        # Keyboard Control (optional manual driving)
        # =====================
        Node(
            package="robotic_chair_nav",
            executable="keyboard_control",
            name="keyboard_control",
            output="screen",
            parameters=[
                {"linear_speed": 0.3},
                {"angular_speed": 0.5},
                {"stop_distance": 0.1},
                {"scan_topic": "/scan"},
                {"cmd_vel_topic": "/cmd_vel"},
                {"front_half_angle_deg": 20.0},
            ],
        ),

        # =====================
        # Nav2 Bringup (map_server, amcl, costmaps, planner, controller, bt_navigator)
        # =====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch),
            launch_arguments={
                "use_sim_time": "True",
                "map": hospital_map,
                "params_file": nav2_params_file,
            }.items(),
        ),

        # =====================
        # RViz2
        # =====================
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            # You can optionally pass -d /path/to/nav2_config.rviz here
        ),
    ])