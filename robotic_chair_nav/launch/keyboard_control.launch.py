from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    world_file = os.path.join(
        FindPackageShare("simple_world").find("simple_world"),
        "worlds",
        "robotic_chair_worlds.sdf"
    )

    hospital_map = os.path.join(
        FindPackageShare("robotic_chair_nav").find("robotic_chair_nav"),
        "maps",
        "hospital_map.yaml"
    )

    desc_share = FindPackageShare("robotic_chair_description").find(
        "robotic_chair_description"
    )
    urdf_file = os.path.join(desc_share, "urdf", "chair.urdf")

    with open(urdf_file, "r") as f:
        robot_description = f.read()

    nav2_bringup_dir = FindPackageShare("nav2_bringup").find("nav2_bringup")
    nav2_bringup_launch = os.path.join(
        nav2_bringup_dir, "launch", "bringup_launch.py"
    )

    nav2_params_file = os.path.join(
        FindPackageShare("robotic_chair_nav").find("robotic_chair_nav"),
        "config",
        "nav2_params.yaml"
    )

    # Use Nav2's default RViz config (already set up with map, TF, robot, etc.)
    rviz_config_file = os.path.join(
        nav2_bringup_dir,
        "rviz",
        "nav2_default_view.rviz"
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                "gazebo",
                "--verbose",
                world_file,
                "-s", "libgazebo_ros_factory.so"
            ],
            output="screen"
        ),

        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "robotic_chair",
                "-file", urdf_file
            ],
            output="screen"
        ),

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
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch),
            launch_arguments={
                "use_sim_time": "True",
                "map": hospital_map,
                "params_file": nav2_params_file,
            }.items(),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file],
        ),
    ])