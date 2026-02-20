from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    pkg_sim = FindPackageShare("ntu_robotsim_octomap")

    # --- Gazebo Maze ---
    maze_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_sim, "launch", "maze.launch.py"])
        )
    )

    # --- Spawn Robot ---
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_sim, "launch", "single_robot_sim.launch.py"])
        )
    )

    # --- Odom -> TF ---
    odom_to_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("odom_to_tf_ros2"),
                "launch",
                "odom_to_tf.launch.py"
            ])
        )
    )

    # --- OctoMap (your filtered launch) ---
    # If you named it differently, change the filename here.
    octomap_filtered_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_sim, "launch", "octomap_filtered.launch.py"])
        )
    )

    # --- OPTIONAL static TF: map -> odom (only if needed by your Nav2 config) ---
    # If your TF already has map->odom from localization, you can remove this node.
    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen"
    )

    # --- Nav2 Bringup ---
    # Make sure this file exists:
    # /home/ntu-user/ros2_ws/src/ntu_robotsim_octomap/ntu_robotsim_octomap/config/nav2_octomap_params.yaml
    nav2_params = "/home/ntu-user/ros2_ws/src/ntu_robotsim_octomap/ntu_robotsim_octomap/config/nav2_octomap_params.yaml"

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "navigation_launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": nav2_params
        }.items()
    )

    return LaunchDescription([
        maze_launch,
        robot_launch,
        odom_to_tf_launch,
        octomap_filtered_launch,
        map_to_odom_tf,     # remove if not needed
        nav2_launch
    ])
