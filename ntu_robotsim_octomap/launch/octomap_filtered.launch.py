from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server2',
            executable='octomap_server',
            name='octomap_server',
            output='screen',
            parameters=[{
                # Core
                'frame_id': 'odom',
                'resolution': 0.05,

                # Use the robot base frame for TF transforms
                'base_frame_id': 'base_link',

                # Ground filtering (Task 1)
                'filter_ground': True,
                'ground_filter_distance': 0.04,
                'ground_filter_angle': 0.15,
                'ground_filter_plane_distance': 0.07,

                # Sensor
                'sensor_model/max_range': 6.0,
            }],
            remappings=[
                ('cloud_in', '/rgbd_camera/points'),
            ],
        )
    ])
