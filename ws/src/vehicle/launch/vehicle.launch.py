import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get paths to config files
    default_pcd_map_path = os.path.join(
        get_package_share_directory('simulation'), 'maps', 'map.pcd'
    )
    rviz_config_dir = os.path.join(
        get_package_share_directory('simulation'), 'rviz', 'simulation.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'pcd_map', default_value=default_pcd_map_path,
            description='Full path to the .pcd map file'
        ),
        DeclareLaunchArgument(
            'rviz_config', default_value=rviz_config_dir,
            description='Full path to the RViz config file'
        ),

        # --- LOCALIZATION ---
        Node(
            package='localization',
            executable='lidar_localization',
            name='lidar_localization',
            output='screen',
            parameters=[{'map_path': LaunchConfiguration('pcd_map')},
                        {'wheelbase': 0.65},
                        {'is_ground_truth': True}]
        ),

        # --- STATIC TRANSFORM ---
        # This is CRITICAL. It tells the system where the lidar sensor is
        # relative to the robot's base.

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_lidar_publisher',
        #     # Args: x, y, z, yaw, pitch, roll, parent_frame, child_frame
        #     arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'base_link', 'lidar_link']
        # ),

        # --- AUTONOMY STACK ---
        Node(
            package='planning',
            executable='simple_trajectory_publisher',
            name='simple_trajectory_publisher',
            output='screen'
        ),
        Node(
            package='control',
            executable='pid_controller',
            name='pid_controller',
            output='screen'
        ),

        # Node(
        #     package='vehicle',
        #     executable='canbridge',
        #     name='canbridge',
        #     output='screen'
        # ),

        # --- VISUALIZATION ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
            additional_env={'MESA_LOADER_DRIVER_OVERRIDE': 'llvmpipe'}
        ),
    ])