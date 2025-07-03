import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # --- Declare launch arguments ---
    # Path to the .pcd map file
    default_pcd_map_path = os.path.join(
        get_package_share_directory('simulation'),
        'maps',
        'map.pcd'
    )
    pcd_map_arg = DeclareLaunchArgument(
        'pcd_map',
        default_value=default_pcd_map_path,
        description='Full path to .pcd map file to load'
    )

    # Path to the RViz configuration file
    rviz_config_dir = os.path.join(
        get_package_share_directory('simulation'),
        'rviz',
        'simulation.rviz')

    return LaunchDescription([
        pcd_map_arg,

        # --- PCD Map Publisher ---
        Node(
            package='simulation',
            executable='pcd_map_publisher',
            name='pcd_map_publisher',
            parameters=[{'pcd_file_path': LaunchConfiguration('pcd_map')}]
        ),
        # --- Planning ---
        Node(
            package='planning',
            executable='simple_trajectory_publisher',
            name='simple_trajectory_publisher',
            output='screen'
        ),

        # --- Control ---
        Node(
            package='control',
            executable='pid_controller',
            name='pid_controller',
            output='screen',
            parameters=[
                {'kp': 1.5},
                {'ki': 0.1},
                {'kd': 0.05},
                {'lookahead_distance': 3.0},
                {'wheelbase': 0.65}
            ]
        ),

        # --- Simulation ---
        Node(
            package='simulation',
            executable='vehicle_simulator',
            name='vehicle_simulator',
            output='screen',
            parameters=[
                {'wheelbase': 0.65}
            ]
        ),

        # --- Visualization ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen',
            # --- FIX for WSL2 Unresponsive GUI ---
            additional_env={'MESA_LOADER_DRIVER_OVERRIDE': 'llvmpipe'}
        ),
    ])
