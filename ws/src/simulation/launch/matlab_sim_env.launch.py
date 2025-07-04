from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # --- Circular Path ---
        Node(
            package='planning',
            executable='path_publisher',
            name='path_publisher',
            output='screen',
            parameters=[
                {'radius': 50.0,
                 'center_x': 0.0,
                 'center_y': 50.0,
                 'num_points': 500}
            ]
        ),

        # --- Trajectory ---
        Node(
            package='planning',
            executable='trajectory_publisher',
            name='trajectory_publisher',
            output='screen',
            parameters=[
                {'num_trajectory_points': 40,
                 'target_velocity': 1.5,
                 'publish_rate': 0.1}
            ]
        ),

        # --- Tractor Trailer Simulation ---
        Node(
            package='simulation',
            executable='vehicle_trailer_simulator',
            name='vehicle_trailer_simulator',
            output='screen',
            parameters=[
                {'tractor_wheelbase': 3.95,
                 'trailer_length': 18.2,
                 'hitch_x_offset': 0.0,
                 'initial_x': 0.0,
                 'initial_y': 0.0,
                 'initial_yaw': 0.0}
            ]
        ),
    ])
