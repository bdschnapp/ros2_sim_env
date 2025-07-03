import rclpy
from rclpy.node import Node
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from tf_transformations import quaternion_from_euler
import numpy as np


class SimpleTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('simple_trajectory_publisher')

        self.traj_publisher_ = self.create_publisher(Trajectory, '/planning/scenario_planning/trajectory', 10)
        self.path_publisher_ = self.create_publisher(Path, '/planning/scenario_planning/path_for_viz', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Simple Trajectory Publisher has been started.')

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        autoware_traj = Trajectory()
        autoware_traj.header.frame_id = 'map'
        autoware_traj.header.stamp = now

        rviz_path = Path()
        rviz_path.header.frame_id = 'map'
        rviz_path.header.stamp = now

        # --- Define Parameters for the Elliptical Path ---
        # Sized to fit the general dimensions of the map
        center_x = 7
        center_y = -1.25
        semi_axis_a = 5  # Half-width
        semi_axis_b = 1.5  # Half-height
        num_points = 150

        # --- Define Velocity Profile Parameters ---
        max_speed = 1.5  # m/s on the straightest parts
        min_speed = 0.1  # m/s on the sharpest turns
        max_lat_accel = 1.5  # m/s^2, max comfortable side-to-side acceleration

        path_points = []
        for i in range(num_points + 1):
            angle = (i / num_points) * 2 * np.pi

            # --- Position and Orientation ---
            x = center_x + semi_axis_a * np.cos(angle)
            y = center_y + semi_axis_b * np.sin(angle)

            dx = -semi_axis_a * np.sin(angle)
            dy = semi_axis_b * np.cos(angle)
            yaw = np.arctan2(dy, dx)

            # --- Curvature and Velocity Calculation ---
            # Analytical curvature for an ellipse
            a_sq_sin_sq = (semi_axis_a * np.sin(angle)) ** 2
            b_sq_cos_sq = (semi_axis_b * np.cos(angle)) ** 2
            numerator = semi_axis_a * semi_axis_b
            denominator = (a_sq_sin_sq + b_sq_cos_sq) ** 1.5
            curvature = numerator / (denominator + 1e-9)  # Add epsilon to avoid division by zero

            # Calculate speed limit based on curvature and max lateral acceleration
            speed_limit_from_curvature = np.sqrt(max_lat_accel / (abs(curvature) + 1e-9))

            # Clamp the velocity between the defined min and max speeds
            velocity = np.clip(speed_limit_from_curvature, min_speed, max_speed)

            path_points.append((x, y, yaw, velocity))

        for x, y, yaw, vel in path_points:
            point = TrajectoryPoint()
            q = quaternion_from_euler(0, 0, yaw)
            point.pose.position = Point(x=x, y=y, z=0.0)
            point.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            point.longitudinal_velocity_mps = vel  # Use the calculated velocity
            autoware_traj.points.append(point)

            pose_stamped = PoseStamped()
            pose_stamped.header = autoware_traj.header
            pose_stamped.pose = point.pose
            rviz_path.poses.append(pose_stamped)

        self.traj_publisher_.publish(autoware_traj)
        self.path_publisher_.publish(rviz_path)

def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = SimpleTrajectoryPublisher()
    rclpy.spin(trajectory_publisher)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()