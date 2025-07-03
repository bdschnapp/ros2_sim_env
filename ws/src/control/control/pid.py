import rclpy
from rclpy.node import Node
import numpy as np
from autoware_planning_msgs.msg import Trajectory
from autoware_control_msgs.msg import Control
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion


class PidController(Node):
    """
    A simple Stanley-based controller for trajectory tracking.
    """

    def __init__(self):
        super().__init__('pid_controller')

        # Declare parameters for the controller
        self.declare_parameter('kp', 1.0)  # Proportional gain for cross-track error
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('wheelbase', 2.85)

        # Get parameters
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.L = self.get_parameter('wheelbase').get_parameter_value().double_value

        # Subscribers
        self.create_subscription(Trajectory, '/planning/scenario_planning/trajectory', self.trajectory_callback, 10)
        self.create_subscription(PoseStamped, '/current_pose', self.pose_callback, 10)

        # Publisher
        self.control_pub = self.create_publisher(Control, '/control/command/control_cmd', 10)

        # Class variables
        self.current_trajectory = None
        self.current_pose = None

        # Create a timer to run the control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        self.get_logger().info('Stanley Controller has been started.')

    def trajectory_callback(self, msg):
        self.current_trajectory = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if self.current_pose is None or self.current_trajectory is None or len(self.current_trajectory.points) < 2:
            return

        # Get current state (rear axle)
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        orientation_q = self.current_pose.pose.orientation
        _, _, current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Get target velocity from the trajectory
        # For simplicity, we get it from the first point, but a more robust implementation
        # would get it from the target point itself.
        target_velocity = self.current_trajectory.points[0].longitudinal_velocity_mps

        # --- FIX: Calculate front axle position for Stanley Method ---
        front_axle_x = current_x + self.L * np.cos(current_yaw)
        front_axle_y = current_y + self.L * np.sin(current_yaw)

        # Find the target point and calculate cross-track error using the front axle position
        target_idx, cross_track_error = self.find_target_and_cte(front_axle_x, front_axle_y)

        if target_idx is None:
            self.get_logger().warn('Could not find a target waypoint.')
            return

        # --- Steering Control using Stanley Method ---
        # 1. Calculate Heading Error
        target_yaw = self.get_yaw_from_trajectory(target_idx)
        heading_error = self.normalize_angle(target_yaw - current_yaw)

        # 2. Calculate Steering Angle from Cross-Track Error
        # Use a small constant to avoid division by zero
        cte_steering_angle = np.arctan2(self.kp * cross_track_error, target_velocity + 0.1)

        # 3. Combine for final steering angle
        steering_angle = heading_error + cte_steering_angle

        # Create and publish the command
        control_cmd = Control()
        control_cmd.longitudinal.velocity = target_velocity
        control_cmd.lateral.steering_tire_angle = np.clip(steering_angle, -0.6, 0.6)  # Clip to reasonable limits

        self.control_pub.publish(control_cmd)

    def find_target_and_cte(self, front_axle_x, front_axle_y):
        """Finds the closest path segment and calculates the cross-track error."""
        waypoints = np.array([[p.pose.position.x, p.pose.position.y] for p in self.current_trajectory.points])

        # Calculate distance from front axle to all waypoints
        distances = np.linalg.norm(waypoints - np.array([front_axle_x, front_axle_y]), axis=1)

        # Find the index of the closest waypoint
        closest_idx = np.argmin(distances)

        # Ensure the chosen path segment is forward of the closest point to prevent going backwards
        path_segment_start_idx = closest_idx
        path_segment_end_idx = path_segment_start_idx + 1
        if path_segment_end_idx >= len(waypoints):
            path_segment_end_idx = len(waypoints) - 1
            path_segment_start_idx = path_segment_end_idx - 1

        p1 = waypoints[path_segment_start_idx]
        p2 = waypoints[path_segment_end_idx]

        # Calculate cross-track error (CTE)
        # Vector from p1 to vehicle's front axle
        vehicle_vector = np.array([front_axle_x, front_axle_y]) - p1
        # Vector from p1 to p2
        path_vector = p2 - p1

        # The sign of the CTE determines if we are left or right of the path
        # We can use the 2D cross product for this
        path_angle = np.arctan2(path_vector[1], path_vector[0])
        vehicle_angle = np.arctan2(vehicle_vector[1], vehicle_vector[0])
        error_angle = self.normalize_angle(path_angle - vehicle_angle)

        cte = distances[closest_idx] * np.sign(error_angle)

        # The target for heading error is the yaw of the end of our path segment
        target_idx = path_segment_end_idx

        return target_idx, cte

    def get_yaw_from_trajectory(self, idx):
        """Extracts yaw from a trajectory point's quaternion."""
        q = self.current_trajectory.points[idx].pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def normalize_angle(self, angle):
        """Normalize an angle to the range [-pi, pi]."""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    pid_controller = PidController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
