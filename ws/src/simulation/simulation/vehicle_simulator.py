import rclpy
from rclpy.node import Node
import numpy as np
from autoware_control_msgs.msg import Control
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class VehicleSimulator(Node):
    """
    Simulates vehicle movement based on Ackermann control commands.

    This node replaces the real vehicle and the SLAM system for testing purposes.
    It subscribes to control commands and publishes the resulting pose and
    transform, creating a closed-loop simulation environment.
    """

    def __init__(self):
        super().__init__('vehicle_simulator')

        # Declare parameters
        self.declare_parameter('wheelbase', 2.85)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        # Get parameters
        self.L = self.get_parameter('wheelbase').get_parameter_value().double_value

        # Vehicle state [x, y, yaw]
        self.x = self.get_parameter('initial_x').get_parameter_value().double_value
        self.y = self.get_parameter('initial_y').get_parameter_value().double_value
        self.yaw = self.get_parameter('initial_yaw').get_parameter_value().double_value

        # Store latest command
        self.last_cmd = Control()

        # Subscriber
        self.create_subscription(
            Control,
            '/control/command/control_cmd',
            self.cmd_callback,
            10)

        # Publisher for the vehicle's pose
        self.pose_pub = self.create_publisher(PoseStamped, '/current_pose', 10)

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Simulation loop timer
        self.timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(self.timer_period, self.update_simulation)

        self.get_logger().info('Vehicle Simulator has been started.')

    def cmd_callback(self, msg):
        """Store the latest control command."""
        self.last_cmd = msg

    def update_simulation(self):
        """Update the vehicle's state based on the kinematic model."""
        dt = self.timer_period
        v = self.last_cmd.longitudinal.velocity
        delta = self.last_cmd.lateral.steering_tire_angle

        # Kinematic bicycle model
        self.x += v * np.cos(self.yaw) * dt
        self.y += v * np.sin(self.yaw) * dt
        self.yaw += (v / self.L) * np.tan(delta) * dt
        self.yaw = self.normalize_angle(self.yaw)

        # Get current time
        now = self.get_clock().now().to_msg()

        # --- Publish the PoseStamped message ---
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y

        q = quaternion_from_euler(0, 0, self.yaw)
        pose_msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.pose_pub.publish(pose_msg)

        # --- Broadcast the transform ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = pose_msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def normalize_angle(self, angle):
        """Normalize an angle to the range [-pi, pi]."""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    vehicle_simulator = VehicleSimulator()
    rclpy.spin(vehicle_simulator)
    vehicle_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()