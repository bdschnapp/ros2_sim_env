import rclpy
from rclpy.node import Node
import numpy as np
from autoware_control_msgs.msg import Control
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

class VehicleSimulator(Node):
    """
    Simulates a tractor-trailer based on Ackermann control commands.
    Publishes poses, twists, and TFs for both the tractor and the trailer.
    Allows arbitrary hitch point offsets relative to the tractor frame.
    """

    def __init__(self):
        super().__init__('vehicle_simulator')

        # Declare parameters
        self.declare_parameter('tractor_wheelbase', 2.85)
        self.declare_parameter('trailer_length', 1.5)
        self.declare_parameter('hitch_x_offset', 0.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        # Get parameters
        self.L = self.get_parameter('tractor_wheelbase').value
        self.L_t = self.get_parameter('trailer_length').value
        self.hitch_x_off = self.get_parameter('hitch_x_offset').value
        init_x = self.get_parameter('initial_x').value
        init_y = self.get_parameter('initial_y').value
        init_yaw = self.get_parameter('initial_yaw').value

        # Tractor state [x, y, yaw]
        self.x = init_x
        self.y = init_y
        self.yaw = init_yaw

        # Trailer state: yaw (psi) and position [x_t, y_t]
        self.psi = init_yaw
        hitch_x0 = init_x + self.hitch_x_off * np.cos(init_yaw)
        self.x_t = hitch_x0 - self.L_t * np.cos(self.psi)
        self.y_t =  - self.L_t * np.sin(self.psi)

        # For finite-difference twist calculation
        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_yaw = self.yaw
        self.prev_psi = self.psi
        self.prev_x_t = self.x_t
        self.prev_y_t = self.y_t

        # Latest command
        self.last_cmd = Control()

        # Subscribers
        self.create_subscription(
            Control,
            '/control/command/control_cmd',
            self.cmd_callback,
            10)

        # Publishers
        self.tractor_pose_pub = self.create_publisher(PoseStamped, '/status/tractor/pose', 10)
        self.trailer_pose_pub = self.create_publisher(PoseStamped, '/status/trailer/pose', 10)
        self.tractor_twist_pub = self.create_publisher(TwistStamped, '/status/tractor/twist', 10)
        self.trailer_twist_pub = self.create_publisher(TwistStamped, '/status/trailer/twist', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Simulation loop timer (50 Hz)
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.update_simulation)

        self.get_logger().info('Tractor-trailer Simulator has been started.')

    def cmd_callback(self, msg):
        """Store the latest control command."""
        self.last_cmd = msg
        self.last_cmd.stamp = self.get_clock().now().to_msg()

    def update_simulation(self):
        """Update states, publish poses, twists & transforms."""
        dt = self.timer_period

        # --- Snapshot previous state for twist calculation ---
        prev_x, prev_y = self.prev_x, self.prev_y
        prev_yaw = self.prev_yaw
        prev_x_t, prev_y_t = self.prev_x_t, self.prev_y_t
        prev_psi = self.prev_psi

        if (self.last_cmd.stamp.sec + 1) < self.get_clock().now().to_msg().sec:
            self.last_cmd = Control()  # Reset to 0 to avoid using stale data

        v = self.last_cmd.longitudinal.velocity
        delta = self.last_cmd.lateral.steering_tire_angle

        # --- Update tractor (bicycle model) ---
        self.x += v * np.cos(self.yaw) * dt
        self.y += v * np.sin(self.yaw) * dt
        self.yaw += (v / self.L) * np.tan(delta) * dt
        self.yaw = self.normalize_angle(self.yaw)

        # --- Compute hitch point (with offset) ---
        hitch_x = self.x + self.hitch_x_off * np.cos(self.yaw)
        hitch_y = self.y + self.hitch_x_off * np.sin(self.yaw)


        # --- Update trailer kinematics ---
        self.psi += (v / self.L_t) * np.sin(self.yaw - self.psi) * dt
        self.psi = self.normalize_angle(self.psi)

        # Trailer axle center L_t behind hitch
        self.x_t = hitch_x - self.L_t * np.cos(self.psi)
        self.y_t = hitch_y - self.L_t * np.sin(self.psi)

        # --- Timestamp ---
        now = self.get_clock().now().to_msg()

        # --- Publish tractor PoseStamped ---
        tractor_msg = PoseStamped()
        tractor_msg.header.stamp = now
        tractor_msg.header.frame_id = 'map'
        tractor_msg.pose.position.x = self.x
        tractor_msg.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.yaw)
        tractor_msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.tractor_pose_pub.publish(tractor_msg)

        # --- Publish trailer PoseStamped ---
        trailer_msg = PoseStamped()
        trailer_msg.header.stamp = now
        trailer_msg.header.frame_id = 'map'
        trailer_msg.pose.position.x = self.x_t
        trailer_msg.pose.position.y = self.y_t
        q_t = quaternion_from_euler(0, 0, self.psi)
        trailer_msg.pose.orientation = Quaternion(x=q_t[0], y=q_t[1], z=q_t[2], w=q_t[3])
        self.trailer_pose_pub.publish(trailer_msg)

        # --- Compute finite-difference twists ---
        vx = (self.x - prev_x) / dt
        vy = (self.y - prev_y) / dt
        yaw_dot = (self.yaw - prev_yaw) / dt

        v_tx = (self.x_t - prev_x_t) / dt
        v_ty = (self.y_t - prev_y_t) / dt
        psi_dot = (self.psi - prev_psi) / dt

        # --- Publish tractor TwistStamped ---
        tractor_twist = TwistStamped()
        tractor_twist.header.stamp = now
        tractor_twist.header.frame_id = 'map'
        tractor_twist.twist.linear.x = vx
        tractor_twist.twist.linear.y = vy
        tractor_twist.twist.angular.z = yaw_dot
        self.tractor_twist_pub.publish(tractor_twist)

        # --- Publish trailer TwistStamped ---
        trailer_twist = TwistStamped()
        trailer_twist.header.stamp = now
        trailer_twist.header.frame_id = 'map'
        trailer_twist.twist.linear.x = v_tx
        trailer_twist.twist.linear.y = v_ty
        trailer_twist.twist.angular.z = psi_dot
        self.trailer_twist_pub.publish(trailer_twist)

        # --- Broadcast tractor TF ---
        t_tr = TransformStamped()
        t_tr.header.stamp = now
        t_tr.header.frame_id = 'map'
        t_tr.child_frame_id = 'base_link'
        t_tr.transform.translation.x = self.x
        t_tr.transform.translation.y = self.y
        t_tr.transform.translation.z = 0.0
        t_tr.transform.rotation = tractor_msg.pose.orientation
        self.tf_broadcaster.sendTransform(t_tr)

        # --- Broadcast trailer TF ---
        t_tl = TransformStamped()
        t_tl.header.stamp = now
        t_tl.header.frame_id = 'map'
        t_tl.child_frame_id = 'trailer_link'
        t_tl.transform.translation.x = self.x_t
        t_tl.transform.translation.y = self.y_t
        t_tl.transform.translation.z = 0.0
        t_tl.transform.rotation = trailer_msg.pose.orientation
        self.tf_broadcaster.sendTransform(t_tl)

        # --- Store current state for next twist calculation ---
        self.prev_x, self.prev_y = self.x, self.y
        self.prev_yaw = self.yaw
        self.prev_x_t, self.prev_y_t = self.x_t, self.y_t
        self.prev_psi = self.psi

    def normalize_angle(self, angle):
        """Normalize an angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    sim = VehicleSimulator()
    rclpy.spin(sim)
    sim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()