import rclpy
from rclpy.node import Node
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, TwistStamped
from tf_transformations import quaternion_from_euler
import bisect
import math

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        self.declare_parameter('num_trajectory_points', 40)
        self.trajectory_size = int(self.get_parameter('num_trajectory_points').value)

        self.declare_parameter('target_velocity', 1.5)
        self.target_velocity = float(self.get_parameter('target_velocity').value)

        self.declare_parameter('publish_rate', 0.1)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.traj_publisher = self.create_publisher(Trajectory,
                                                    '/planning/scenario_planning/trajectory',
                                                    10)

        self.path = None
        self.trailer_pose = None
        self.trailer_twist = None
        self.sub_path = self.create_subscription(Path,
                                                 '/planning/scenario_planning/path',
                                                 self.path_callback,
                                                 10)

        self.sub_pose = self.create_subscription(PoseStamped,
                                                 '/status/trailer/pose',
                                                 self.trailer_pose_callback,
                                                 10)

        self.sub_twist = self.create_subscription(TwistStamped,
                                                  '/status/trailer/twist',
                                                  self.trailer_twist_callback,
                                                  10)

        self.timer = self.create_timer(self.publish_rate, self.publish_trajectory)
        self.get_logger().info('Trajectory Publisher has been started.')

    def path_callback(self, msg: Path):
        self.path = msg

    def trailer_pose_callback(self, msg: PoseStamped):
        self.trailer_pose = msg

    def trailer_twist_callback(self, msg: TwistStamped):
        self.trailer_twist = msg

    def publish_trajectory(self):
        path = self.path
        pose = self.trailer_pose
        twist = self.trailer_twist
        if path is None or pose is None or twist is None:
            if path is None: self.get_logger().warn('no path yet')
            if pose is None: self.get_logger().warn('no trailer pose yet')
            if twist is None: self.get_logger().warn('no trailer twist yet')
            return

        pts = path.poses
        M = len(pts)
        N = self.trajectory_size
        u0 = self.trailer_twist.twist.linear.x
        v_target = self.target_velocity
        dt = self.publish_rate

        s = [0.0]
        for i in range(1, M):
            dx = pts[i].pose.position.x - pts[i-1].pose.position.x
            dy = pts[i].pose.position.y - pts[i-1].pose.position.y
            s.append(s[-1] + math.hypot(dx, dy))
        total_len = s[-1]

        # 2) find the index on the path closest to the trailer now
        tx = pose.pose.position.x
        ty = pose.pose.position.y
        best_i, best_d2 = 0, float('inf')
        for i, P in enumerate(pts):
            dx = P.pose.position.x - tx
            dy = P.pose.position.y - ty
            d2 = dx*dx + dy*dy
            if d2 < best_d2:
                best_d2, best_i = d2, i
        s0 = s[best_i]

        traj = Trajectory()
        traj.header.frame_id = 'map'
        traj.header.stamp = self.get_clock().now().to_msg()

        for k in range(1, N + 1):
            # compute this point's speed by linear ramp:
            vk = u0 + (v_target - u0) * (k / float(N))

           # the distance we should cover in this dt:
            deltas = vk * dt
            target_s = (s0 + k * deltas) % total_len

            # binary search to find the smallest j with s[j] >= target_s
            j = bisect.bisect_left(s, target_s)
            if j >= M:
                j = M-1

            # pick that pose directly (could interpolate if you want)
            target_pose = pts[j].pose

            pt = TrajectoryPoint()
            pt.pose = target_pose
            pt.longitudinal_velocity_mps = v_target

            # schedule at k*dt seconds
            t = k * dt
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)

            traj.points.append(pt)

        self.traj_publisher.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    rclpy.spin(trajectory_publisher)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()