#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, TwistStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import bisect
import math

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        self.declare_parameter('num_trajectory_points', 40)
        self.trajectory_size = int(self.get_parameter('num_trajectory_points').value)

        self.declare_parameter('target_velocity', 2.0)
        self.target_velocity = float(self.get_parameter('target_velocity').value)

        self.declare_parameter('publish_rate', 10)        # in Hz
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        timer_period = 1.0 / self.publish_rate              # secondsm, publish rate is in int
        self.timer = self.create_timer(timer_period, self.publish_trajectory)

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
        # 1) must have path + current trailer pose/twist
        if self.path is None or self.trailer_pose is None or self.trailer_twist is None:
            if self.path is None: self.get_logger().warn('no path yet')
            if self.trailer_pose is None: self.get_logger().warn('no trailer pose yet')
            if self.trailer_twist is None: self.get_logger().warn('no trailer twist yet')
            return

        pts = self.path.poses
        M = len(pts)
        N = self.trajectory_size  # 40
        dt = 1.0 / self.publish_rate
        v = self.target_velocity  # 1.5 m/s

        # 2) build the cumulative arc-length of the path
        s = [0.0]
        for i in range(1, M):
            dx = pts[i].pose.position.x - pts[i - 1].pose.position.x
            dy = pts[i].pose.position.y - pts[i - 1].pose.position.y
            s.append(s[-1] + math.hypot(dx, dy))
        total_len = s[-1]

        # 3) find s0 = arc-length at the point on the path closest to current trailer
        tx = self.trailer_pose.pose.position.x
        ty = self.trailer_pose.pose.position.y
        best_i, best_d2 = 0, float('inf')
        for i, P in enumerate(pts):
            dx = P.pose.position.x - tx
            dy = P.pose.position.y - ty
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2, best_i = d2, i
        s0 = s[best_i]

        # 4) decide where to MERGE: halfway = ~3m ahead
        horizon_dist = v * N * dt  # ~1.5 * 40 * 0.1 = 6m
        merge_dist = horizon_dist * 0.5  # = 3m
        merge_s = min(s0 + merge_dist, total_len)

        # 5) find the two path indices around merge_s, then interpolate
        idx = bisect.bisect_left(s, merge_s)
        if idx == 0:
            i0, i1 = 0, 1
        elif idx >= M:
            i0, i1 = M - 2, M - 1
        else:
            i0, i1 = idx - 1, idx

        s_a, s_b = s[i0], s[i1]
        α = (merge_s - s_a) / (s_b - s_a) if s_b > s_a else 0.0

        pa = pts[i0].pose.position
        pb = pts[i1].pose.position
        xm = pa.x + α * (pb.x - pa.x)
        ym = pa.y + α * (pb.y - pa.y)
        # tangent heading at the merge point
        yaw_m = math.atan2(pb.y - pa.y, pb.x - pa.x)

        from geometry_msgs.msg import Pose
        Pm = Pose()
        Pm.position.x = xm
        Pm.position.y = ym
        Pm.position.z = 0.0
        qxm, qym, qzm, qwm = quaternion_from_euler(0, 0, yaw_m)
        Pm.orientation.x = qxm
        Pm.orientation.y = qym
        Pm.orientation.z = qzm
        Pm.orientation.w = qwm

        # 6) build a cubic-Hermite from current trailer pose P0 → Pm over K points
        P0 = self.trailer_pose.pose
        # extract its heading
        q0 = P0.orientation
        _, _, yaw0 = euler_from_quaternion([q0.x, q0.y, q0.z, q0.w])

        # “tangent magnitudes” = distance P0→Pm
        d0m = math.hypot(Pm.position.x - P0.position.x,
                         Pm.position.y - P0.position.y)
        T0 = (math.cos(yaw0) * d0m, math.sin(yaw0) * d0m)
        T1 = (math.cos(yaw_m) * d0m, math.sin(yaw_m) * d0m)

        # number of blend points (including both ends)
        K = max(2, N // 2)  # 20

        traj = Trajectory()
        traj.header.frame_id = 'map'
        traj.header.stamp = self.get_clock().now().to_msg()

        # 7) HERMITE BLEND: k = 0 .. K
        for k in range(K + 1):
            tnorm = k / float(K)
            h00 = 2 * tnorm ** 3 - 3 * tnorm ** 2 + 1
            h10 = tnorm ** 3 - 2 * tnorm ** 2 + tnorm
            h01 = -2 * tnorm ** 3 + 3 * tnorm ** 2
            h11 = tnorm ** 3 - tnorm ** 2

            # position
            x = (h00 * P0.position.x
                 + h10 * T0[0]
                 + h01 * Pm.position.x
                 + h11 * T1[0])
            y = (h00 * P0.position.y
                 + h10 * T0[1]
                 + h01 * Pm.position.y
                 + h11 * T1[1])

            # derivative for heading
            dh00 = 6 * tnorm ** 2 - 6 * tnorm
            dh10 = 3 * tnorm ** 2 - 4 * tnorm + 1
            dh01 = -6 * tnorm ** 2 + 6 * tnorm
            dh11 = 3 * tnorm ** 2 - 2 * tnorm
            dxdt = (dh00 * P0.position.x
                    + dh10 * T0[0]
                    + dh01 * Pm.position.x
                    + dh11 * T1[0])
            dydt = (dh00 * P0.position.y
                    + dh10 * T0[1]
                    + dh01 * Pm.position.y
                    + dh11 * T1[1])
            yaw = math.atan2(dydt, dxdt)
            qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)

            pt = TrajectoryPoint()
            pt.pose.position.x = x
            pt.pose.position.y = y
            pt.pose.position.z = 0.0
            pt.pose.orientation.x = qx
            pt.pose.orientation.y = qy
            pt.pose.orientation.z = qz
            pt.pose.orientation.w = qw

            # constant speed
            pt.longitudinal_velocity_mps = v

            # time-from-start
            T = k * dt
            pt.time_from_start.sec = int(T)
            pt.time_from_start.nanosec = int((T - int(T)) * 1e9)

            traj.points.append(pt)

        # 8) ARC FOLLOW: k = K+1 .. N-1
        for k in range(K + 1, N):
            # how far along the path (arc-length) from merge_s?
            extra_s = v * dt * (k - K)
            target_s = min(merge_s + extra_s, total_len)

            # locate segment and interpolate
            idx = bisect.bisect_left(s, target_s)
            if idx == 0:
                i0, i1 = 0, 1
            elif idx >= M:
                i0, i1 = M - 2, M - 1
            else:
                i0, i1 = idx - 1, idx

            sa, sb = s[i0], s[i1]
            α = ((target_s - sa) / (sb - sa)) if sb > sa else 0.0
            pa = pts[i0].pose.position
            pb = pts[i1].pose.position

            x = pa.x + α * (pb.x - pa.x)
            y = pa.y + α * (pb.y - pa.y)
            yaw = math.atan2(pb.y - pa.y, pb.x - pa.x)
            qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)

            pt = TrajectoryPoint()
            pt.pose.position.x = x
            pt.pose.position.y = y
            pt.pose.position.z = 0.0
            pt.pose.orientation.x = qx
            pt.pose.orientation.y = qy
            pt.pose.orientation.z = qz
            pt.pose.orientation.w = qw

            pt.longitudinal_velocity_mps = v

            T = k * dt
            pt.time_from_start.sec = int(T)
            pt.time_from_start.nanosec = int((T - int(T)) * 1e9)

            traj.points.append(pt)

            if target_s >= total_len:
                break

        # 9) publish
        self.traj_publisher.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    rclpy.spin(trajectory_publisher)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()