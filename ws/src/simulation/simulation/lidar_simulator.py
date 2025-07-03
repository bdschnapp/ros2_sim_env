import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np
import os
from scipy.spatial import KDTree
from tf_transformations import euler_from_quaternion, inverse_matrix, quaternion_matrix


def load_pcd(file_path):
    """A simple PCD file loader for ASCII format."""
    with open(file_path, 'r') as f:
        lines = f.readlines()
    header_end_line = 0
    for i, line in enumerate(lines):
        if line.strip().startswith('DATA'):
            header_end_line = i + 1
            break
    points = []
    for line in lines[header_end_line:]:
        if line.strip():
            try:
                parts = line.split()
                points.append([float(parts[0]), float(parts[1]), float(parts[2])])
            except (ValueError, IndexError):
                # Skip malformed lines
                continue
    return np.array(points, dtype=np.float32)


class LidarSimulator(Node):
    """
    Simulates a LiDAR sensor by extracting points from a PCD map
    based on the vehicle's ground truth position.
    """

    def __init__(self):
        super().__init__('lidar_simulator')

        # Declare and get parameters
        self.declare_parameter('pcd_file_path', '')
        self.declare_parameter('lidar_range', 10.0)
        self.declare_parameter('lidar_frame_id', 'base_link')
        pcd_file = self.get_parameter('pcd_file_path').get_parameter_value().string_value
        self.lidar_range = self.get_parameter('lidar_range').get_parameter_value().double_value
        self.lidar_frame = self.get_parameter('lidar_frame_id').get_parameter_value().string_value

        if not os.path.exists(pcd_file):
            self.get_logger().error(f"PCD file not found at: {pcd_file}")
            return

        # Load map and build KD-Tree for efficient point searching
        self.map_points = load_pcd(pcd_file)
        self.map_kdtree = KDTree(self.map_points)
        self.get_logger().info(f"Loaded {len(self.map_points)} points for LiDAR simulation.")

        # Publisher for the simulated lidar scan
        self.scan_publisher = self.create_publisher(PointCloud2, '/cloud', 10)

        # Subscriber for the ground truth pose from the vehicle simulator
        self.create_subscription(PoseStamped, '/current_pose', self.pose_callback, 10)

    def pose_callback(self, msg):
        pos = msg.pose.position
        orient = msg.pose.orientation

        # Find all points in the map within the LiDAR's range
        indices = self.map_kdtree.query_ball_point([pos.x, pos.y, pos.z], self.lidar_range)
        if not indices:
            return

        visible_points = self.map_points[indices]

        # --- Transform points from map frame to lidar frame ---
        # Create the vehicle's transformation matrix (map -> base_link)
        translation_vector = np.array([pos.x, pos.y, pos.z])
        rotation_mat = quaternion_matrix([orient.x, orient.y, orient.z, orient.w])[:3, :3]

        # Create a 4x4 homogeneous transformation matrix
        transform_map_to_vehicle = np.identity(4)
        transform_map_to_vehicle[:3, :3] = rotation_mat
        transform_map_to_vehicle[:3, 3] = translation_vector

        # Get the inverse transform (vehicle -> map) to bring map points into the vehicle's frame
        transform_vehicle_to_map = inverse_matrix(transform_map_to_vehicle)

        # Add a fourth component (1.0) for homogeneous coordinates
        visible_points_h = np.hstack((visible_points, np.ones((visible_points.shape[0], 1))))

        # Transform points into the vehicle's frame (which is the lidar_link frame in this simple case)
        points_in_lidar_frame = (transform_vehicle_to_map @ visible_points_h.T).T

        final_points = points_in_lidar_frame[:, :3].astype(np.float32)

        # --- Publish as PointCloud2 message ---
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.lidar_frame)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        point_cloud_msg = PointCloud2(
            header=header, height=1, width=len(final_points),
            is_dense=True, is_bigendian=False, fields=fields,
            point_step=12, row_step=12 * len(final_points),
            data=final_points.tobytes()
        )
        self.scan_publisher.publish(point_cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    lidar_simulator = LidarSimulator()
    rclpy.spin(lidar_simulator)
    lidar_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()