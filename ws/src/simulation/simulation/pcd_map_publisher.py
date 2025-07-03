import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from std_msgs.msg import Header
import os

# A simple PCD file loader.
# For a more robust solution, consider installing a library like pypcd
def load_pcd(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()

    header_end_line = 0
    for i, line in enumerate(lines):
        if line.startswith('DATA'):
            header_end_line = i + 1
            break

    points = []
    for line in lines[header_end_line:]:
        if line.strip():
            parts = line.split()
            points.append([float(parts[0]), float(parts[1]), float(parts[2])])

    return np.array(points, dtype=np.float32)


class PcdMapPublisher(Node):
    """
    Loads a .pcd file and publishes it as a PointCloud2 message.
    """
    def __init__(self):
        super().__init__('pcd_map_publisher')

        self.declare_parameter('pcd_file_path', '')
        pcd_file = self.get_parameter('pcd_file_path').get_parameter_value().string_value

        if not os.path.exists(pcd_file):
            self.get_logger().error(f"PCD file not found at: {pcd_file}")
            return

        self.points = load_pcd(pcd_file)

        self.publisher_ = self.create_publisher(PointCloud2, '/map', 10)
        self.timer = self.create_timer(2.0, self.publish_map) # Publish every 2 seconds

        self.get_logger().info(f"Loaded {len(self.points)} points from {pcd_file}. Publishing map...")

    def publish_map(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        # Define the fields in the PointCloud2 message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Create the PointCloud2 message
        msg = PointCloud2(
            header=header,
            height=1,
            width=len(self.points),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=12,  # 3 fields * 4 bytes/field
            row_step=12 * len(self.points),
            data=self.points.tobytes()
        )
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    pcd_map_publisher = PcdMapPublisher()
    rclpy.spin(pcd_map_publisher)
    pcd_map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
