import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from tf_transformations import quaternion_from_euler


class PathPublisher(Node):
    """
    Publishes a static circular path as nav_msgs/Path.
    """
    def __init__(self):
        super().__init__('path_publisher')
        self.declare_parameter('radius', 50.0)
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 50.0)
        self.declare_parameter('num_points', 500)
        self.radius = self.get_parameter('radius').value
        self.cx = self.get_parameter('center_x').value
        self.cy = self.get_parameter('center_y').value
        self.num_points = int(self.get_parameter('num_points').value)

        self.publisher = self.create_publisher(Path, '/planning/scenario_planning/path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)

    def publish_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        angles = np.linspace(0, 2*np.pi, self.num_points, endpoint=False)
        for theta in angles:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = self.cx + self.radius * np.cos(theta)
            pose.pose.position.y = self.cy + self.radius * np.sin(theta)
            q = quaternion_from_euler(0, 0, theta + np.pi/2)
            pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            path.poses.append(pose)
        self.publisher.publish(path)

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
