import rclpy
from rclpy.node import Node
import pickle
from nav_msgs.msg import Odometry, Path
from rclpy.clock import Clock

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher = self.create_publisher(Path, 'path_topic', 10)  # Replace with your topic name
        self.timer = self.create_timer(1.0, self.publish_path)  # Publish at 1Hz
        self.path = self.load_path_from_file('/home/robot/tita_ws/path2.pkl')  # Replace with your actual file name
        # self.publish_path()
    def load_path_from_file(self, file_name):
        self.get_logger().info('Loading path from file...')
        with open(file_name, 'rb') as f:
            path = pickle.load(f)
        return path

    def publish_path(self):
        if self.path is not None:
            self.path.header.stamp = Clock().now().to_msg()
            self.path.header.frame_id = 'chassis_odom'
            self.publisher.publish(self.path)
            self.get_logger().info('Path published.')

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()