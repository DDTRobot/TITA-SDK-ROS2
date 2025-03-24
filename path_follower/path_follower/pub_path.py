import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class SinePathPublisher(Node):
    def __init__(self):
        super().__init__('sine_path_publisher')
        self.publisher_ = self.create_publisher(Path, 'sine_path', 10)
        self.publish_sine_path()

    def publish_sine_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'chassis_odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # 创建正弦路径
        for i in range(100):
            x = i * 0.02
            y = math.sin(x*3.1415)

            pose = PoseStamped()
            pose.header.frame_id = 'chassis_odom'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)
        self.get_logger().info('Published sine path')

def main(args=None):
    rclpy.init(args=args)
    sine_path_publisher = SinePathPublisher()
    rclpy.spin_once(sine_path_publisher)

    sine_path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
