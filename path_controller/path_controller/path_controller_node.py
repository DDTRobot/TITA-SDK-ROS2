import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import time
import pickle

class OdomPathManager(Node):
    def __init__(self):
        super().__init__('path_controller_node')
        
        # 订阅 /odom 话题
        self.subscription = self.create_subscription(
            Odometry,
            # 'tower/Odometry',
            'chassis/odometry',
            self.listener_callback,
            10)
        
        # 发布 /path 话题
        self.path_publisher = self.create_publisher(Path, 'path_rec', 10)
        self.path = Path()
        self.path.header.frame_id = 'chassis_odom'
        self.start_record = False
        # 创建服务
        self.srv = self.create_service(Trigger, 'publish_path', self.handle_publish_path)

        # self.reset_client = self.create_client(Trigger, "chassis/srv/reset")
        self.reset_client = self.create_client(Trigger, 'chassis/srv/reset')
        
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        # self.request = Trigger.Request()

        self.start_path_record_srv = self.create_service(Trigger, 'start_record_path', self.start_record_path)

        self.end_path_record_srv = self.create_service(Trigger, 'end_record_path', self.end_record_path)
        
        self.get_logger().info('Node initialized and ready.')

    def send_request(self):
                # Wait for Service B to be available
        if self.reset_client.wait_for_service(timeout_sec=1.0):
            self.future = self.reset_client.call_async(Trigger.Request())
            # rclpy.spin_until_future_complete(self, self.future)
            # return self.future.result()
    

    def callback_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Response: {response.success}, {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed {str(e)}')

    def listener_callback(self, msg):
        if self.start_record:
            current_pose = PoseStamped()
            current_pose.header = msg.header
            current_pose.pose = msg.pose.pose
            self.path.poses.append(current_pose)
            self.get_logger().info(f'Pose added. Total poses: {len(self.path.poses)}')

    def start_record_path(self, request, response):
        self.path = Path()
        self.send_request()
        # self.future = self.reset_client.call_async(self.request)
        # time.sleep(0.3)
        self.start_record = True
        response.success = True
        response.message = 'Start record successfully.'
        self.get_logger().info('Start record successfully.')
        return response

    def end_record_path(self, request, response):
        self.start_record = False
        self.path_callback("path2.pkl",self.path)
        response.success = True
        response.message = 'End record successfully.'
        self.get_logger().info('End record successfully.')
        return response
 
    def path_callback(self,file_name,msg):
        self.get_logger().info('Path received, saving to file...')
        with open(file_name, 'wb') as f:
            pickle.dump(self.path, f)

    def save_record_path(self, request, response):
        self.start_record = False
        response.success = True
        response.message = 'End record successfully.'
        self.get_logger().info('End record successfully.')
        return response

    def handle_publish_path(self, request, response):
        self.send_request()
        # self.future = self.reset_client.call_async(self.request)
        # time.sleep(0.3)
        # 发布路径
        self.path_publisher.publish(self.path)
        response.success = True
        response.message = 'Path published successfully.'
        self.get_logger().info('Path published.')
        return response
    def load_path_from_file(self, file_name):
        self.get_logger().info('Loading path from file...')
        with open(file_name, 'rb') as f:
            path = pickle.load(f)
        return path
def main(args=None):
    rclpy.init(args=args)
    node = OdomPathManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()