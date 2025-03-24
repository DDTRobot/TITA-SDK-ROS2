import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry, Path
from tita_locomotion_interfaces.msg import LocomotionCmd
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
from std_srvs.srv import Trigger
from rclpy.time import Time
from datetime import datetime
import time

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower_node')
        self.last_time = 0.0
        self.odom_subscriber = self.create_subscription(
            Odometry,
            #'tower/odometry',
            'chassis/odometry',
            self.odom_callback,
            10)
        
        self.path_subscriber = self.create_subscription(
            Path,
            'path_topic',
            self.path_callback,
            10)
        
        self.twist_publisher = self.create_publisher(
            LocomotionCmd,
            'command/user/command',
            10)
        self.reset_client = self.create_client(Trigger, 'chassis/srv/reset')
        self.send_request() 
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.path = None
        self.current_goal_index = 0
        self.track_fin = False
        self.mark = True

    def send_request(self):
        if self.reset_client.wait_for_service(timeout_sec=1.0):
            self.future = self.reset_client.call_async(Trigger.Request())
            return self.future
        else:
            self.get_logger().info('Service not available, will not send request')
            return None
    def quaternion_to_zyx(self,quaternion):
        # 将四元数转换为旋转矩阵
        quat = [quaternion.x, quaternion.y ,quaternion.z ,quaternion.w]
        rotation = R.from_quat(quat)
        # 将旋转矩阵转换为欧拉角（ZYX顺序）
        zyx = rotation.as_euler('zyx', degrees=False)
        return zyx

    def odom_callback(self, msg):
        t1 = time.time()
        ding_dt = t1 - self.last_time
        # self.get_logger().info("ding " + str(ding_dt))
        # self.get_logger().info(f'ding dt {ding_dt}')
        if self.last_time == 0.0:  # 首次初始化
            self.last_time = t1
            return  # 跳过第一次无效计算
        self.last_time = t1
        current_time = datetime.now()
        timestamp = current_time.timestamp()

        if self.path is None or self.current_goal_index >= len(self.path.poses):
            if self.track_fin == True:
                self.get_logger().info(f'FIN!')
                self.track_fin = False
            cmd_vel = LocomotionCmd()
            cmd_vel.twist.linear.x = 0.0
            cmd_vel.twist.angular.z = 0.0
            self.twist_publisher.publish(cmd_vel)
            return
        #Initialize
        if self.current_goal_index < 1:
            self.track_fin = True
            cmd_vel = LocomotionCmd()
            cmd_vel.twist.linear.x = 0.0
            cmd_vel.twist.angular.z = 0.0
            self.twist_publisher.publish(cmd_vel)
            self.current_goal_index = self.current_goal_index + 1
            return

        dx = 0
        dy = 0
        dyaw = 0
        dx2 = 0
        dy2 = 0
        dist = 0
        pos_yaw_err = 0
        dt = ding_dt#0.016

        
        last_target_pose = self.path.poses[self.current_goal_index-1].pose
        target_pose = self.path.poses[self.current_goal_index].pose
        
        
        current_pose = msg.pose.pose


        zyx_target = self.quaternion_to_zyx(target_pose.orientation)
        last_zyx_target = self.quaternion_to_zyx(last_target_pose.orientation)
        zyx = self.quaternion_to_zyx(current_pose.orientation)

        yaw = zyx[0]

        # self.get_logger().info(f'Target Pose: Position -> x: {target_pose.position.x}, y: {target_pose.position.y},yaw:{zyx_target[0]}')
        # self.get_logger().info(f'Current Pose: Position -> x: {current_pose.position.x}, y: {current_pose.position.y},yaw:{zyx[0]}')        

        dx = target_pose.position.x - last_target_pose.position.x
        dy = target_pose.position.y - last_target_pose.position.y

        dist = math.sqrt(dx**2 + dy**2)
        dyaw = zyx_target[0] - last_zyx_target[0]
        #dyaw =  math.atan2(target_pose.position.y - last_target_pose.position.y,target_pose.position.x - last_target_pose.position.x)
        
        if dyaw > 3.0:
            dyaw -= 6.28
        elif dyaw < -3.0:
            dyaw += 6.28

        dx2 = math.cos(yaw) * dx + math.sin(yaw) * dy
        dy2 = -math.sin(yaw) * dx + math.cos(yaw) * dy

        if abs(dy2) > 0.005:
            if dx2 >= 0.0:
                vel_ff = (dyaw * dist / 2) / (math.sin(dyaw/2) * dt)
            else:
                vel_ff = -(dyaw * dist / 2) / (math.sin(dyaw/2) * dt)
        else:
            if dx2 >= 0.0:
                vel_ff = dist / dt
            else:
                vel_ff = -dist / dt

        omega_ff = dyaw / dt

        dx_err = target_pose.position.x - current_pose.position.x
        dy_err = target_pose.position.y - current_pose.position.y
        dyaw_err = zyx_target[0] - zyx[0]

        len_err = math.sqrt(dx_err * dx_err + dy_err * dy_err)

        dx2_err = math.cos(yaw) * dx_err + math.sin(yaw) * dy_err
        dy2_err = -math.sin(yaw) * dx_err + math.cos(yaw) * dy_err

        pos_yaw_err = math.atan2(dy2_err,dx2_err)

        if pos_yaw_err > 1.57:
            pos_yaw_err = pos_yaw_err - 3.14
    
        elif pos_yaw_err < -1.57:
            pos_yaw_err = pos_yaw_err + 3.14
        
        if dyaw_err > 3:
            dyaw_err -= 6.28
        elif dyaw_err < -3:
            dyaw_err += 6.28

        vel_err = 0
        
        if dx2_err >= 0:
            vel_err = len_err
        else:
            vel_err = -len_err
            
        if vel_err > 1:
            vel_err = 1
        elif vel_err < -1:
            vel_err = -1
        cmd_vel = LocomotionCmd()

        kp_v = 1.0
        kp_w = 1.0
        ki_w = 0

        cmd_vel.twist.linear.x = kp_v * vel_err + 0.6 * vel_ff
        cmd_vel.twist.angular.z = 1.0 * omega_ff + kp_w * dyaw_err + 0.8 * pos_yaw_err
        # cmd_vel.linear.x = 0.4
        # cmd_vel.angular.z = 0.0
        # self.get_logger().info(f'vel -> x: {dyaw}, vel_om: {dist}')  
        # if distance < 0.1:  # Threshold to consider "close enough"
        self.current_goal_index = self.current_goal_index + 1
        cmd_vel.pose.position.z = 0.2
                
        self.twist_publisher.publish(cmd_vel)

    def path_callback(self, msg):
        if self.mark:
            if self.current_goal_index == 0:  
                if self.send_request():
                    self.get_logger().info(f'Reset request sent successfully')
                else:
                    self.get_logger().info(f'Reset request failed')
            self.get_logger().info(f'Get path')
            self.path = msg
            self.current_goal_index = 0
            self.get_logger().info(f'L: {len(self.path.poses)}')
            self.mark = False 

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    executor = MultiThreadedExecutor(num_threads=4)  # 设置线程数
    executor.add_node(node)
    try:
        executor.spin()  # 启动多线程执行器
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
    