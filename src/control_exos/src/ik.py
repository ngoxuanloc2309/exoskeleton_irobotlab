#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import numpy as np

class EllipseTrajectoryNode(Node):
    def __init__(self):
        super().__init__('ellipse_trajectory_node')
        
        # Khởi tạo action clients cho hai chân
        self.chan_trai_client = ActionClient(self, FollowJointTrajectory, '/chan_trai_group_controller/follow_joint_trajectory')
        self.chan_phai_client = ActionClient(self, FollowJointTrajectory, '/chan_phai_group_controller/follow_joint_trajectory')
        
        # Các thông số của robot
        self.a = 410.0  # Chiều dài đốt 1
        self.b = 390.0  # Chiều dài đốt 2
        
        # Các thông số của quỹ đạo elip
        self.ellipse_a = 150.0  # Bán kính lớn
        self.ellipse_b = 80.0   # Bán kính nhỏ
        self.y_offset = 760.0   # Độ cao cơ sở
        
        # Thông số điều khiển
        self.declare_parameter('movement_duration', 0.1)
        self.movement_duration = self.get_parameter('movement_duration').value
        
        # Trạng thái di chuyển
        self.x_trai = -150.0
        self.x_phai = -150.0
        self.chan_trai_up = True
        self.chan_phai_up = False
        self.trai_done = True
        self.phai_done = True
        
        # Timer để điều khiển chuyển động
        self.timer = self.create_timer(0.1, self.trajectory_control_loop)
        
    def inverse_kinematics(self, x, y):
        """Tính động học ngược cho một chân"""
        theta_x = math.atan2(x, y)
        c = y / math.cos(theta_x)
        
        # Tính các góc sử dụng định lý cosin
        cos_B = (self.b * self.b + c * c - self.a * self.a) / (2 * self.b * c)
        cos_C = (self.a * self.a + self.b * self.b - c * c) / (2 * self.a * self.b)
        
        # Giới hạn giá trị của cos để tránh lỗi domain error
        cos_B = min(1.0, max(-1.0, cos_B))
        cos_C = min(1.0, max(-1.0, cos_C))
        
        goc_B = math.acos(cos_B)
        goc_C = math.acos(cos_C)
        
        # Chuyển đổi sang độ
        theta_x_deg = math.degrees(theta_x)
        d_B = -math.degrees(goc_B) - theta_x_deg
        d_C = 180.0 - math.degrees(goc_C)
        
        return d_B, d_C
    
    def calculate_leg_position(self, x, is_up):
        """Tính vị trí cho một chân dựa trên trạng thái"""
        if is_up:
            # Chân đang ở phần trên của elip
            y = self.y_offset - math.sqrt(self.ellipse_b**2 * (1 - (x**2)/(self.ellipse_a**2)))
        else:
            # Chân đang ở dưới (đường thẳng)
            y = self.y_offset
        return x, y
    
    def move_leg(self, leg_name, positions):
        """Gửi lệnh điều khiển cho một chân"""
        client = self.chan_trai_client if leg_name == 'chan_trai' else self.chan_phai_client
        joint_names = ['joint_1', 'joint_2'] if leg_name == 'chan_trai' else ['joint_3', 'joint_4']
        
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [math.radians(pos) for pos in positions]
        point.time_from_start = Duration(sec=int(self.movement_duration),
                                       nanosec=int((self.movement_duration % 1) * 1e9))
        
        trajectory.points = [point]
        goal_msg.trajectory = trajectory
        
        self.get_logger().info(f'Moving {leg_name} to {positions}')
        future = client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.goal_response_callback(f, leg_name))
    
    def goal_response_callback(self, future, leg_name):
        """Callback khi nhận phản hồi từ action server"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Goal rejected for {leg_name}')
            return
        
        self.get_logger().info(f'Goal accepted for {leg_name}')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.get_result_callback(f, leg_name))
    
    def get_result_callback(self, future, leg_name):
        """Callback khi nhận kết quả từ action server"""
        result = future.result().result
        if leg_name == 'chan_trai':
            self.trai_done = True
        else:
            self.phai_done = True
        self.get_logger().info(f'Result for {leg_name}: {result.error_code}')
    
    def update_leg_positions(self):
        """Cập nhật vị trí của cả hai chân"""
        # Cập nhật vị trí chân trái
        if self.chan_trai_up:
            self.x_trai += 6
            if self.x_trai >= 150.0:
                self.x_trai = 150.0
                self.chan_trai_up = False
        else:
            self.x_trai -= 6
            if self.x_trai <= -150.0:
                self.x_trai = -150.0
                self.chan_trai_up = True
        
        # Cập nhật vị trí chân phải
        if self.chan_phai_up:
            self.x_phai += 6
            if self.x_phai >= 150.0:
                self.x_phai = 150.0
                self.chan_phai_up = False
        else:
            self.x_phai -= 6
            if self.x_phai <= -150.0:
                self.x_phai = -150.0
                self.chan_phai_up = True
    
    def trajectory_control_loop(self):
        """Hàm điều khiển chính cho quỹ đạo"""
        try:
            # Chỉ cập nhật khi cả hai chân đã hoàn thành chuyển động trước đó
            if not (self.trai_done and self.phai_done):
                return
                
            # Cập nhật vị trí của cả hai chân
            self.update_leg_positions()
            
            # Tính toán vị trí và góc cho chân trái
            x_trai, y_trai = self.calculate_leg_position(self.x_trai, self.chan_trai_up)
            d_B_trai, d_C_trai = self.inverse_kinematics(x_trai, y_trai)
            
            # Tính toán vị trí và góc cho chân phải
            x_phai, y_phai = self.calculate_leg_position(self.x_phai, self.chan_phai_up)
            d_B_phai, d_C_phai = self.inverse_kinematics(x_phai, y_phai)
            
            # Gửi lệnh điều khiển cho cả hai chân
            self.trai_done = False
            self.phai_done = False
            self.move_leg('chan_trai', [d_B_trai, d_C_trai])
            self.move_leg('chan_phai', [d_B_phai, d_C_phai])
            
            # Log thông tin
            self.get_logger().info(f'Left leg: x={x_trai:.2f}, y={y_trai:.2f}, up={self.chan_trai_up}')
            self.get_logger().info(f'Right leg: x={x_phai:.2f}, y={y_phai:.2f}, up={self.chan_phai_up}')
            
        except Exception as e:
            self.get_logger().error(f'Error in trajectory control: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = EllipseTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()