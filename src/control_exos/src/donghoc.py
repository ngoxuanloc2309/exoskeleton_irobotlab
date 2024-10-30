#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
from enum import Enum

class LegState(Enum):
    WAITING = 0    # Chờ đến lượt di chuyển
    MOVING_UP = 1  # Đang di chuyển trên quỹ đạo elip
    MOVING_DOWN = 2  # Đang di chuyển xuống vị trí ban đầu
    DONE = 3       # Đã hoàn thành chu kỳ chuyển động

class EllipseTrajectoryNode(Node):
    def __init__(self):
        super().__init__('ellipse_trajectory_node')
        
        # Khởi tạo action clients
        self.chan_trai_client = ActionClient(self, FollowJointTrajectory, '/chan_trai_group_controller/follow_joint_trajectory')
        self.chan_phai_client = ActionClient(self, FollowJointTrajectory, '/chan_phai_group_controller/follow_joint_trajectory')
        
        # Thông số robot
        self.link_length_1 = 410.0  # Chiều dài đốt 1
        self.link_length_2 = 390.0  # Chiều dài đốt 2
        
        # Thông số quỹ đạo elip
        self.ellipse_a = 150.0  # Bán kính lớn
        self.ellipse_b = 80.0   # Bán kính nhỏ
        self.y_offset = 760.0   # Độ cao cơ sở
        
        # Thông số điều khiển
        self.declare_parameter('movement_duration', 0.05)   #0.5
        self.movement_duration = self.get_parameter('movement_duration').value
        
        # Khởi tạo trạng thái cho mỗi chân
        self.chan_trai_state = LegState.WAITING
        self.chan_phai_state = LegState.WAITING
        
        # Vị trí hiện tại của mỗi chân
        self.x_trai = -150.0
        self.x_phai = -150.0
        
        # Cờ báo hoàn thành chuyển động
        self.trai_done = True
        self.phai_done = True
        
        # Timer điều khiển
        self.timer = self.create_timer(0.1, self.movement_control_loop)
        
        # Trạng thái ban đầu
        self.current_moving_leg = 'chan_trai'  # Bắt đầu với chân trái
    
    def inverse_kinematics(self, x, y):
        """Tính động học ngược cho một chân"""
        try:
            theta_x = math.atan2(x, y)
            c = y / math.cos(theta_x)
            
            cos_B = (self.link_length_2**2 + c**2 - self.link_length_1**2) / (2 * self.link_length_2 * c)
            cos_C = (self.link_length_1**2 + self.link_length_2**2 - c**2) / (2 * self.link_length_1 * self.link_length_2)
            
            cos_B = min(1.0, max(-1.0, cos_B))
            cos_C = min(1.0, max(-1.0, cos_C))
            
            angle_B = math.acos(cos_B)
            angle_C = math.acos(cos_C)
            
            d_B = -math.degrees(angle_B) - math.degrees(theta_x)
            d_C = 180.0 - math.degrees(angle_C)
            
            return d_B, d_C
        except Exception as e:
            self.get_logger().error(f'Inverse kinematics error: {str(e)}')
            return None, None

    def calculate_ellipse_point(self, x, is_moving_up):
        """Tính điểm trên quỹ đạo elip"""
        if is_moving_up:
            y = self.y_offset - math.sqrt(self.ellipse_b**2 * (1 - (x**2)/(self.ellipse_a**2)))
        else:
            y = self.y_offset
        return x, y

    def move_leg(self, leg_name, positions):
        """Gửi lệnh điều khiển cho một chân"""
        if positions[0] is None or positions[1] is None:
            self.get_logger().error(f'Invalid positions for {leg_name}')
            return
            
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
            
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.get_result_callback(f, leg_name))

    def get_result_callback(self, future, leg_name):
        """Callback khi nhận kết quả từ action server"""
        if leg_name == 'chan_trai':
            self.trai_done = True
        else:
            self.phai_done = True

    def movement_control_loop(self):
        """Hàm điều khiển chính"""
        if not (self.trai_done and self.phai_done):
            return
            
        self.trai_done = False
        self.phai_done = False
        
        # Xử lý chân trái
        if self.current_moving_leg == 'chan_trai':
            # Chân trái di chuyển theo quỹ đạo elip
            self.x_trai += 15
            x, y = self.calculate_ellipse_point(self.x_trai, True)
            angles_trai = self.inverse_kinematics(x, y)
            
            # Chân phải giữ nguyên ở mặt đất
            x_phai, y_phai = self.calculate_ellipse_point(self.x_phai, False)
            angles_phai = self.inverse_kinematics(x_phai, y_phai)
            
            if self.x_trai >= 150.0:
                self.x_trai = -150.0
                self.current_moving_leg = 'chan_phai'
        
        else:  # Chân phải đang di chuyển
            # Chân phải di chuyển theo quỹ đạo elip
            self.x_phai += 15
            x, y = self.calculate_ellipse_point(self.x_phai, True)
            angles_phai = self.inverse_kinematics(x, y)
            
            # Chân trái giữ nguyên ở mặt đất
            x_trai, y_trai = self.calculate_ellipse_point(self.x_trai, False)
            angles_trai = self.inverse_kinematics(x_trai, y_trai)
            
            if self.x_phai >= 150.0:
                self.x_phai = -150.0
                self.current_moving_leg = 'chan_trai'
        
        # Gửi lệnh điều khiển cho cả hai chân
        self.move_leg('chan_trai', angles_trai)
        self.move_leg('chan_phai', angles_phai)
        
        self.get_logger().info(f'Current moving leg: {self.current_moving_leg}')
        self.get_logger().info(f'Positions - Left: {self.x_trai:.2f}, Right: {self.x_phai:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = EllipseTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()