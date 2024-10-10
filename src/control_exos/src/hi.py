#!/usr/bin/env python3
#########################################################################
#                                                                        #
#                    Author: Logan Ngo                         #
#                                                                        #
#########################################################################

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

class JointAngleController(Node):
    def __init__(self):
        super().__init__('joint_angle_controller')
        
        # Các thông số cố định
        self.x1 = 0.39  # meters
        self.x2 = 0.41    # meters
        self.x3 = 0.39  # meters
        self.x4 = 0.41     # meters
        
        # Thiết lập QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.chan_trai_publisher = self.create_publisher(
            JointTrajectory, 
            '/chan_trai_group_controller/joint_trajectory',
            qos_profile
        )
        
        self.chan_phai_publisher = self.create_publisher(
            JointTrajectory,
            '/chan_phai_group_controller/joint_trajectory',
            qos_profile
        )
        
        # Subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile
        )
        
        # Joint configuration
        self.joint_config = {
            'joint_1': {'display_name': 'B', 'limit_min': -180, 'limit_max': 180},
            'joint_2': {'display_name': 'C', 'limit_min': -180, 'limit_max': 180},
            'joint_3': {'display_name': "B_prime", 'limit_min': -180, 'limit_max': 180},
            'joint_4': {'display_name': "C_prime", 'limit_min': -180, 'limit_max': 180}
        }
        
        # Lưu trữ giá trị góc hiện tại
        self.current_angles = {joint: 0.0 for joint in self.joint_config.keys()}
        
        self.get_logger().info('Joint Angle Controller đã được khởi tạo')

    def calculate_angles(self, joint2_deg, joint4_deg):
        """
        Tính toán các góc B, C, B', C' dựa trên các phương trình đã cho
        """
        try:
            # Chuyển đổi góc từ độ sang radian
            joint2_rad = math.radians(joint2_deg)
            joint4_rad = math.radians(joint4_deg)

            # Tính góc C và C'
            goc_C = math.radians(180) - joint2_rad
            goc_C_prime = math.radians(180) - joint4_rad

            # Tính h1
            cos_C = math.cos(goc_C)
            h1 = math.sqrt(-(2 * cos_C * self.x1 * self.x2 - self.x1**2 - self.x2**2))

            # Tính h2
            cos_C_prime = math.cos(goc_C_prime)
            h2 = math.sqrt(-(2 * cos_C_prime * self.x3 * self.x4 - self.x3**2 - self.x4**2))

            # Tính góc B
            cos_B = (self.x1**2 + h1**2 - self.x2**2) / (2 * self.x1 * h1)
            goc_B = math.acos(cos_B)

            # Tính góc B'
            cos_B_prime = (self.x3**2 + h2**2 - self.x4**2) / (2 * self.x3 * h2)
            goc_B_prime = math.acos(cos_B_prime)

            # Chuyển đổi kết quả sang độ
            angles = {
                'B': math.degrees(goc_B),
                'C': math.degrees(goc_C),
                'B_prime': math.degrees(goc_B_prime),
                'C_prime': math.degrees(goc_C_prime)
            }

            self.get_logger().info(f'Các góc đã tính: {angles}')
            return angles

        except Exception as e:
            self.get_logger().error(f'Lỗi trong tính toán góc: {str(e)}')
            return None

    def joint_state_callback(self, msg):
        """Xử lý callback khi nhận được thông tin trạng thái joint mới"""
        try:
            # Cập nhật góc hiện tại
            for name, position in zip(msg.name, msg.position):
                if name in self.joint_config:
                    self.current_angles[name] = position
                    angle_deg = math.degrees(position)
                    
            # Tính toán các góc mới
            if 'joint_2' in self.current_angles and 'joint_4' in self.current_angles:
                joint2_deg = math.degrees(self.current_angles['joint_2'])
                joint4_deg = math.degrees(self.current_angles['joint_4'])
                
                angles = self.calculate_angles(joint2_deg, joint4_deg)
                if angles:
                    self.get_logger().info(
                        "Góc B: {:.2f}°, Góc C: {:.2f}°, Góc B_prime: {:.2f}°, Góc C_prime: {:.2f}°".format(
                            angles['B'],
                            angles['C'],
                            angles['B_prime'],
                            angles['C_prime']
                        )
                    )
                
        except Exception as e:
            self.get_logger().error(f'Lỗi trong joint_state_callback: {str(e)}')

    def validate_angles(self, angles_deg):
        """Kiểm tra tính hợp lệ của các góc"""
        for joint, angle in angles_deg.items():
            if joint not in self.joint_config:
                raise ValueError(f'Joint không hợp lệ: {joint}')
            
            min_angle = self.joint_config[joint]['limit_min']
            max_angle = self.joint_config[joint]['limit_max']
            
            if not min_angle <= angle <= max_angle:
                raise ValueError(
                    f'Góc {angle}° cho {joint} nằm ngoài giới hạn [{min_angle}, {max_angle}]'
                )

    def create_trajectory_msg(self, joint_names, positions, duration_sec=1.0):
        """Tạo message JointTrajectory"""
        msg = JointTrajectory()
        msg.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        
        msg.points = [point]
        return msg

    def publish_angles(self, angles_dict, duration_sec=1.0):
        """Publish các góc mới cho cả hai chân"""
        try:
            # Tạo và publish message cho chân trái
            chan_trai_msg = self.create_trajectory_msg(
                ['joint_1', 'joint_2'],
                [angles_dict['joint_1'], angles_dict['joint_2']],
                duration_sec
            )
            self.chan_trai_publisher.publish(chan_trai_msg)

            # Tạo và publish message cho chân phải
            chan_phai_msg = self.create_trajectory_msg(
                ['joint_3', 'joint_4'],
                [angles_dict['joint_3'], angles_dict['joint_4']],
                duration_sec
            )
            self.chan_phai_publisher.publish(chan_phai_msg)
            
            self.get_logger().info('Đã publish các góc mới thành công')
        except Exception as e:
            self.get_logger().error(f'Lỗi khi publish góc: {str(e)}')

    def move_to_angles(self, angles_deg, duration_sec=1.0):
        """Di chuyển các joint đến góc mới (đơn vị: độ)"""
        try:
            # Kiểm tra tính hợp lệ của các góc
            self.validate_angles(angles_deg)
            
            # Chuyển đổi từ độ sang radian
            angles_rad = {
                joint: math.radians(angle) 
                for joint, angle in angles_deg.items()
            }
            
            # Publish các góc mới
            self.publish_angles(angles_rad, duration_sec)
        except Exception as e:
            self.get_logger().error(f'Lỗi trong move_to_angles: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = JointAngleController()
        rclpy.spin(controller)
    except Exception as e:
        print(f'Lỗi trong main: {str(e)}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    #######