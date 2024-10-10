#!/usr/bin/env python3
#########################################################################
#                                                                        #
#                    Author: Logan Ngo                                    #
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
        self.x2 = 0.41  # meters
        self.x3 = 0.39  # meters
        self.x4 = 0.41  # meters
        
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
        
        # Lưu trữ giá trị góc hiện tại
        self.current_angles = np.zeros(4)  # [goc_B, goc_C, goc_B_prime, goc_C_prime]
        
        self.get_logger().info('Joint Angle Controller đã được khởi tạo')

    def calculate_angles(self, joint2_deg, joint4_deg):
        """
        Tính toán các góc B, C, B', C' và trả về dưới dạng mảng
        Return: [goc_B, goc_C, goc_B_prime, goc_C_prime]
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

            # Chuyển đổi kết quả sang độ và trả về dạng mảng
            angles_array = np.array([
                math.degrees(goc_B),
                math.degrees(goc_C),
                math.degrees(goc_B_prime),
                math.degrees(goc_C_prime)
            ])

            # Log kết quả
            self.get_logger().info(
                f'Các góc đã tính [B, C, B_prime, C_prime]: {angles_array}'
            )
            
            return angles_array

        except Exception as e:
            self.get_logger().error(f'Lỗi trong tính toán góc: {str(e)}')
            return np.zeros(4)  # Trả về mảng 0 nếu có lỗi

    def joint_state_callback(self, msg):
        """Xử lý callback khi nhận được thông tin trạng thái joint mới"""
        try:
            # Tìm các chỉ số của joint2 và joint4 trong message
            joint2_idx = msg.name.index('joint_2')
            joint4_idx = msg.name.index('joint_4')
            
            # Lấy giá trị góc
            joint2_deg = math.degrees(msg.position[joint2_idx])
            joint4_deg = math.degrees(msg.position[joint4_idx])
            
            # Tính toán các góc mới
            self.current_angles = self.calculate_angles(joint2_deg, joint4_deg)
            
            # Log kết quả
            self.get_logger().info(
                f'Góc hiện tại [B, C, B_prime, C_prime]: {self.current_angles}'
            )
                
        except ValueError as e:
            self.get_logger().error(f'Joint không tìm thấy trong message: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Lỗi trong joint_state_callback: {str(e)}')

    def get_current_angles(self):
        """Trả về mảng các góc hiện tại"""
        return self.current_angles

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