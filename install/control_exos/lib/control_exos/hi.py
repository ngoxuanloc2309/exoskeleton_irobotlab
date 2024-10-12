#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import serial
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import glob

class SerialController(Node):
    def __init__(self):
        super().__init__('serial_controller')
        
        # Debug flag
        self.debug = True
        
        # Thông số cố định cho tính toán góc
        self.x1 = 0.41
        self.x2 = 0.39
        self.x3 = 0.41
        self.x4 = 0.39
        
        # Tự động tìm cổng Serial với nhiều lần thử
        self.ser = None
        self.max_retries = 3
        self.retry_count = 0
        self.find_and_connect_serial()
            
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile
        )
        
        self.get_logger().info('Serial Controller đã được khởi tạo')
        if self.debug:
            self.get_logger().info(f'Đang subscribe tới topic: /joint_states')

    def find_and_connect_serial(self):
        if self.retry_count >= self.max_retries:
            self.get_logger().error('Đã vượt quá số lần thử kết nối Serial')
            return

        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        
        if self.debug:
            self.get_logger().info(f'Tìm thấy các cổng: {ports}')
        
        if not ports:
            self.get_logger().error('Không tìm thấy cổng Serial nào')
            self.retry_count += 1
            self.create_timer(1.0, self.find_and_connect_serial)
            return
            
        for port in ports:
            try:
                self.ser = serial.Serial(port, 115200, timeout=1)
                self.get_logger().info(f'Kết nối Serial thành công tại {port}')
                if self.debug:
                    self.get_logger().info(f'Serial settings: {self.ser.get_settings()}')
                self.retry_count = 0
                return
            except serial.SerialException as e:
                self.get_logger().error(f'Không thể kết nối với {port}: {str(e)}')
                
        self.retry_count += 1
        self.create_timer(1.0, self.find_and_connect_serial)

    def calculate_angles(self, joint2_deg, joint4_deg):
        if self.debug:
            self.get_logger().info(f'Input angles - joint2: {joint2_deg}°, joint4: {joint4_deg}°')
            
        try:
            joint2_rad = math.radians(joint2_deg)
            joint4_rad = math.radians(joint4_deg)

            goc_C = math.radians(180) - joint2_rad
            goc_C_prime = math.radians(180) - joint4_rad

            cos_C = math.cos(goc_C)
            h1 = math.sqrt(abs(-(2 * cos_C * self.x1 * self.x2 - self.x1**2 - self.x2**2)))

            cos_C_prime = math.cos(goc_C_prime)
            h2 = math.sqrt(abs(-(2 * cos_C_prime * self.x3 * self.x4 - self.x3**2 - self.x4**2)))

            cos_B = (self.x1**2 + h1**2 - self.x2**2) / (2 * self.x1 * h1)
            cos_B = min(1.0, max(-1.0, cos_B))
            goc_B = math.acos(cos_B)

            cos_B_prime = (self.x3**2 + h2**2 - self.x4**2) / (2 * self.x3 * h2)
            cos_B_prime = min(1.0, max(-1.0, cos_B_prime))
            goc_B_prime = math.acos(cos_B_prime)

            angles = [
                math.degrees(goc_B),
                math.degrees(goc_C),
                math.degrees(goc_B_prime),
                math.degrees(goc_C_prime)
            ]
            
            if self.debug:
                self.get_logger().info(f'Calculated angles: {[f"{angle:.2f}°" for angle in angles]}')
                
            return angles

        except Exception as e:
            self.get_logger().error(f'Lỗi trong tính toán góc: {str(e)}')
            return [0, 0, 0, 0]

    def joint_state_callback(self, msg):
        if self.debug:
            self.get_logger().info(f'Nhận được message từ /joint_states')
            self.get_logger().info(f'Joint names: {msg.name}')
            self.get_logger().info(f'Joint positions: {msg.position}')
            
        try:
            if 'joint_2' in msg.name and 'joint_4' in msg.name:
                joint2_idx = msg.name.index('joint_2')
                joint4_idx = msg.name.index('joint_4')
                
                joint2_deg = math.degrees(msg.position[joint2_idx])
                joint4_deg = math.degrees(msg.position[joint4_idx])
                
                angles = self.calculate_angles(joint2_deg, joint4_deg)
                
                control_data = np.zeros(7, dtype=np.uint8)
                control_data[0] = 255  # Start byte
                control_data[1] = 245  # Command byte
                
                for i, angle in enumerate(angles):
                    bounded_angle = max(0, min(255, int(angle)))
                    control_data[i+2] = bounded_angle

                if self.ser and self.ser.is_open:
                    self.ser.write(bytes(control_data))
                    if self.debug:
                        self.get_logger().info(f'Đã gửi dữ liệu: {[f"0x{x:02X}" for x in control_data]}')
                else:
                    self.get_logger().warn('Cổng Serial không khả dụng, thử kết nối lại...')
                    self.find_and_connect_serial()
                    
        except Exception as e:
            self.get_logger().error(f'Lỗi trong callback: {str(e)}')

    def __del__(self):
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = SerialController()
        rclpy.spin(controller)
    except Exception as e:
        print(f'Lỗi trong main: {str(e)}')
    finally:
        if controller:
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()