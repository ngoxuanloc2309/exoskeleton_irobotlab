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
        
        # Tự động tìm cổng Serial
        self.ser = None
        self.find_and_connect_serial()
            
        # Thiết lập QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Thứ tự các góc trong message cần tìm
        self.angle_names = {
            'goc_B': None,      # góc B
            'goc_C': None,      # góc C
            'goc_B_prime': None,  # góc B'
            'goc_C_prime': None   # góc C'
        }
        
        # Subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile
        )
        
        self.get_logger().info('Serial Controller đã được khởi tạo')

    def find_and_connect_serial(self):
        """Tìm và kết nối với cổng Serial khả dụng"""
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        
        if not ports:
            self.get_logger().error('Không tìm thấy cổng Serial nào')
            return
            
        for port in ports:
            try:
                self.ser = serial.Serial(port, 115200, timeout=1)
                self.get_logger().info(f'Kết nối Serial thành công tại {port}')
                return
            except serial.SerialException as e:
                self.get_logger().error(f'Không thể kết nối với {port}: {str(e)}')
                
        self.get_logger().error('Không thể kết nối với bất kỳ cổng Serial nào')

    def joint_state_callback(self, msg):
        """Xử lý callback khi nhận được thông tin góc mới"""
        try:
            # In ra tên các joint trong message lần đầu để debug
            if not any(self.angle_names.values()):
                self.get_logger().info(f'Tên các góc trong message: {msg.name}')
                # Tìm index cho các góc
                for i, name in enumerate(msg.name):
                    if 'goc_B' in name.lower():
                        self.angle_names['goc_B'] = i
                    elif 'goc_C' in name.lower():
                        self.angle_names['goc_C'] = i
                    elif 'goc_B_prime' in name.lower() or 'goc_b_prime' in name.lower():
                        self.angle_names['goc_B_prime'] = i
                    elif 'goc_C_prime' in name.lower() or 'goc_c_prime' in name.lower():
                        self.angle_names['goc_C_prime'] = i

            # Kiểm tra xem đã tìm thấy tất cả các góc chưa
            if None in self.angle_names.values():
                missing_angles = [name for name, idx in self.angle_names.items() if idx is None]
                self.get_logger().warn(f'Không tìm thấy các góc: {missing_angles}')
                return

            # Lấy giá trị các góc
            angles = []
            for angle_name, idx in self.angle_names.items():
                angle = msg.position[idx]
                # Nếu góc đang ở radian, chuyển sang độ
                if abs(angle) < math.pi:  # Giả sử góc nhỏ hơn pi là radian
                    angle = math.degrees(angle)
                angles.append(angle)

            # Format số để in ra dễ đọc
            formatted_angles = [f"{angle:.2f}" for angle in angles]
            self.get_logger().info(
                f'Góc [B, C, B\', C\']: {formatted_angles}'
            )
            
            # Tạo control data
            control_data = np.zeros(7, dtype=np.uint8)
            control_data[0] = 255  # Start byte
            control_data[1] = 245  # Command byte
            
            # Điền 4 góc vào control data
            for i, angle in enumerate(angles):
                # Đảm bảo góc nằm trong khoảng 0-255
                bounded_angle = max(0, min(255, int(angle)))
                control_data[i+2] = bounded_angle & 0xFF
            
            # Gửi dữ liệu qua Serial
            if self.ser and self.ser.is_open:
                self.get_logger().info(f'Dữ liệu gửi: {control_data}')
                try:
                    self.ser.write(bytes(control_data))
                except serial.SerialException as e:
                    self.get_logger().error(f'Lỗi gửi Serial: {str(e)}')
                    self.find_and_connect_serial()
            else:
                self.get_logger().error('Cổng Serial không khả dụng')
                self.find_and_connect_serial()
                
        except Exception as e:
            self.get_logger().error(f'Lỗi trong callback: {str(e)}')

    def __del__(self):
        """Đóng cổng Serial khi node bị hủy"""
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
