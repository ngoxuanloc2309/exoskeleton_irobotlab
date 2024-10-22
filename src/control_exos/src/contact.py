#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import numpy as np

class FixedDataSender(Node):
    def __init__(self):
        super().__init__('fixed_data_sender')
        
        # Kết nối Serial
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Điều chỉnh cổng và baudrate nếu cần
        
        # Dữ liệu cố định để gửi (sử dụng numpy.uint8)
        self.data_to_send = np.array([255, 245, 32, 98, 89, 23, 0], dtype=np.uint8)
        
        # Timer để gửi dữ liệu định kỳ
        self.create_timer(0.0, self.send_data)  # Gửi mỗi 100ms
        
        self.get_logger().info('Fixed Data Sender đã được khởi tạo')

    def send_data(self):
        try:
            # Gửi dữ liệu numpy array trực tiếp
            self.ser.write(self.data_to_send.tobytes())
            self.get_logger().info(f'Đã gửi dữ liệu: {self.data_to_send}')
        except serial.SerialException as e:
            self.get_logger().error(f'Lỗi khi gửi dữ liệu: {str(e)}')

    def __del__(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        fixed_data_sender = FixedDataSender()
        rclpy.spin(fixed_data_sender)
    except Exception as e:
        print(f'Lỗi: {str(e)}')
    finally:
        if fixed_data_sender:
            fixed_data_sender.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()