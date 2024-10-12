#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time
from datetime import datetime

class AngleSerialMonitor(Node):
    def __init__(self):
        super().__init__('angle_serial_monitor')
        
        # Khởi tạo Serial
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info(f"Đã kết nối Serial: {self.ser.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Lỗi kết nối Serial: {e}")
            return
            
        # Tạo subscriber
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info("Starting Angle Serial Monitor...")

    def decode_serial_data(self, data_hex):
        """Giải mã dữ liệu hex nhận được từ Serial"""
        try:
            # Chuyển string hex thành list bytes
            data = bytes.fromhex(data_hex)
            
            if len(data) >= 7 and data[0] == 0xFF and data[1] == 0xF5:
                angle_B = data[2]    # Góc B
                angle_C = data[3]    # Góc C
                angle_B_prime = data[4]    # Góc B'
                angle_C_prime = data[5]    # Góc C'
                
                return {
                    'B': angle_B,
                    'C': angle_C,
                    'B_prime': angle_B_prime,
                    'C_prime': angle_C_prime
                }
            return None
        except Exception as e:
            self.get_logger().error(f"Lỗi giải mã dữ liệu: {e}")
            return None

    def joint_state_callback(self, msg):
        """Xử lý callback và in ra dữ liệu góc"""
        try:
            # Đọc dữ liệu từ Serial (nếu có)
            if self.ser.in_waiting:
                serial_data = self.ser.read(self.ser.in_waiting)
                data_hex = serial_data.hex()
                self.get_logger().info(f"\nDữ liệu hex từ MCU: {data_hex}")
                
                # Giải mã và in ra các góc
                angles = self.decode_serial_data(data_hex)
                if angles:
                    self.get_logger().info("Các góc nhận được từ MCU:")
                    self.get_logger().info(f"Góc B: {angles['B']}°")
                    self.get_logger().info(f"Góc C: {angles['C']}°")
                    self.get_logger().info(f"Góc B': {angles['B_prime']}°")
                    self.get_logger().info(f"Góc C': {angles['C_prime']}°")

            # In thông tin từ ROS2 cho tham khảo
            self.get_logger().info("\nDữ liệu góc joint từ ROS2:")
            for name, pos in zip(msg.name, msg.position):
                if name in ['joint_2', 'joint_4']:  # Chỉ in joint_2 và joint_4
                    self.get_logger().info(f"{name}: {pos:.4f} rad")
            
            self.get_logger().info("-" * 40)
                    
        except Exception as e:
            self.get_logger().error(f"Lỗi trong callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    monitor = AngleSerialMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        if hasattr(monitor, 'ser'):
            monitor.ser.close()
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()