#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import serial
import struct
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import glob

class ImprovedSTM32ToROS2Node(Node):
    def __init__(self):
        super().__init__('improved_stm32_to_ros2_node')
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.chan_trai_pub = self.create_publisher(Float64MultiArray, '/chan_trai_group_controller/commands', qos_profile)
        self.chan_phai_pub = self.create_publisher(Float64MultiArray, '/chan_phai_group_controller/commands', qos_profile)
        
        # Tìm cổng serial tự động
        self.connect_serial()
        
        # Timer để đọc dữ liệu từ STM32
        self.create_timer(0.0001, self.read_serial)  # 100Hz
        
        self.get_logger().info('ImprovedSTM32ToROS2Node đã được khởi tạo')

    def connect_serial(self):
        ports = glob.glob('/dev/ttyUSB*')
        if not ports:
            self.get_logger().error('Không tìm thấy cổng Serial USB!')
            raise serial.SerialException("No USB Serial port found")
        
        port = ports[0]  # Sử dụng cổng đầu tiên tìm thấy
        try:
            self.ser = serial.Serial(port, 115200, timeout=1)
            self.get_logger().info(f'Đã kết nối với cổng {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Không thể mở cổng {port}: {str(e)}')
            raise

    def read_serial(self):
        try:
            if self.ser.in_waiting >= 7:
                data = self.ser.read(7)
                if data[0] == 255 and data[1] == 245:
                    goc_B, goc_C, goc_B_prime, goc_C_prime = struct.unpack('BBBB', data[2:6])
                    
                    # Chuyển đổi góc
                    joint1 = -goc_B * (math.pi / 180)
                    joint2 = (180 - goc_C) * (math.pi / 180)
                    joint3 = -goc_B_prime * (math.pi / 180)
                    joint4 = (180 - goc_C_prime) * (math.pi / 180)
                    
                    # Publish JointState
                    joint_state_msg = JointState()
                    joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                    joint_state_msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
                    joint_state_msg.position = [joint1, joint2, joint3, joint4]
                    self.joint_state_pub.publish(joint_state_msg)
                    
                    # Publish cho controllers
                    chan_trai_msg = Float64MultiArray()
                    chan_trai_msg.data = [joint1, joint2]
                    self.chan_trai_pub.publish(chan_trai_msg)
                    
                    chan_phai_msg = Float64MultiArray()
                    chan_phai_msg.data = [joint3, joint4]
                    self.chan_phai_pub.publish(chan_phai_msg)
                    
                    self.get_logger().info(f'Đã nhận góc: B={goc_B}, C={goc_C}, B\'={goc_B_prime}, C\'={goc_C_prime}')
                    self.get_logger().info(f'Đã publish góc: j1={joint1:.2f}, j2={joint2:.2f}, j3={joint3:.2f}, j4={joint4:.2f}')
        except serial.SerialException as e:
            self.get_logger().error(f'Lỗi đọc Serial: {str(e)}')
            self.connect_serial()  # Thử kết nối lại

    def __del__(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = ImprovedSTM32ToROS2Node()
        rclpy.spin(node)
    except Exception as e:
        print(f'Lỗi: {str(e)}')
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()