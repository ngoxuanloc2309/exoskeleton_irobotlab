#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import serial
import struct
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import glob

class ImprovedControlAngleNode(Node):
    def __init__(self):
        super().__init__('improved_control_angle_node')
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.chan_trai_pub = self.create_publisher(Float64MultiArray, '/chan_trai_group_controller/commands', qos_profile)
        self.chan_phai_pub = self.create_publisher(Float64MultiArray, '/chan_phai_group_controller/commands', qos_profile)
        
        # Tìm và kết nối cổng serial tự động
        self.ser = self.connect_serial()
        
        # Timer để đọc dữ liệu từ STM32
        self.create_timer(0.05, self.read_serial)  # 20Hz
        
        # Biến để theo dõi thời gian của lần publish cuối cùng
        self.last_publish_time = self.get_clock().now()
        
        # Thêm các tham số có thể cấu hình
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('angle_threshold', 0.5)  # độ
        
        self.get_logger().info('ImprovedControlAngleNode đã được khởi tạo')

    def connect_serial(self):
        ports = glob.glob('/dev/ttyUSB*')# + glob.glob('/dev/ttyACM*')
        if not ports:
            self.get_logger().error('Không tìm thấy cổng Serial USB!')
            raise serial.SerialException("No USB Serial port found")
        
        for port in ports:
            try:
                ser = serial.Serial(port, 115200, timeout=0.1)
                self.get_logger().info(f'Đã kết nối với cổng {port}')
                return ser
            except serial.SerialException:
                continue
        
        self.get_logger().error('Không thể kết nối với bất kỳ cổng Serial nào!')
        raise serial.SerialException("Unable to connect to any Serial port")

    def read_serial(self):
        try:
            if self.ser.in_waiting >= 7:
                data = self.ser.read(7)
                if data[0] == 255 and data[1] == 245:
                    goc_B, goc_C, goc_B_prime, goc_C_prime = struct.unpack('BBBB', data[2:6])
                    
                    # Chuyển đổi góc
                    joints = self.convert_angles(goc_B, goc_C, goc_B_prime, goc_C_prime)
                    
                    current_time = self.get_clock().now()
                    
                    # Kiểm tra tần suất publish
                    if self.should_publish(current_time):
                        self.publish_joint_states(joints, current_time)
                        self.publish_controller_commands(joints)
                        
                        self.last_publish_time = current_time
                        
                        self.log_debug_info(current_time, goc_B, goc_C, goc_B_prime, goc_C_prime)
                    
        except serial.SerialException as e:
            self.get_logger().error(f'Lỗi khi đọc dữ liệu Serial: {str(e)}')
            self.ser = self.connect_serial()  # Thử kết nối lại

    def convert_angles(self, goc_B, goc_C, goc_B_prime, goc_C_prime):
        return [
            -goc_B * (math.pi / 180),
            (180 - goc_C) * (math.pi / 180),
            -goc_B_prime * (math.pi / 180),
            (180 - goc_C_prime) * (math.pi / 180)
        ]

    def should_publish(self, current_time):
        publish_rate = self.get_parameter('publish_rate').value
        return (current_time - self.last_publish_time).nanoseconds >= 1e9 / publish_rate

    def publish_joint_states(self, joints, current_time):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = current_time.to_msg()
        joint_state_msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        joint_state_msg.position = joints
        self.joint_state_pub.publish(joint_state_msg)

    def publish_controller_commands(self, joints):
        chan_trai_msg = Float64MultiArray(data=joints[:2])
        self.chan_trai_pub.publish(chan_trai_msg)
        
        chan_phai_msg = Float64MultiArray(data=joints[2:])
        self.chan_phai_pub.publish(chan_phai_msg)

    def log_debug_info(self, current_time, goc_B, goc_C, goc_B_prime, goc_C_prime):
        self.get_logger().debug(f'Thời gian xử lý: {(self.get_clock().now() - current_time).nanoseconds} ns')
        self.get_logger().info(f'Đã nhận góc: B={goc_B}, C={goc_C}, B\'={goc_B_prime}, C\'={goc_C_prime}')

    def run(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ImprovedControlAngleNode()
    node.run()

if __name__ == '__main__':
    main()