#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import serial
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import glob
import time

class SerialController(Node):
    def __init__(self):
        super().__init__('serial_controller')
        
        # Thông số cố định cho tính toán góc
        self.x1 = 0.41  # meters
        self.x2 = 0.39  # meters
        self.x3 = 0.41  # meters
        self.x4 = 0.39  # meters
        self.y = 0.8    # meters
        self.x = 0      # meters
        
        # Tự động tìm cổng Serial với nhiều lần thử
        self.ser = None
        self.max_retries = 3
        self.retry_count = 0
        self.find_and_connect_serial()
            
        # Thiết lập QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
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
        if self.retry_count >= self.max_retries:
            self.get_logger().error('Đã vượt quá số lần thử kết nối Serial')
            return

        ports = glob.glob('/dev/ttyUSB*') # + glob.glob('/dev/ttyACM*')
        
        if not ports:
            self.get_logger().error('Không tìm thấy cổng Serial nào')
            self.retry_count += 1
            self.create_timer(1.0, self.find_and_connect_serial)  # Thử lại sau 1 giây
            return
            
        for port in ports:
            try:
                self.ser = serial.Serial(port, 115200, timeout=1)
                self.get_logger().info(f'Kết nối Serial thành công tại {port}')
                self.retry_count = 0  # Reset số lần thử
                return
            except serial.SerialException as e:
                self.get_logger().error(f'Không thể kết nối với {port}: {str(e)}')
                
        self.retry_count += 1
        self.create_timer(1.0, self.find_and_connect_serial)  # Thử lại sau 1 giây

    def calculate_angles(self, joint1_deg, joint2_deg, joint3_deg, joint4_deg):
        """Tính toán các góc B, C, B', C'"""
        try:
            # Tính góc B, C, B', C'
            goc_B = -joint1_deg  # Breal
            goc_C = 180 - joint2_deg
            goc_B_prime = -joint3_deg  # B'real
            goc_C_prime = 180 - joint4_deg

            return [goc_B, goc_C, goc_B_prime, goc_C_prime]

        except Exception as e:
            self.get_logger().error(f'Lỗi trong tính toán góc: {str(e)}')
            return [0, 0, 0, 0]

    def joint_state_callback(self, msg):
        """Xử lý callback khi nhận được thông tin góc mới"""
        try:
            # Tìm index của các joint
            joint_indices = {name: msg.name.index(name) for name in ['joint_1', 'joint_2', 'joint_3', 'joint_4']}
            
            # Lấy giá trị góc và chuyển sang độ
            joint_angles = {name: math.degrees(msg.position[idx]) for name, idx in joint_indices.items()}
            
            self.get_logger().info(f'Nhận được góc: {joint_angles}')
            
            # Tính toán các góc B, C, B', C'
            angles = self.calculate_angles(
                joint_angles['joint_1'], 
                joint_angles['joint_2'], 
                joint_angles['joint_3'], 
                joint_angles['joint_4']
            )
            
            self.get_logger().info(f'Góc tính toán: B = {angles[0]:.2f}, C = {angles[1]:.2f}, B\' = {angles[2]:.2f}, C\' = {angles[3]:.2f}')
            
            # Tạo control data
            control_data = np.zeros(7, dtype=np.uint8)
            control_data[0] = 255  # Start byte
            control_data[1] = 245  # Command byte
            
            # Điền 4 góc vào control data
            for i, angle in enumerate(angles):
                # Đảm bảo góc nằm trong khoảng 0-255 và lấy giá trị tuyệt đối
                bounded_angle = max(0, min(255, int(abs(angle))))
                control_data[i+2] = bounded_angle

            # Gửi dữ liệu qua Serial
            if self.ser and self.ser.is_open:
                self.ser.write(bytes(control_data))
                self.get_logger().info(f'Đã gửi dữ liệu: {control_data}')
                
                # Thêm đoạn code để đọc phản hồi từ STM32 (nếu có)
                response = self.ser.read(1)
                if response:
                    self.get_logger().info(f'Nhận được phản hồi từ STM32: {response}')
                    time.sleep(0.1)
                else:
                    self.get_logger().warn('Không nhận được phản hồi từ STM32')
            else:
                self.get_logger().warn('Cổng Serial không khả dụng, thử kết nối lại...')
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
        rclpy.shutdown()#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import serial
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import glob
import time

class SerialController(Node):
    def __init__(self):
        super().__init__('serial_controller')
        
        # Thông số cố định cho tính toán góc
        self.x1 = 0.41  # meters
        self.x2 = 0.39  # meters
        self.x3 = 0.41  # meters
        self.x4 = 0.39  # meters
        self.y = 0.8    # meters
        self.x = 0      # meters
        
        # Tự động tìm cổng Serial với nhiều lần thử
        self.ser = None
        self.max_retries = 3
        self.retry_count = 0
        self.find_and_connect_serial()
            
        # Thiết lập QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
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
        if self.retry_count >= self.max_retries:
            self.get_logger().error('Đã vượt quá số lần thử kết nối Serial')
            return

        ports = glob.glob('/dev/ttyUSB*') # + glob.glob('/dev/ttyACM*')
        
        if not ports:
            self.get_logger().error('Không tìm thấy cổng Serial nào')
            self.retry_count += 1
            self.create_timer(1.0, self.find_and_connect_serial)  # Thử lại sau 1 giây
            return
            
        for port in ports:
            try:
                self.ser = serial.Serial(port, 115200, timeout=1)
                self.get_logger().info(f'Kết nối Serial thành công tại {port}')
                self.retry_count = 0  # Reset số lần thử
                return
            except serial.SerialException as e:
                self.get_logger().error(f'Không thể kết nối với {port}: {str(e)}')
                
        self.retry_count += 1
        self.create_timer(1.0, self.find_and_connect_serial)  # Thử lại sau 1 giây

    def calculate_angles(self, joint1_deg, joint2_deg, joint3_deg, joint4_deg):
        """Tính toán các góc B, C, B', C'"""
        try:
            # Chuyển đổi góc từ độ sang radian
            joint1_rad = math.radians(joint1_deg)
            joint2_rad = math.radians(joint2_deg)
            joint3_rad = math.radians(joint3_deg)
            joint4_rad = math.radians(joint4_deg)

            # Tính góc B, C, B', C'
            goc_B = -joint1_rad
            goc_C = math.radians(180) - joint2_rad
            goc_B_prime = -joint3_rad
            goc_C_prime = math.radians(180) - joint4_rad

            # Chuyển đổi sang độ
            return [
                math.degrees(goc_B),
                math.degrees(goc_C),
                math.degrees(goc_B_prime),
                math.degrees(goc_C_prime)
            ]

        except Exception as e:
            self.get_logger().error(f'Lỗi trong tính toán góc: {str(e)}')
            return [0, 0, 0, 0]

    def joint_state_callback(self, msg):
        """Xử lý callback khi nhận được thông tin góc mới"""
        try:
            # Tìm index của các joint
            joint_indices = {name: msg.name.index(name) for name in ['joint_1', 'joint_2', 'joint_3', 'joint_4']}
            
            # Lấy giá trị góc và chuyển sang độ
            joint_angles = {name: math.degrees(msg.position[idx]) for name, idx in joint_indices.items()}
            
            self.get_logger().info(f'Nhận được góc: {joint_angles}')
            
            # Tính toán các góc B, C, B', C'
            angles = self.calculate_angles(
                joint_angles['joint_1'], 
                joint_angles['joint_2'], 
                joint_angles['joint_3'], 
                joint_angles['joint_4']
            )
            
            self.get_logger().info(f'Góc tính toán: B = {angles[0]:.2f}, C = {angles[1]:.2f}, B\' = {angles[2]:.2f}, C\' = {angles[3]:.2f}')
            
            # Tạo control data
            control_data = np.zeros(7, dtype=np.uint8)
            control_data[0] = 255  # Start byte
            control_data[1] = 245  # Command byte
            
            # Điền 4 góc vào control data
            for i, angle in enumerate(angles):
                # Đảm bảo góc nằm trong khoảng 0-255
                bounded_angle = max(0, min(255, int(angle)))  # Thêm 128 để chuyển từ -128~127 sang 0~255
                control_data[i+2] = bounded_angle

            # Gửi dữ liệu qua Serial
            if self.ser and self.ser.is_open:
                self.ser.write(bytes(control_data))
                self.get_logger().info(f'Đã gửi dữ liệu: {control_data}')
                
                # Thêm đoạn code để đọc phản hồi từ STM32 (nếu có)
                response = self.ser.read(1)
                if response:
                    self.get_logger().info(f'Nhận được phản hồi từ STM32: {response}')
                    #time.sleep(0.1)
                else:
                    self.get_logger().warn('Không nhận được phản hồi từ STM32')
            else:
                self.get_logger().warn('Cổng Serial không khả dụng, thử kết nối lại...')
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

if __name__ == '__main__':
    main()