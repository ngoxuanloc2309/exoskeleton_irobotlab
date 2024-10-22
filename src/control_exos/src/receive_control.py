#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import serial
import struct
import math

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        
        # Kết nối Serial với STM32
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        
        # Action clients cho điều khiển chân
        self.chan_trai_client = ActionClient(self, FollowJointTrajectory, '/chan_trai_group_controller/follow_joint_trajectory')
        self.chan_phai_client = ActionClient(self, FollowJointTrajectory, '/chan_phai_group_controller/follow_joint_trajectory')
        
        # Thời gian chuyển động
        self.declare_parameter('movement_duration', 2.0)
        self.movement_duration = self.get_parameter('movement_duration').value
        
        # Timer để đọc dữ liệu từ STM32 và điều khiển robot
        self.create_timer(0.001, self.control_loop)
        
        self.get_logger().info('Robot Control Node đã được khởi tạo')

    def control_loop(self):
        try:
            # Đọc dữ liệu từ STM32
            data = b''
            while len(data) < 7:  # Đọc cho đến khi nhận đủ 7 byte
                byte = self.ser.read(1)
                if not byte:
                    self.get_logger().warn('Timeout khi đọc dữ liệu từ STM32')
                    return
                data += byte
                
                # Kiểm tra byte bắt đầu
                if len(data) == 1 and data[0] != 255:
                    data = b''  # Reset nếu byte đầu không phải 255
                    continue
                
                # Kiểm tra byte truy cập bắt đầu
                if len(data) == 2 and data[1] != 245:
                    data = b''  # Reset nếu byte thứ hai không phải 245
                    continue

            # Kiểm tra byte kết thúc
            if data[-1] != 0:
                self.get_logger().warn('Dữ liệu không hợp lệ: byte kết thúc không phải 0')
                return

            # Giải mã dữ liệu
            goc_B, goc_C, goc_B_prime, goc_C_prime = struct.unpack('BBBB', data[2:6])
            self.get_logger().info(f'Nhận được dữ liệu góc: B={goc_B}, C={goc_C}, B\'={goc_B_prime}, C\'={goc_C_prime}')
            
            # Chuyển đổi góc sang radian và áp dụng công thức
            chan_trai_angles = [-goc_B * (math.pi / 180), (180 - goc_C) * (math.pi / 180)]
            chan_phai_angles = [-goc_B_prime * (math.pi / 180), (180 - goc_C_prime) * (math.pi / 180)]
            
            self.get_logger().info(f'Góc đã chuyển đổi: Chan trái {chan_trai_angles}, Chan phải {chan_phai_angles}')
            
            # Điều khiển chân robot
            self.move_leg('chan_trai', chan_trai_angles)
            self.move_leg('chan_phai', chan_phai_angles)

        except serial.SerialException as e:
            self.get_logger().error(f'Lỗi khi đọc dữ liệu từ STM32: {str(e)}')
        except struct.error as e:
            self.get_logger().error(f'Lỗi khi giải mã dữ liệu: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Lỗi không xác định: {str(e)}')

    def move_leg(self, leg_name, positions):
        client = self.chan_trai_client if leg_name == 'chan_trai' else self.chan_phai_client
        joint_names = ['joint_1', 'joint_2'] if leg_name == 'chan_trai' else ['joint_3', 'joint_4']
        
        goal_msg = FollowJointTrajectory.Goal()
        
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(self.movement_duration), nanosec=int((self.movement_duration % 1) * 1e9))
        
        trajectory.points = [point]
        goal_msg.trajectory = trajectory

        self.get_logger().info(f'Di chuyển {leg_name} đến {positions}')
        future = client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.goal_response_callback(f, leg_name))

    def goal_response_callback(self, future, leg_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'Mục tiêu bị từ chối cho {leg_name}')
            return

        self.get_logger().info(f'Mục tiêu được chấp nhận cho {leg_name}')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.get_result_callback(f, leg_name))

    def get_result_callback(self, future, leg_name):
        result = future.result().result
        self.get_logger().info(f'Kết quả cho {leg_name}: {result.error_code}')

    def __del__(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        robot_control_node = RobotControlNode()
        rclpy.spin(robot_control_node)
    except Exception as e:
        print(f'Lỗi: {str(e)}')
    finally:
        if 'robot_control_node' in locals():
            robot_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()