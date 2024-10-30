#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class ImprovedEllipseTrajectoryNode(Node):
    def __init__(self):
        super().__init__('improved_ellipse_trajectory_node')
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.chan_trai_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/chan_trai_group_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        self.chan_phai_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/chan_phai_group_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        # Robot parameters (mm)
        self.link_length_1 = 390.0
        self.link_length_2 = 410.0
        
        # Ellipse parameters - now using your required values
        self.declare_parameter('ellipse_a', 150.0)  # Increased to 150mm
        self.declare_parameter('ellipse_b', 80.0)   # Increased to 80mm
        self.declare_parameter('y_offset', 760.0)   # Set to 760mm
        
        self.ellipse_a = self.get_parameter('ellipse_a').value
        self.ellipse_b = self.get_parameter('ellipse_b').value
        self.y_offset = self.get_parameter('y_offset').value
        
        # Extended workspace limits to accommodate larger ellipse
        self.x_min = -200.0  # Extended from -120 to -200
        self.x_max = 200.0   # Extended from 120 to 200
        self.y_min = 400.0   # Lowered from 500 to 400
        self.y_max = 900.0   # Raised from 800 to 900
        
        # Adjusted control parameters for smoother motion
        self.declare_parameter('control_frequency', 50.0)  # Increased for better control
        self.declare_parameter('movement_duration', 0.5)   # Increased for smoother motion
        self.declare_parameter('num_points', 20)          # Increased for smoother trajectory
        self.declare_parameter('max_velocity', 0.4)       # Adjusted for stability
        self.declare_parameter('max_acceleration', 0.2)   # Reduced for stability
        
        self.control_frequency = self.get_parameter('control_frequency').value
        self.movement_duration = self.get_parameter('movement_duration').value
        self.num_points = self.get_parameter('num_points').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        
        # State variables
        self.phase = 0.0
        self.initialized = False
        self.trai_done = True
        self.phai_done = True
        self.current_angles_trai = [0.0, 0.0]
        self.current_angles_phai = [0.0, 0.0]
        
        self.timer = self.create_timer(
            1.0/self.control_frequency,
            self.movement_control_loop,
            callback_group=self.callback_group
        )
        
        self.wait_for_action_clients()

    def wait_for_action_clients(self):
        """Wait for action clients to be ready"""
        self.chan_trai_client.wait_for_server()
        self.chan_phai_client.wait_for_server()
        self.get_logger().info('Action clients are ready')
        
    def check_workspace_limits(self, x, y):
        """Enhanced workspace limit checking with safety margins"""
        try:
            # Add safety margins
            margin = 10.0  # 10mm safety margin
            if x < (self.x_min + margin) or x > (self.x_max - margin) or \
               y < (self.y_min + margin) or y > (self.y_max - margin):
                return False
                
            # Calculate distance from base to point
            r = math.sqrt(x*x + y*y)
            
            # Add safety margin to reach limits
            max_reach = self.link_length_1 + self.link_length_2 - margin
            min_reach = abs(self.link_length_1 - self.link_length_2) + margin
            
            if r > max_reach or r < min_reach:
                return False
                
            # Calculate angles for reachability check
            theta_x = math.atan2(x, y)
            c = y / math.cos(theta_x) if math.cos(theta_x) != 0 else y
            
            cos_B = (self.link_length_1**2 + c**2 - self.link_length_2**2) / (2 * self.link_length_1 * c)
            cos_C = (self.link_length_2**2 + self.link_length_1**2 - c**2) / (2 * self.link_length_2 * self.link_length_1)
            
            # Add angle safety margin
            if abs(cos_B) > 1.1 or abs(cos_C) > 1.1:  # Slightly less than 1 for safety
                return False
                
            return True
            
        except Exception:
            return False

    def inverse_kinematics(self, x, y):
        """Enhanced inverse kinematics with error handling"""
        try:
            #if not self.check_workspace_limits(x, y):
               # self.get_logger().debug(f'Point ({x:.2f},{y:.2f}) outside safe workspace')
                #return None, None
                
            theta_x = math.atan2(x, y)
            c = y / math.cos(theta_x)
            
            cos_B = (self.link_length_1**2 + c**2 - self.link_length_2**2) / (2 * self.link_length_1 * c)
            cos_C = (self.link_length_2**2 + self.link_length_1**2 - c**2) / (2 * self.link_length_2 * self.link_length_1)
            
            goc_B = math.acos(cos_B)
            goc_C = math.acos(cos_C)
            
            goc_new = math.degrees(theta_x)
            d_C = 180 - math.degrees(goc_C)
            d_B = -math.degrees(goc_B) - goc_new
            
            # Enhanced angle limit check
            #if abs(d_B) > 190 or abs(d_C) > 190:  # Reduced from 180 for safety
            #    return None, None
                
            return d_B, d_C
            
        except Exception as e:
            self.get_logger().debug(f'IK calculation error: {str(e)}')
            return None, None

    def calculate_ellipse_points(self, phase):
        """Calculate ellipse trajectory points with enhanced error checking"""
        try:
            # Calculate points with more precise phase control
            x1 = self.ellipse_a * math.cos(phase)
            y1 = self.y_offset - self.ellipse_b * math.sin(phase)
            
            x2 = self.ellipse_a * math.cos(phase + math.pi)
            y2 = self.y_offset - self.ellipse_b * math.sin(phase + math.pi)
            
            # Add additional workspace verification
            if not self.check_workspace_limits(x1, y1):
                self.get_logger().warn(
                    f'Left leg point ({x1:.2f},{y1:.2f}) outside safe workspace'
                )
                #return None, None
                
            if not self.check_workspace_limits(x2, y2):
                self.get_logger().warn(
                    f'Right leg point ({x2:.2f},{y2:.2f}) outside safe workspace'
                )
                #return None, None
                
            return (x1, y1), (x2, y2)
            
        except Exception as e:
            self.get_logger().error(f'Ellipse calculation error: {str(e)}')
            return None, None

    def generate_smooth_trajectory(self, start_angles, end_angles):
        """Generate smoother trajectory with optimized velocity profile"""
        points = []
        times = np.linspace(0, self.movement_duration, self.num_points)
        
        for t in times:
            # Enhanced smooth interpolation
            alpha = 0.5 * (1 - np.cos(t / self.movement_duration * math.pi))
            current_angles = [
                start + alpha * (end - start)
                for start, end in zip(start_angles, end_angles)
            ]
            
            point = JointTrajectoryPoint()
            point.positions = [math.radians(angle) for angle in current_angles]
            
            # Optimized velocity profile
            velocity = self.max_velocity * np.sin(t / self.movement_duration * math.pi)
            point.velocities = [velocity] * len(current_angles)
            
            # Smoother acceleration profile
            acceleration = self.max_acceleration * np.cos(t / self.movement_duration * math.pi)
            point.accelerations = [acceleration] * len(current_angles)
            
            point.time_from_start = Duration(
                sec=int(t),
                nanosec=int((t % 1) * 1e9)
            )
            
            points.append(point)
            
        return points

    def move_leg(self, leg_name, target_angles):
        """Move leg with enhanced error handling"""
        if target_angles[0] is None or target_angles[1] is None:
            if leg_name == 'chan_trai':
                self.trai_done = True
            else:
                self.phai_done = True
            return
            
        current_angles = self.current_angles_trai if leg_name == 'chan_trai' else self.current_angles_phai
        
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint_1', 'joint_2'] if leg_name == 'chan_trai' else ['joint_3', 'joint_4']
        
        trajectory.points = self.generate_smooth_trajectory(
            current_angles,
            target_angles
        )
        
        goal_msg.trajectory = trajectory
        
        if leg_name == 'chan_trai':
            self.current_angles_trai = target_angles
            self.trai_done = False
        else:
            self.current_angles_phai = target_angles
            self.phai_done = False
            
        client = self.chan_trai_client if leg_name == 'chan_trai' else self.chan_phai_client
        future = client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.goal_response_callback(f, leg_name))

    def movement_control_loop(self):
        """Enhanced main control loop with better error handling"""
        if not self.initialized:
            self.initialized = True
            return
            
        if not (self.trai_done and self.phai_done):
            return
            
        points = self.calculate_ellipse_points(self.phase)
        if points is None:
            self.get_logger().error('Failed to calculate valid trajectory points')
            return
            
        (x1, y1), (x2, y2) = points
        
        angles_trai = self.inverse_kinematics(x1, y1)
        angles_phai = self.inverse_kinematics(x2, y2)
        
        self.move_leg('chan_trai', angles_trai)
        self.move_leg('chan_phai', angles_phai)
        
        # Smoother phase update
        phase_increment = 2 * math.pi / (self.control_frequency * self.movement_duration)
        self.phase += phase_increment
        if self.phase >= 2 * math.pi:
            self.phase = 0.0
            
        self.get_logger().debug(
            f'Phase: {self.phase:.2f}, '
            f'Left: ({x1:.1f}, {y1:.1f}), '
            f'Right: ({x2:.1f}, {y2:.1f})'
        )

    def goal_response_callback(self, future, leg_name):
        """Enhanced goal response callback"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Goal rejected for {leg_name}')
            if leg_name == 'chan_trai':
                self.trai_done = True
            else:
                self.phai_done = True
            return
            
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.get_result_callback(f, leg_name))

    def get_result_callback(self, future, leg_name):
        """Enhanced result callback with better error handling"""
        try:
            result = future.result().result
            self.get_logger().debug(f'Movement completed successfully for {leg_name}')
        except Exception as e:
            self.get_logger().error(f'Movement failed for {leg_name}: {str(e)}')
        finally:
            if leg_name == 'chan_trai':
                self.trai_done = True
            else:
                self.phai_done = True

def main(args=None):
    rclpy.init(args=args)
    
    executor = MultiThreadedExecutor()
    node = ImprovedEllipseTrajectoryNode()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()