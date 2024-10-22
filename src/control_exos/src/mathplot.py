#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class SimplifiedSerialController(Node):
    def __init__(self):
        super().__init__('simplified_serial_controller')
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.angles = {'B': [], 'C': [], 'B\'': [], 'C\'': []}
        self.max_points = 100
        
        self.fig, self.ax = plt.subplots()
        self.lines = {angle: self.ax.plot([], [], label=angle)[0] for angle in self.angles}
        self.ax.legend()
        self.ax.set_xlim(0, self.max_points)
        self.ax.set_ylim(-180, 180)
        self.ax.set_title('Góc B, C, B\', C\'')
        self.ax.set_xlabel('Thời gian')
        self.ax.set_ylabel('Góc (độ)')
        
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=True)

    def joint_state_callback(self, msg):
        joint_angles = {name: math.degrees(position) for name, position in zip(msg.name, msg.position)}
        
        self.angles['B'].append(-joint_angles.get('joint_1', 0))
        self.angles['C'].append(180 - joint_angles.get('joint_2', 0))
        self.angles['B\''].append(-joint_angles.get('joint_3', 0))
        self.angles['C\''].append(180 - joint_angles.get('joint_4', 0))
        
        for angle_list in self.angles.values():
            if len(angle_list) > self.max_points:
                angle_list.pop(0)

    def update_plot(self, frame):
        for angle, line in self.lines.items():
            line.set_data(range(len(self.angles[angle])), self.angles[angle])
        return self.lines.values()

def main(args=None):
    rclpy.init(args=args)
    controller = SimplifiedSerialController()
    plt.show(block=False)
    rclpy.spin(controller)
    rclpy.shutdown()
    plt.close()

if __name__ == '__main__':
    main()