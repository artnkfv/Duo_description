#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.joint_state = JointState()

        # Define joint names
        self.joint_state.name = [
            'left_wheel_joint',
            'right_wheel_joint',
            'caster_mount_joint',
            'caster_wheel_joint'
        ]

        # Initialize joint positions
        self.joint_state.position = [0.0, 0.0, 0.0, 0.0]

        self.start_time = time.time()

    def timer_callback(self):
        current_time = time.time() - self.start_time
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # Simple oscillation for demonstration purposes
        self.joint_state.position[0] = math.sin(current_time)
        self.joint_state.position[1] = math.sin(current_time)
        self.joint_state.position[2] = math.sin(current_time)
        self.joint_state.position[3] = math.sin(current_time)

        self.publisher_.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
