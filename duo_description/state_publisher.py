from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.loop_rate = self.create_rate(30)  # 30 Hz

        # Update joint names based on your URDF
        self.joint_state = JointState()
        self.joint_state.name = ['left_wheel_joint', 'right_wheel_joint', 'caster_mount_joint', 'caster_wheel_joint']

        # Set initial positions to zero (assuming these are rotational joints)
        self.joint_state.position = [0.0, 0.0, 0.0, 0.0]

        # Message declarations
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'base_reference'

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.wheel_radius = 0.1
        self.wheel_base = 0.554  # distance between the left and right wheels

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                now = self.get_clock().now()
                self.joint_state.header.stamp = now.to_msg()

                # Update transform (moving in a circle with radius=0.25)
                elapsed_time = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9  # Get the current time in seconds
                dt = 1.0 / 30.0  # assuming a 30 Hz update rate

                # Update the robot's position and orientation
                v = 0.1  # linear velocity in m/s
                omega = 0.1  # angular velocity in rad/s

                self.x += v * cos(self.theta) * dt
                self.y += v * sin(self.theta) * dt

                
                self.theta += omega * dt

                self.odom_trans.header.stamp = now.to_msg()
                self.odom_trans.transform.translation.x = self.x
                self.odom_trans.transform.translation.y = self.y
                self.odom_trans.transform.translation.z = 0.0  # Adjust z-offset if needed
                self.odom_trans.transform.rotation = euler_to_quaternion(0, 0, self.theta)  # roll, pitch, yaw

                # Update joint positions (simulate wheel rotations for circular motion)
                wheel_angle = v / self.wheel_radius * dt
                caster_angle = sin(elapsed_time)

                self.joint_state.position[0] += wheel_angle  # left wheel
                self.joint_state.position[1] += wheel_angle  # right wheel
                self.joint_state.position[2] = caster_angle  # caster mount
                self.joint_state.position[3] = caster_angle  # caster wheel

                # Send the joint state and transform
                self.joint_pub.publish(self.joint_state)
                self.broadcaster.sendTransform(self.odom_trans)

                self.loop_rate.sleep()

        except KeyboardInterrupt:
            pass

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()
