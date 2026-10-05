#!/usr/bin/env python3
"""PS5 (DualSense) -> /cmd_vel for the mecanum robot.

Left stick  : forward/back + strafe left/right
Right stick : rotate (X axis)
Hold R1     : deadman. The robot only moves while held.
Press R3    : turbo while held (press the right stick down)

It publishes ONLY while R1 is held (plus one stop command on release), so it
does not fight with Nav2 on /cmd_vel. Holding R1 = manual takeover.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class Ps5Teleop(Node):
    def __init__(self):
        super().__init__('ps5_teleop')
        # Indices for your DualSense (R1 = button 5). If something is wrong,
        # run `ros2 topic echo /joy` and change these.
        self.declare_parameter('axis_x', 1)           # left stick up/down
        self.declare_parameter('axis_y', 0)           # left stick left/right
        self.declare_parameter('axis_yaw', 3)         # right stick left/right
        self.declare_parameter('deadman_button', 5)   # R1
        self.declare_parameter('turbo_button', 12)    # R3
        self.declare_parameter('scale_linear', 0.4)         # m/s
        self.declare_parameter('scale_linear_turbo', 0.9)   # plugin max is 1.0
        self.declare_parameter('scale_angular', 1.2)        # rad/s
        self.declare_parameter('scale_angular_turbo', 2.0)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('joy_timeout', 0.5)

        self.p = lambda n: self.get_parameter(n).value
        self.pub = self.create_publisher(Twist, self.p('cmd_topic'), 10)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.cmd = Twist()
        self.last_joy = self.get_clock().now()
        self.active = False
        self.sent_stop = True
        self.create_timer(0.05, self.tick)  # 20 Hz
        self.get_logger().info('PS5 teleop ready: hold R1 and use the sticks')

    def dz(self, v):
        return 0.0 if abs(v) < self.p('deadzone') else v

    def on_joy(self, msg: Joy):
        self.last_joy = self.get_clock().now()
        try:
            held = msg.buttons[self.p('deadman_button')] == 1
            turbo = msg.buttons[self.p('turbo_button')] == 1
            ax = self.dz(msg.axes[self.p('axis_x')])
            ay = self.dz(msg.axes[self.p('axis_y')])
            az = self.dz(msg.axes[self.p('axis_yaw')])
        except IndexError:
            self.get_logger().warn('Joy index out of range, fix the params',
                                   throttle_duration_sec=2.0)
            return

        self.active = held
        cmd = Twist()
        if held:
            lin = self.p('scale_linear_turbo' if turbo else 'scale_linear')
            ang = self.p('scale_angular_turbo' if turbo else 'scale_angular')
            cmd.linear.x = ax * lin
            cmd.linear.y = ay * lin
            cmd.angular.z = az * ang
        self.cmd = cmd

    def tick(self):
        age = (self.get_clock().now() - self.last_joy).nanoseconds * 1e-9
        if age > self.p('joy_timeout'):
            self.cmd = Twist()   # controller lost -> stop
            self.active = False
        if self.active:
            self.pub.publish(self.cmd)
            self.sent_stop = False
        elif not self.sent_stop:
            self.pub.publish(Twist())   # one stop command on release
            self.sent_stop = True


def main():
    rclpy.init()
    node = Ps5Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()