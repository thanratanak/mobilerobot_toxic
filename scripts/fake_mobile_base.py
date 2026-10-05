#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class FakeMobileBase(Node):
    """Simple kinematic fake controller for a 4-wheel mecanum robot.

    It listens to /cmd_vel, integrates planar odometry, publishes /odom and
    the odom -> base_footprint TF, and publishes wheel joint positions.
    This is intentionally a fake/kinematic simulator: no Gazebo physics.
    """

    def __init__(self):
        super().__init__('fake_mobile_base')

        self.declare_parameter('wheel_radius', 0.0645)
        self.declare_parameter('wheelbase_x', 0.195)
        self.declare_parameter('wheelbase_y', 0.272)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('cmd_timeout', 0.5)

        self.r = float(self.get_parameter('wheel_radius').value)
        self.lx = float(self.get_parameter('wheelbase_x').value)
        self.ly = float(self.get_parameter('wheelbase_y').value)
        self.rate = float(self.get_parameter('publish_rate').value)
        self.timeout = float(self.get_parameter('cmd_timeout').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.wheel_pos = [0.0, 0.0, 0.0, 0.0]  # FL, BL, FR, BR

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_cmd = self.get_clock().now()
        self.last_update = self.get_clock().now()

        self.joint_names = [
            'wheel_fl_joint',
            'wheel_bl_joint',
            'wheel_fr_joint',
            'wheel_br_joint',
        ]

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / self.rate, self.update)

        self.get_logger().info('Fake mecanum base started. Publish Twist to /cmd_vel.')

    def cmd_callback(self, msg: Twist):
        self.vx = float(msg.linear.x)
        self.vy = float(msg.linear.y)
        self.wz = float(msg.angular.z)
        self.last_cmd = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds * 1e-9
        self.last_update = now
        if dt <= 0.0 or dt > 0.2:
            dt = 1.0 / self.rate

        # Stop automatically if /cmd_vel has not been refreshed.
        age = (now - self.last_cmd).nanoseconds * 1e-9
        vx = self.vx if age <= self.timeout else 0.0
        vy = self.vy if age <= self.timeout else 0.0
        wz = self.wz if age <= self.timeout else 0.0

        # Body velocity -> world velocity.
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        self.x += (c * vx - s * vy) * dt
        self.y += (s * vx + c * vy) * dt
        self.yaw = self.normalize_angle(self.yaw + wz * dt)

        # Mecanum wheel angular velocities.
        k = self.lx + self.ly
        fl = (vx - vy - k * wz) / self.r
        bl = (vx + vy - k * wz) / self.r
        fr = (vx + vy + k * wz) / self.r
        br = (vx - vy + k * wz) / self.r
        wheel_vel = [fl, bl, fr, br]
        for i in range(4):
            self.wheel_pos[i] += wheel_vel[i] * dt

        self.publish_joint_states(now)
        self.publish_odom(now, vx, vy, wz)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def quaternion_from_yaw(yaw):
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    def publish_joint_states(self, stamp):
        msg = JointState()
        msg.header.stamp = stamp.to_msg()
        msg.name = self.joint_names
        msg.position = self.wheel_pos
        self.joint_pub.publish(msg)

    def publish_odom(self, stamp, vx, vy, wz):
        qx, qy, qz, qw = self.quaternion_from_yaw(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = FakeMobileBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
