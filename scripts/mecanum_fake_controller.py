#!/usr/bin/env python3
"""Kinematic mecanum controller for mobilerobot_toxic.

/cmd_vel -> mecanum wheel speeds -> /joint_states + fake /odom + odom TF.
No Gazebo or physics engine is required.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class MecanumFakeController(Node):
    def __init__(self):
        super().__init__('mecanum_fake_controller')

        self.declare_parameter('wheel_radius', 0.0644)
        self.declare_parameter('wheelbase_x', 0.195)
        self.declare_parameter('wheelbase_y', 0.2729)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.r = float(self.get_parameter('wheel_radius').value)
        self.lx = float(self.get_parameter('wheelbase_x').value)
        self.ly = float(self.get_parameter('wheelbase_y').value)
        self.rate = float(self.get_parameter('publish_rate').value)
        self.timeout = float(self.get_parameter('cmd_timeout').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.wheel_pos = [0.0] * 4
        self.last_cmd = self.get_clock().now()
        self.last_update = self.get_clock().now()

        # Match RobotNavigation.urdf exactly.
        self.joint_names = [
            'wheel_fl_joint',
            'wheel_bl_joint',
            'wheel_fr_joint',
            'wheel_br_joint',
        ]

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.path_pub = self.create_publisher(Path, '/odom_path', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.path = Path()
        self.path.header.frame_id = self.odom_frame
        self.timer = self.create_timer(1.0 / self.rate, self.update)

        self.get_logger().info('Mecanum fake controller started')
        self.get_logger().info('Input: /cmd_vel')
        self.get_logger().info('Output: /joint_states, /odom, odom -> base_footprint TF')

    def cmd_callback(self, msg: Twist):
        self.vx = float(msg.linear.x)
        self.vy = float(msg.linear.y)
        self.wz = float(msg.angular.z)
        self.last_cmd = self.get_clock().now()

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def yaw_quaternion(yaw):
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds * 1e-9
        self.last_update = now
        if dt <= 0.0 or dt > 0.2:
            dt = 1.0 / self.rate

        # Safety stop when teleop stops publishing /cmd_vel.
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

        # Standard X mecanum inverse kinematics.
        k = self.lx + self.ly
        fl = (vx - vy - k * wz) / self.r
        bl = (vx + vy - k * wz) / self.r
        fr = (vx + vy + k * wz) / self.r
        br = (vx - vy + k * wz) / self.r

        for i, speed in enumerate((fl, bl, fr, br)):
            self.wheel_pos[i] += speed * dt

        self.publish_joint_states(now)
        self.publish_odom(now, vx, vy, wz)

    def publish_joint_states(self, stamp):
        msg = JointState()
        msg.header.stamp = stamp.to_msg()
        msg.name = self.joint_names
        msg.position = self.wheel_pos
        self.joint_pub.publish(msg)

    def publish_odom(self, stamp, vx, vy, wz):
        qx, qy, qz, qw = self.yaw_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        # Simple fake covariance: small but non-zero.
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.02
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp.to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        pose = geometry_pose_stamped(stamp, self.odom_frame, self.x, self.y, qz, qw)
        self.path.header.stamp = stamp.to_msg()
        self.path.poses.append(pose)
        # Keep RViz responsive during long runs.
        if len(self.path.poses) > 2000:
            self.path.poses = self.path.poses[-2000:]
        self.path_pub.publish(self.path)


def geometry_pose_stamped(stamp, frame_id, x, y, qz, qw):
    from geometry_msgs.msg import PoseStamped
    msg = PoseStamped()
    msg.header.stamp = stamp.to_msg()
    msg.header.frame_id = frame_id
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.w = qw
    msg.pose.orientation.z = qz
    return msg


def main(args=None):
    rclpy.init(args=args)
    node = MecanumFakeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
