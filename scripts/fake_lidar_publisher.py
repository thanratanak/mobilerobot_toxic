#!/usr/bin/env python3
"""Fake 2D lidar for mobilerobot_toxic.

Simulates a 360-degree LaserScan by ray-casting against a simple
rectangular room whose walls are given as parameters. Robot pose is
taken from /odom (published by fake_mobile_base.py / mecanum_fake_controller.py),
and the lidar's mount offset from base_link (from RobotNavigation.urdf's
lidar_joint) is applied so the scan origin matches the real sensor frame.

No physics engine, no real sensor: purely geometric ray-casting against
four walls. This is intended to give SLAM/Nav2 something real to consume
before actual lidar hardware exists.
"""
import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class FakeLidarPublisher(Node):

    def __init__(self):
        super().__init__('fake_lidar_publisher')

        # Room walls (meters), axis-aligned box in the odom frame.
        self.declare_parameter('room_min_x', -5.0)
        self.declare_parameter('room_max_x', 5.0)
        self.declare_parameter('room_min_y', -5.0)
        self.declare_parameter('room_max_y', 5.0)

        # Lidar mount offset from base_link, matching lidar_joint in
        # RobotNavigation.urdf (xyz="0.13308 0 0.16241").
        self.declare_parameter('lidar_offset_x', 0.13308)
        self.declare_parameter('lidar_offset_y', 0.0)

        # Standard 2D lidar-ish specs.
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('range_max', 8.0)
        self.declare_parameter('angle_increment_deg', 1.0)  # 360 pts/rev
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('noise_stddev', 0.0)  # meters, 0 = no noise
        self.declare_parameter('frame_id', 'laser_frame')

        self.room_min_x = float(self.get_parameter('room_min_x').value)
        self.room_max_x = float(self.get_parameter('room_max_x').value)
        self.room_min_y = float(self.get_parameter('room_min_y').value)
        self.room_max_y = float(self.get_parameter('room_max_y').value)
        self.off_x = float(self.get_parameter('lidar_offset_x').value)
        self.off_y = float(self.get_parameter('lidar_offset_y').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.angle_increment = math.radians(
            float(self.get_parameter('angle_increment_deg').value))
        self.rate = float(self.get_parameter('publish_rate').value)
        self.noise_stddev = float(self.get_parameter('noise_stddev').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.have_odom = False

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(1.0 / self.rate, self.publish_scan)

        self.get_logger().info(
            'Fake lidar started. Room: x[%.2f, %.2f] y[%.2f, %.2f]' % (
                self.room_min_x, self.room_max_x,
                self.room_min_y, self.room_max_y))

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # yaw from quaternion (planar robot, only z/w matter)
        self.robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.have_odom = True

    def ray_to_walls(self, ox, oy, angle):
        """Distance from (ox, oy) along angle to the nearest wall, or
        range_max if it would exceed range_max."""
        dx = math.cos(angle)
        dy = math.sin(angle)
        best = self.range_max

        # Vertical walls (x = room_min_x / room_max_x)
        for wall_x in (self.room_min_x, self.room_max_x):
            if abs(dx) > 1e-9:
                t = (wall_x - ox) / dx
                if t > 0:
                    y_hit = oy + t * dy
                    if self.room_min_y <= y_hit <= self.room_max_y:
                        best = min(best, t)

        # Horizontal walls (y = room_min_y / room_max_y)
        for wall_y in (self.room_min_y, self.room_max_y):
            if abs(dy) > 1e-9:
                t = (wall_y - oy) / dy
                if t > 0:
                    x_hit = ox + t * dx
                    if self.room_min_x <= x_hit <= self.room_max_x:
                        best = min(best, t)

        return best

    def publish_scan(self):
        if not self.have_odom:
            return

        # World-frame position of the lidar (base_link position + rotated
        # sensor offset).
        c = math.cos(self.robot_yaw)
        s = math.sin(self.robot_yaw)
        lidar_x = self.robot_x + (c * self.off_x - s * self.off_y)
        lidar_y = self.robot_y + (s * self.off_x + c * self.off_y)

        angle_min = -math.pi
        angle_max = math.pi
        n = int(round((angle_max - angle_min) / self.angle_increment))

        ranges = []
        noise = self.noise_stddev
        for i in range(n):
            angle = angle_min + i * self.angle_increment
            world_angle = self.robot_yaw + angle
            r = self.ray_to_walls(lidar_x, lidar_y, world_angle)
            if noise > 0.0:
                r += random.gauss(0.0, noise)
            r = max(self.range_min, min(self.range_max, r))
            ranges.append(r)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.rate
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()