#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32

import math


class CameraLidarNode(Node):

    def __init__(self):
        super().__init__('camera_lidar_node')

        # -------- Subscribers --------
        self.create_subscription(
            Image,
            '/camera/image_raw',   # فقط برای فعال بودن دوربین
            self.image_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # -------- Publisher --------
        self.distance_pub = self.create_publisher(
            Float32,
            '/distance/front',
            10
        )

        self.get_logger().info("📡 Camera + Front LiDAR node started")

    def image_callback(self, msg):
        """
        فعلاً کاری با تصویر نمی‌کنیم
        فقط برای اینکه دوربین فعال باشه
        """
        pass

    def lidar_callback(self, msg):
        """
        محاسبه فاصله دقیق جلوی ربات (زاویه 0 رادیان)
        """
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        # زاویه جلو = 0 رادیان
        index = int((0.0 - angle_min) / angle_inc)

        if 0 <= index < len(msg.ranges):
            distance = msg.ranges[index]

            if math.isinf(distance) or math.isnan(distance):
                distance = 10.0
        else:
            distance = 10.0

        self.distance_pub.publish(Float32(data=distance))


def main(args=None):
    rclpy.init(args=args)
    node = CameraLidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

