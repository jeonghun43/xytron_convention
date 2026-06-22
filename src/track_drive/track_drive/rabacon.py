import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import XycarMotor


class RabaconDrive():
    def __init__(self):
        self.motor_pub = self.create_publisher(XycarMotor, "/xycar_motor", 10)
        self.scan_sub = self.create_subscription(LaserScan,"/scan",self.scan_callback,10)

        self.motor_msg = XycarMotor()

        self.speed = 10.0
        self.kp = 55.0
        self.max_angle = 100.0

        self.min_dist = 0.05
        self.max_dist = 3.0

        # self.get_logger().info("🚌 Rabacon lidar drive started.")

    def normalize_angle(self, angle_deg):
        """0~360도 각도를 -180~180도로 변환"""
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg < -180:
            angle_deg += 360
        return angle_deg


    def get_range_values(self, scan, deg_min, deg_max):
        values = []

        for i, dist in enumerate(scan.ranges):
            if math.isinf(dist) or math.isnan(dist):
                continue

            if dist < self.min_dist or dist > self.max_dist:
                continue

            if 0.14 <= dist <= 0.17:
                continue

            angle = scan.angle_min + scan.angle_increment * i
            angle_deg = math.degrees(angle)
            angle_deg = self.normalize_angle(angle_deg)

            if deg_min <= angle_deg <= deg_max:
                values.append(dist)

        return values

    def mean_or_none(self, values):
        if len(values) == 0:
            return None
        return float(np.mean(values))

    def drive(self, angle, speed):
        msg = XycarMotor()
        msg.angle = float(angle)
        msg.speed = float(speed)
        self.motor_pub.publish(msg)

    def scan_callback(self, scan):
        left_values = self.get_range_values(scan, 20, 80)
        right_values = self.get_range_values(scan, -80, -20)
        front_values = self.get_range_values(scan, -15, 15)

        left_dist = self.mean_or_none(left_values)
        right_dist = self.mean_or_none(right_values)
        front_dist = self.mean_or_none(front_values)

        angle = 0.0
        speed = self.speed

        if left_dist is not None and right_dist is not None:
            error = left_dist - right_dist
            angle = self.kp * error -30.0

        elif left_dist is not None:
            angle = 55.0

        elif right_dist is not None:
            angle = -85.0

        else:
            angle = -20.0
            speed = 8.0

        if front_dist is not None and front_dist < 0.6:
            speed = 8.0

        angle = max(min(angle, self.max_angle), -self.max_angle)

        self.drive(angle, speed)

#         self.get_logger().info(
#             f"L={left_dist}, R={right_dist}, F={front_dist}, "
#             f"Lcnt={len(left_values)}, Rcnt={len(right_values)}, "
#             f"angle={angle:.1f}, speed={speed:.1f}"
# )


def main(args=None):
    rclpy.init(args=args)
    node = RabaconDrive()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.drive(0.0, 0.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()