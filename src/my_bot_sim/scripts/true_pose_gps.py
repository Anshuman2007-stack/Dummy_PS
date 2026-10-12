#!/usr/bin/env python3
import math, random
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus

LAT0, LON0 = 22.62, 75.69    # same as the world file
R_EARTH = 6378137.0
SIGMA = 0.3                  # metres of GPS noise (lower = cleaner)
RATE = 5.0                   # Hz


class TruePoseGps(Node):
    def __init__(self):
        super().__init__('true_pose_gps')
        self.x = None
        self.y = None
        self.create_subscription(Odometry, '/true_odom', self.odom_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(NavSatFix, '/gnss/fix', qos_profile_sensor_data)
        self.create_timer(1.0 / RATE, self.tick)
        self.get_logger().info("GPS from true pose ready.")

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def tick(self):
        if self.x is None:
            return
        east = self.x + random.gauss(0.0, SIGMA)      # world +X = east
        north = self.y + random.gauss(0.0, SIGMA)     # world +Y = north

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gnss_link'
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = LAT0 + math.degrees(north / R_EARTH)
        fix.longitude = LON0 + math.degrees(east / (R_EARTH * math.cos(math.radians(LAT0))))
        fix.altitude = 0.0
        v = SIGMA * SIGMA
        fix.position_covariance = [v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, 4.0 * v]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub.publish(fix)


def main():
    rclpy.init()
    rclpy.spin(TruePoseGps())


if __name__ == '__main__':
    main()
