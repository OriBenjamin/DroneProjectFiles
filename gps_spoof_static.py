#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import CostumeFakeGps
import time
import sys

class GpsPublisher(Node):
    def __init__(self, latitude, longitude, altitude):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(CostumeFakeGps, '/fmu/in/costume_fake_gps', 10)
        self.timer = self.create_timer(0.2, self.timer_callback)  # 5 Hz
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

    def timer_callback(self):
        msg = CostumeFakeGps()
        msg.timestamp = int(time.time() * 1e6) 
        msg.latitude_deg = self.latitude
        msg.longitude_deg = self.longitude
        msg.altitude_m = self.altitude
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published GPS: lat={self.latitude}, lon={self.longitude}, alt={self.altitude}")

def main():
    # Default values
    latitude = 4.7397969
    longitude = 8.5461574862
    altitude = 0.0011810859441

    if len(sys.argv) == 4:
        try:
            latitude = float(sys.argv[1])
            longitude = float(sys.argv[2])
            altitude = float(sys.argv[3])
        except ValueError:
            print("Error: All parameters must be numbers (float). Using default values.")
    elif len(sys.argv) != 1:
        print("Usage: python3 gps_spoof_static.py [latitude longitude altitude]")
        return

    rclpy.init()
    node = GpsPublisher(latitude, longitude, altitude)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
