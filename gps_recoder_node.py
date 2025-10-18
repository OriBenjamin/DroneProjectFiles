#!/usr/bin/env python3

import os
import csv
import time
import argparse
from datetime import datetime, UTC

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleGlobalPosition, CostumeFakeGps


def current_utc_str(fmt="iso"):
    """
    Returns the current UTC timestamp as a string.
    - fmt="iso":    2025-10-18T14:52:33.123456Z
    - fmt="compact": 20251018_145233
    """
    now = datetime.now(UTC)
    if fmt == "iso":
        return now.isoformat(timespec='microseconds') + "Z"
    elif fmt == "compact":
        return now.strftime("%Y%m%d_%H%M%S")
    else:
        raise ValueError("Invalid format: use 'iso' or 'compact'")


class GpsRecorderNode(Node):
    def __init__(self, out_dir: str, basename: str):
        super().__init__('gps_recorder_node')

        self.out_dir = out_dir
        os.makedirs(self.out_dir, exist_ok=True)

        timestamp = current_utc_str(fmt="compact")
        self.filename = os.path.join(self.out_dir, f"{basename}_{timestamp}.csv")

        self.get_logger().info(f"Recording GPS data to {self.filename}")
        self.csv_file = open(self.filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write CSV header
        self.csv_writer.writerow([
            "ts_utc_iso",
            "real_lat", "real_lon", "real_alt_m",
            "spoof_lat", "spoof_lon", "spoof_alt_m"
        ])

        self.real_gps = None
        self.spoof_gps = None

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        px4_qos = QoSProfile(
	        reliability=ReliabilityPolicy.BEST_EFFORT,
	        history=HistoryPolicy.KEEP_LAST,
	        depth=1
        )

        self.create_subscription(
	        VehicleGlobalPosition,
	        '/fmu/out/vehicle_global_position',
	        self._real_cb,
	        px4_qos
        )


        self.create_subscription(
            CostumeFakeGps,
            '/fmu/in/costume_fake_gps',
            self._spoof_cb,
            10
        )

        self.create_timer(0.2, self._write_row)

    def _real_cb(self, msg: VehicleGlobalPosition):
        try:
            self.real_gps = (
                float(msg.lat) / 1e7,
                float(msg.lon) / 1e7,
                float(msg.alt) / 1000.0
            )
        except Exception as e:
            self.get_logger().error(f"Failed to parse real GPS: {e}")

    def _spoof_cb(self, msg: CostumeFakeGps):
        self.spoof_gps = (
            float(msg.latitude_deg),
            float(msg.longitude_deg),
            float(msg.altitude_m)
        )

    def _write_row(self):
        ts_utc = current_utc_str(fmt="iso")
        real = self.real_gps or (None, None, None)
        spoof = self.spoof_gps or (None, None, None)
        self.csv_writer.writerow([ts_utc] + list(real) + list(spoof))
        self.csv_file.flush()


def main():
    parser = argparse.ArgumentParser(description="Record real and spoofed GPS data from PX4 ROS 2 topics.")
    parser.add_argument("--out-dir", type=str, default="gps_logs", help="Output directory for CSV files")
    parser.add_argument("--basename", type=str, default="gps_record", help="Base name for CSV file")
    parser.add_argument("--duration", type=int, default=None, help="Optional max duration (seconds) to record")
    args = parser.parse_args()

    rclpy.init()

    node = GpsRecorderNode(out_dir=args.out_dir, basename=args.basename)

    if args.duration:
        end_time = time.time() + args.duration
        try:
            while rclpy.ok() and time.time() < end_time:
                rclpy.spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
    else:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
