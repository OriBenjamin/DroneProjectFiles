#!/usr/bin/env python3
"""
gps_spoof_node.py

Behaviour:
 - subscribes to /fmu/out/vehicle_global_position (PX4)
 - continuously publishes CostumeFakeGps to /fmu/in/costume_fake_gps
 - by default publishes the real GPS (so nothing changes)
 - when user presses 's' + Enter in the launching terminal, the node
   starts publishing spoofed GPS (computed using the chosen mode)
 - 'p' + Enter pauses spoofing (reverts to publishing real GPS again)
 - 'q' + Enter requests node shutdown

Usage:
    python gps_spoof_node.py [mode]
Modes: random_walk | circular | sine (default: circular)
"""

import sys
import time
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import CostumeFakeGps, VehicleGlobalPosition

from gps_spoof_modify_helper import spoof_random_walk, spoof_circular, spoof_sine_along_heading

# --- helper: map mode -> spoof function with validation ---
def get_spoof_function(mode: str):
    spoof_functions = {
        "random_walk": spoof_random_walk,
        "circular": spoof_circular,
        "sine": spoof_sine_along_heading
    }
    if mode not in spoof_functions:
        raise ValueError(f"Invalid mode '{mode}'. Supported modes: {', '.join(spoof_functions.keys())}")
    return spoof_functions[mode]


class GpsSpoofNode(Node):
    def __init__(self, spoof_func):
        super().__init__('gps_spoof_node')

        self.spoof_func = spoof_func

        # PX4-compatible QoS for subscriber
        self.px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher: send CostumeFakeGps to PX4
        # (use default publisher QoS; keep queue depth reasonable)
        self.publisher_ = self.create_publisher(CostumeFakeGps, '/fmu/in/costume_fake_gps', 10)

        # Subscriber: receive real vehicle global position
        self.subscription = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.real_position_callback,
            self.px4_qos
        )

        # publish timer (5 Hz)
        self.timer = self.create_timer(0.2, self._publish_loop)

        # state
        self.latest_real = None      # (lat_deg, lon_deg, alt_m)
        self.latest_spoof = None     # (lat_deg, lon_deg, alt_m) computed from latest_real
        self.last_update_time = None

        # control flags (thread-safe enough for this use)
        self.spoof_active = False
        self.stop_requested = False

        self.get_logger().info("GpsSpoofNode initialized. Spoofing is OFF by default.")
        self.get_logger().info("Press 's' + Enter to start spoofing, 'p' + Enter to pause, 'q' + Enter to quit.")

    def real_position_callback(self, msg: VehicleGlobalPosition):
        """
        Parse incoming VehicleGlobalPosition. PX4 may provide lat/lon as floats
        or as scaled integers. We try both conversions.
        """
        # parse lat/lon/alt into degrees, meters
        lat = lon = alt = None
        try:
            # often floats (degrees / meters)
            lat = float(msg.lat)
            lon = float(msg.lon)
            alt = float(msg.alt)
        except Exception:
            try:
                # scaled ints from PX4: lat/lon in 1e7, alt in mm
                lat = float(msg.lat) / 1e7
                lon = float(msg.lon) / 1e7
                alt = float(msg.alt) / 1e6
            except Exception as e:
                self.get_logger().error(f"Unable to parse VehicleGlobalPosition: {e}")
                return

        self.latest_real = (lat, lon, alt)
        self.last_update_time = self.get_clock().now().to_msg()

        # compute spoofed coordinates — keep them updated even if spoofing not active;
        # they will be used once user toggles spoofing on.
        try:
            new_lat, new_lon, new_alt = self.spoof_func(lat, lon, alt)
            self.latest_spoof = (new_lat, new_lon, new_alt)
        except Exception as e:
            self.get_logger().error(f"Error computing spoofed coords: {e}")
            self.latest_spoof = None

        # info log (coarse)
        self.get_logger().debug(
            f"REAL -> lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f} ; "
            f"computed SPOOF -> lat={self.latest_spoof[0]:.7f} lon={self.latest_spoof[1]:.7f} alt={self.latest_spoof[2]:.2f}"
            if self.latest_spoof else "No spoof computed"
        )

    def _publish_loop(self):
        """
        Called periodically to publish either real or spoofed GPS as CostumeFakeGps.
        If spoofing is active and spoofed coords are available -> publish spoof.
        Otherwise publish the real GPS (so the drone sees the real signal).
        """
        # prefer latest_real if available
        if self.latest_real is None and self.latest_spoof is None:
            # nothing to publish yet
            self.get_logger().debug("No GPS data available yet.")
            return

        if self.spoof_active and self.latest_spoof is not None:
            lat, lon, alt = self.latest_spoof
            source = "SPOOF"
        else:
            # default to publishing real GPS
            lat, lon, alt = self.latest_real if self.latest_real is not None else self.latest_spoof
            source = "REAL"

        # build message
        msg = CostumeFakeGps()
        msg.timestamp = int(time.time() * 1e6)
        msg.latitude_deg = float(lat)
        msg.longitude_deg = float(lon)
        msg.altitude_m = float(alt)

        self.publisher_.publish(msg)
        self.get_logger().debug(f"Published ({source}) GPS: lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f}")

    # helper control methods
    def start_spoofing(self):
        if self.spoof_active:
            self.get_logger().info("Spoofing already active.")
            return
        if self.latest_spoof is None:
            self.get_logger().warning("No spoofed coordinates available yet; waiting for first real message.")
        self.spoof_active = True
        self.get_logger().info("=== SPOOFING STARTED ===")

    def pause_spoofing(self):
        if not self.spoof_active:
            self.get_logger().info("Spoofing already paused.")
            return
        self.spoof_active = False
        self.get_logger().info("=== SPOOFING PAUSED: publishing real GPS again ===")

    def request_stop(self):
        self.stop_requested = True


def keyboard_listener(node: GpsSpoofNode):
    """
    Simple CLI keyboard control. Runs in its own thread and reads lines
    from stdin. Commands:
      s - start spoofing
      p - pause (stop) spoofing
      q - quit node (requests node shutdown)
    Note: uses input() so requires pressing Enter after the key.
    """
    try:
        while rclpy.ok() and not node.stop_requested:
            try:
                cmd = input().strip().lower()
            except EOFError:
                # stdin closed; request shutdown
                node.get_logger().info("Stdin closed; requesting shutdown.")
                node.request_stop()
                break

            if cmd == 's':
                node.start_spoofing()
            elif cmd == 'p':
                node.pause_spoofing()
            elif cmd == 'q':
                node.get_logger().info("Quit command received. Requesting shutdown...")
                node.request_stop()
                break
            elif cmd == '':
                # ignore empty lines
                continue
            else:
                node.get_logger().info("Commands: 's' to start spoofing, 'p' to pause, 'q' to quit.")
    except Exception as e:
        # log and exit thread
        try:
            node.get_logger().error(f"Keyboard listener error: {e}")
        except Exception:
            print(f"Keyboard listener error: {e}")
    finally:
        # ensure stop_requested flagged
        node.request_stop()


def main(argv=None):
    if argv is None:
        argv = sys.argv

    rclpy.init(args=argv)

    # default mode
    mode = "circular"
    if len(argv) > 1:
        # allow calling as: python gps_spoof_node.py circular
        mode = argv[1].lower()

    try:
        spoof_func = get_spoof_function(mode)
    except ValueError as e:
        print(f"[ERROR] {e}")
        print("Usage: gps_spoof_node.py [mode]")
        rclpy.shutdown()
        return

    node = GpsSpoofNode(spoof_func)

    # start keyboard listener thread (daemon so it won't block process exit)
    kb_thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    kb_thread.start()

    # spin loop but also watch for stop_requested flag to do graceful shutdown
    try:
        # keep spinning until ctrl+c or keyboard thread requests stop
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received; shutting down.")
    finally:
        node.get_logger().info("Shutting down node...")
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()
        # wait briefly for keyboard thread to exit (it is daemon so will not block)
        time.sleep(0.1)


if __name__ == '__main__':
    main(sys.argv)
