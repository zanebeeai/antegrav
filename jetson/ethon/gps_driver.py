#!/usr/bin/env python3
"""GPS driver for the Radiolink M10N (u-blox M10) on the Jetson 40-pin UART.

Reads NMEA from /dev/ttyTHS1 @38400 and republishes GGA fixes as
sensor_msgs/NavSatFix on /gps/fix (best-effort), which lap_timer.py consumes.
It also publishes GGA fix quality and RMC/VTG course on /ethon/gps_status for
synchronized data capture. No pynmea2 dependency; supported sentences are
parsed directly while interleaved UBX binary frames are ignored.
"""
import json
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

import serial   # python3-serial

PORT = "/dev/ttyTHS1"
BAUD = 38400
FRAME_ID = "gps"
TOPIC = "/gps/fix"
STATUS_TOPIC = "/ethon/gps_status"


def _dm_to_deg(val, hemi):
    """NMEA ddmm.mmmm + N/S/E/W -> signed decimal degrees, or None."""
    if not val:
        return None
    try:
        f = float(val)
    except ValueError:
        return None
    deg = int(f / 100)
    dd = deg + (f - deg * 100) / 60.0
    if hemi in ("S", "W"):
        dd = -dd
    return dd


class GpsDriver(Node):
    def __init__(self):
        super().__init__("gps_driver")
        log = self.get_logger()
        self.declare_parameter("port", PORT)
        self.declare_parameter("baud", BAUD)
        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        try:
            self._ser = serial.Serial(port, baud, timeout=0.1)
            log.info("GPS open %s @ %d" % (port, baud))
        except (serial.SerialException, OSError) as exc:
            log.error("cannot open GPS %s: %s -- HEADLESS" % (port, exc))
            self._ser = None
        self._pub = self.create_publisher(NavSatFix, TOPIC, qos_profile_sensor_data)
        self._status_pub = self.create_publisher(
            String, STATUS_TOPIC, qos_profile_sensor_data)
        self._buf = bytearray()
        self._had_fix = False
        self._quality = 0
        self._satellites = None
        self._heading = None
        self._latitude = None
        self._longitude = None
        self._fix_timestamp_ns = None
        self._heading_timestamp_ns = None
        self.create_timer(0.1, self._poll)

    def _poll(self):
        if self._ser is None:
            return
        try:
            data = self._ser.read(1024)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning("GPS read failed: %s" % exc)
            return
        if not data:
            return
        self._buf.extend(data)
        while b"\n" in self._buf:
            i = self._buf.index(b"\n")
            line = bytes(self._buf[:i])
            del self._buf[:i + 1]
            self._handle(line.decode("ascii", "ignore").strip())
        if len(self._buf) > 4096:
            del self._buf[:-512]

    def _handle(self, line):
        if len(line) < 7 or line[0] != "$":
            return
        sentence = line[3:6]
        if sentence == "GGA":
            self._handle_gga(line)
        elif sentence in ("RMC", "VTG"):
            self._handle_course(line, sentence)

    def _handle_gga(self, line):
        # any-talker GGA: $GPGGA, $GNGGA, ...
        f = line.split(",")
        if len(f) < 10:
            return
        try:
            quality = int(f[6]) if f[6] else 0
        except ValueError:
            quality = 0
        self._quality = quality
        self._fix_timestamp_ns = time.monotonic_ns()
        try:
            self._satellites = int(f[7]) if f[7] else None
        except ValueError:
            self._satellites = None
        lat = _dm_to_deg(f[2], f[3])
        lon = _dm_to_deg(f[4], f[5])
        self._latitude, self._longitude = lat, lon
        try:
            alt = float(f[9]) if f[9] else 0.0
        except ValueError:
            alt = 0.0
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        msg.status.service = NavSatStatus.SERVICE_GPS
        if quality > 0 and lat is not None and lon is not None and (lat or lon):
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = alt
            if not self._had_fix:
                self._had_fix = True
                self.get_logger().info("GPS FIX: %.6f, %.6f (q=%d sats=%s)"
                                       % (lat, lon, quality, f[7]))
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self._pub.publish(msg)
        self._publish_status()

    def _handle_course(self, line, sentence):
        f = line.split(",")
        # RMC: field 8 is course over ground; VTG: field 1. RMC validity is A.
        if sentence == "RMC" and (len(f) < 9 or f[2] != "A"):
            return
        index = 8 if sentence == "RMC" else 1
        try:
            heading = float(f[index]) if len(f) > index and f[index] else None
        except ValueError:
            heading = None
        if heading is not None:
            self._heading = heading % 360.0
            self._heading_timestamp_ns = time.monotonic_ns()
            self._publish_status()

    def _publish_status(self):
        status = {
            "timestamp_ns": time.monotonic_ns(),
            "latitude": self._latitude,
            "longitude": self._longitude,
            "heading_deg": self._heading,
            "heading_timestamp_ns": self._heading_timestamp_ns,
            "fix_quality": self._quality,
            "fix_timestamp_ns": self._fix_timestamp_ns,
            "satellites": self._satellites,
        }
        self._status_pub.publish(String(data=json.dumps(
            status, separators=(",", ":"))))


def _on_term(signum, _frame):
    raise SystemExit(signum)


def main():
    rclpy.init()
    node = GpsDriver()
    signal.signal(signal.SIGTERM, _on_term)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
