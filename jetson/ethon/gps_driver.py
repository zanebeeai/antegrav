#!/usr/bin/env python3
"""GPS driver for the Radiolink M10N (u-blox M10) on the Jetson 40-pin UART.

Reads NMEA from /dev/ttyTHS1 @38400 and republishes GGA fixes as
sensor_msgs/NavSatFix on /gps/fix (best-effort), which lap_timer.py consumes.
No pynmea2 dependency -- GGA is parsed directly. The M10 interleaves UBX binary
frames with NMEA; any line that is not a $..GGA sentence is ignored.
"""
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus

import serial   # python3-serial

PORT = "/dev/ttyTHS1"
BAUD = 38400
FRAME_ID = "gps"
TOPIC = "/gps/fix"


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
        self._buf = bytearray()
        self._had_fix = False
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
        # any-talker GGA: $GPGGA, $GNGGA, ...
        if len(line) < 7 or line[0] != "$" or line[3:6] != "GGA":
            return
        f = line.split(",")
        if len(f) < 10:
            return
        try:
            quality = int(f[6]) if f[6] else 0
        except ValueError:
            quality = 0
        lat = _dm_to_deg(f[2], f[3])
        lon = _dm_to_deg(f[4], f[5])
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
