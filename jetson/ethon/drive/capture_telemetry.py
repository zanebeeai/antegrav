"""High-rate, timestamped drive telemetry for the v1 recorder.

Phoenix status signals are cached once and assigned only the update rates the
capture contract needs. The normal 2 Hz health/status topic remains unchanged.
"""

import json
import math
import time

from std_msgs.msg import String

TOPIC_CAPTURE_TELEMETRY = "/ethon/capture/telemetry"
CAPTURE_TELEMETRY_HZ = 100.0


def _value(signal):
    try:
        if hasattr(signal, "status") and not signal.status.is_ok():
            return None
        return float(signal.value)
    except Exception:
        return None


def _latency_s(signal):
    try:
        stamp = signal.timestamp
        getter = getattr(stamp, "get_latency", None)
        latency = getter() if callable(getter) else getattr(stamp, "latency", None)
        return float(latency) if latency is not None else None
    except Exception:
        return None


def _refresh(signals):
    """Copy the newest Phoenix background value/timestamp into each signal."""
    for signal in signals:
        try:
            signal.refresh()
        except Exception:
            # _value() will turn a failed status into None for the recorder.
            pass


def _set_rate(signal, hz, logger, label):
    try:
        result = signal.set_update_frequency(float(hz))
        if hasattr(result, "is_ok") and not result.is_ok():
            logger.warning("capture signal rate rejected for %s: %s" %
                           (label, result.name))
    except Exception as exc:
        logger.warning("capture signal rate unavailable for %s: %s" % (label, exc))


class CaptureTelemetry:
    """Attach a 100 Hz raw-telemetry publisher to an EthonDrive node."""

    def __init__(self, node):
        self.node = node
        self.cfg = node.cfg
        self.log = node.get_logger()
        self.pub = node.create_publisher(String, TOPIC_CAPTURE_TELEMETRY, 100)
        steering = node._steering
        self.fast = {
            "cancoder_position": steering.cancoder.get_absolute_position(),
            "cancoder_velocity": steering.cancoder.get_velocity(),
            "steering_motor_position": steering.steer.get_position(),
            "steering_motor_velocity": steering.steer.get_velocity(),
        }
        self.drive_velocity = [node.drive.get_velocity()] + [
            motor.get_velocity() for motor in node.followers]
        self.diagnostics = {
            "steering_voltage": steering.steer.get_motor_voltage(),
            "steering_current": steering.steer.get_torque_current(),
            "supply_voltage": node.drive.get_supply_voltage(),
        }
        self.drive_currents = [node.drive.get_supply_current()] + [
            motor.get_supply_current() for motor in node.followers]
        self.fault_fields = [motor.get_fault_field() for motor in
                             ([node.drive] + node.followers + [steering.steer])]
        for name, signal in self.fast.items():
            _set_rate(signal, 100, self.log, name)
        for index, signal in enumerate(self.drive_velocity):
            _set_rate(signal, 50, self.log, "drive_%d_velocity" % (index + 1))
        for name, signal in self.diagnostics.items():
            _set_rate(signal, 20, self.log, name)
        for index, signal in enumerate(self.drive_currents):
            _set_rate(signal, 20, self.log, "drive_%d_current" % (index + 1))
        for index, signal in enumerate(self.fault_fields):
            _set_rate(signal, 20, self.log, "device_%d_fault_field" % index)
        self._tick_count = 0
        self._drive_cache = [None, None, None]
        self._diag_cache = {}
        self._fault_cache = "{}"
        node.create_timer(1.0 / CAPTURE_TELEMETRY_HZ, self.publish)

    def publish(self):
        self._tick_count += 1
        now_ns = time.monotonic_ns()
        # Read near the pedal's native ~90 Hz stream rate. The 50 Hz control
        # tick also pumps it; both callbacks share the single-threaded executor.
        self.node._pedal.pump()
        drive_due = self._tick_count % 2 == 0
        diag_due = self._tick_count % 5 == 0
        _refresh(self.fast.values())
        if drive_due:
            _refresh(self.drive_velocity)
            self._drive_cache = [_value(signal) for signal in self.drive_velocity]
        if diag_due:
            _refresh(self.diagnostics.values())
            _refresh(self.drive_currents)
            _refresh(self.fault_fields)
            self._diag_cache = {name: _value(signal)
                                for name, signal in self.diagnostics.items()}
            self._diag_cache["drive_supply_currents"] = [
                _value(signal) for signal in self.drive_currents]
            self._fault_cache = json.dumps(
                {str(motor.device_id): _value(signal)
                 for motor, signal in zip(
                     [self.node.drive] + self.node.followers +
                     [self.node._steering.steer], self.fault_fields)},
                separators=(",", ":"), sort_keys=True)

        raw_cc_pos = _value(self.fast["cancoder_position"])
        raw_cc_vel = _value(self.fast["cancoder_velocity"])
        if raw_cc_pos is not None:
            cc_rot = ((-raw_cc_pos if self.cfg.cancoder_invert else raw_cc_pos)
                      - self.cfg.cancoder_offset_rot)
            cc_pos_rad = cc_rot * 2.0 * math.pi
        else:
            cc_pos_rad = None
        if raw_cc_vel is not None:
            cc_vel_rad_s = (-raw_cc_vel if self.cfg.cancoder_invert
                            else raw_cc_vel) * 2.0 * math.pi
        else:
            cc_vel_rad_s = None

        latencies = [latency for latency in
                     (_latency_s(signal) for signal in self.fast.values())
                     if latency is not None]
        ctre_latency_s = max(latencies) if latencies else None
        values = (self._drive_cache if drive_due else []) + [None, None, None]
        currents = ((self._diag_cache.get("drive_supply_currents") or [])
                    if diag_due else [])
        currents = currents + [None, None, None]
        row = {
            "timestamp_ns": now_ns,
            "ctre_timestamp_ns": (now_ns - int(ctre_latency_s * 1e9)
                                  if ctre_latency_s is not None else None),
            "ctre_latency_ms": (ctre_latency_s * 1000.0
                                if ctre_latency_s is not None else None),
            "cancoder_position_rad": cc_pos_rad,
            "cancoder_velocity_rad_s": cc_vel_rad_s,
            "steering_motor_position": _value(
                self.fast["steering_motor_position"]),
            "steering_motor_velocity": _value(
                self.fast["steering_motor_velocity"]),
            # Rotor rotations, matching the Talon position fields.
            "steering_target": (
                self.node._steering.column_target * self.cfg.steer_belt_ratio
                if self.node._steering.column_target is not None else None),
            "steering_voltage": (self._diag_cache.get("steering_voltage")
                                 if diag_due else None),
            "steering_current": (self._diag_cache.get("steering_current")
                                 if diag_due else None),
            "drive_1_velocity": values[0],
            "drive_2_velocity": values[1],
            "drive_3_velocity": values[2],
            "drive_1_current": currents[0],
            "drive_2_current": currents[1],
            "drive_3_current": currents[2],
            "supply_voltage": (self._diag_cache.get("supply_voltage")
                               if diag_due else None),
            "vehicle_speed_m_s": (self.node._wheel_speed_ms()
                                   if drive_due else None),
            "pedal_fraction": self.node._pedal.frac,
            "pedal_sample_timestamp_ns": self.node._pedal.sample_timestamp_ns,
            "manual_or_auto": "auto" if self.node._armed else "manual",
            "estop": bool(self.node._estop_latched),
            "can_faults": self._fault_cache if diag_due else None,
        }
        self.pub.publish(String(data=json.dumps(row, separators=(",", ":"))))
