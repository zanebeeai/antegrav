"""CAN bus resolution + Phoenix6 device-config/fault helpers.

This car has NO CANivore — the 4 Krakens hang off the Jetson's native CAN
controller (mttcan) through an SN65HVD230, i.e. SocketCAN 'can0'. Probing a
'canivore' bus is not merely useless: constructing a device on a phoenix6
bus name registers it with the native runtime, which then retries the
connection FOREVER and logs "CANbus Failed to Connect: canivore" every ~3 s
for the life of the process. Keep the fallback list to buses that can exist.
"""
import os
import time

from phoenix6.hardware import TalonFX

CAN_BUS = "can0"
CAN_BUS_FALLBACKS = ["can0"]
CAN_PROBE_TIMEOUT_S = 0.5
CONFIG_APPLY_RETRIES = 3

# Per-fault getters probed defensively (phoenix6 API drifts across versions;
# missing getters are simply skipped).
FAULT_GETTERS = (
    "get_fault_hardware",
    "get_fault_device_temp",
    "get_fault_proc_temp",
    "get_fault_undervoltage",
    "get_fault_over_supply_v",
    "get_fault_unstable_supply_v",
    "get_fault_bridge_brownout",
    "get_fault_boot_during_enable",
    "get_fault_unlicensed_feature_in_use",
    "get_fault_forward_soft_limit",
    "get_fault_reverse_soft_limit",
    "get_fault_stator_curr_limit",
    "get_fault_supply_curr_limit",
)


def _bus_responds(name: str, probe_device_id: int) -> bool:
    """Best-effort liveness probe of a CAN bus name.

    A SocketCAN name that has no matching netdev is rejected WITHOUT touching
    phoenix6: constructing a device on a bus registers it with the native
    runtime, which then retries that connection for the life of the process
    and spams the log. Never probe a bus that cannot exist.
    """
    if name.startswith(("can", "vcan")) and not os.path.isdir(
            "/sys/class/net/%s" % name):
        return False
    # 1) CANBus API where available (newer phoenix6 builds).
    try:
        from phoenix6 import CANBus
        up = getattr(CANBus(name), "is_network_up")
        if callable(up):
            up = up()
        if isinstance(up, bool):
            return up
    except Exception:
        pass
    # 2) Fall back: read the version signal of the drive master on this bus.
    try:
        sig = TalonFX(probe_device_id, name).get_version()
        sig.wait_for_update(CAN_PROBE_TIMEOUT_S)
        return sig.status.is_ok()
    except Exception:
        return False


def resolve_can_bus(preferred: str, probe_device_id: int, logger) -> str:
    """Return the first responding bus; fall back to `preferred` if none do."""
    candidates = [preferred] + [b for b in CAN_BUS_FALLBACKS if b != preferred]
    for name in candidates:
        if _bus_responds(name, probe_device_id):
            if name != preferred:
                logger.warning(
                    f"CAN bus '{preferred}' not responding — "
                    f"falling back to '{name}'")
            else:
                logger.info(f"CAN bus '{name}' is up")
            return name
    logger.error(
        f"no CAN bus responded (tried {candidates}) — proceeding with "
        f"'{preferred}'; devices will be unreachable until the bus appears. "
        "Check `ip -details link show can0` (should be UP, classic 1 Mbps, "
        "ERROR-ACTIVE), motor power, and `can_bus` in vehicle.yaml. If the "
        "link is UP but silent, the mttcan control-mode flags are sticky — "
        "re-up it explicitly with `fd off listen-only off`.")
    return preferred


def apply_device_config(device, dev_cfg, label: str, logger) -> bool:
    """Apply a device configuration with retries. Returns success."""
    for attempt in range(1, CONFIG_APPLY_RETRIES + 1):
        try:
            status = device.configurator.apply(dev_cfg)
            if status.is_ok():
                return True
            logger.warning(
                f"config apply to {label}, attempt {attempt}: {status.name}")
        except Exception as exc:
            logger.warning(f"config apply to {label}, attempt {attempt}: {exc}")
        time.sleep(0.1)
    logger.error(
        f"FAILED to apply config to {label} after {CONFIG_APPLY_RETRIES} attempts")
    return False


def read_faults(device) -> list:
    """Active fault names for a device, defensively across phoenix6 versions."""
    names, readable = [], False
    for getter in FAULT_GETTERS:
        fn = getattr(device, getter, None)
        if fn is None:
            continue
        try:
            sig = fn()
            if not sig.status.is_ok():
                continue
            readable = True
            if bool(sig.value):
                names.append(getter[len("get_fault_"):])
        except Exception:
            continue
    if not readable:
        # Per-fault getters unusable — try the raw bitfield instead.
        try:
            sig = device.get_fault_field()
            if sig.status.is_ok():
                raw = int(sig.value)
                return [f"raw:0x{raw:X}"] if raw else []
        except Exception:
            pass
        return ["unavailable"]
    return names


def signal_value(device, getter_name: str):
    """Read a status-signal value defensively; None on any failure."""
    try:
        sig = getattr(device, getter_name)()
        if hasattr(sig, "status") and not sig.status.is_ok():
            return None
        return float(sig.value)
    except Exception:
        return None
