"""ROS2 launch file for the perception/planning/health part of the Ethon stack.

Starts perception -> planning pipeline plus the health monitor:

    birdseye_fusion.py        cameras + YOLO -> /ethon/cones, /ethon/obstacles
    cone_corridor_planner.py  /ethon/cones -> /cmd_vel (armed via ROS param)
    health_monitor.py         aggregates liveness -> /ethon/health

ethon_drive.py runs SEPARATELY as its own ethon-drive.service (not bundled
here) so that re-homing steering -- which requires a full ethon_drive
restart -- doesn't also kill/reinit the cameras and break the dashboard's
live view. Both share ROS_DOMAIN_ID=42 / cyclonedds so /cmd_vel still
crosses the process boundary normally.

Each node runs as a plain ``python3 /home/jetson/ethon/<file>`` process
(none of these scripts are installed as ROS packages) and is respawned
5 s after an unexpected exit.  Output goes to the ROS launch log
(~/.ros/log/) so the systemd journal stays readable.

Usage:
    ros2 launch /home/jetson/ethon/ethon_stack.launch.py

Normally invoked by ethon-stack.service, which sets ROS_DOMAIN_ID=42 and
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp before calling ros2 launch.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess

ETHON_DIR = "/home/jetson/ethon"
PYTHON = "python3"
RESPAWN_DELAY_S = 5.0

# (script filename, human-readable process name)
STACK_NODES = (
    ("birdseye_fusion.py", "birdseye_fusion"),
    ("cone_corridor_planner.py", "cone_corridor_planner"),
    ("health_monitor.py", "health_monitor"),
)


def generate_launch_description() -> LaunchDescription:
    """Build the launch description for the perception/planning stack."""
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[PYTHON, f"{ETHON_DIR}/{script}"],
                name=name,
                cwd=ETHON_DIR,
                output="log",
                respawn=True,
                respawn_delay=RESPAWN_DELAY_S,
            )
            for script, name in STACK_NODES
        ]
    )
