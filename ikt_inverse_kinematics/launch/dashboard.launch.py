"""Launch the optional IK web dashboard (connects to a running ik_node).

The dashboard is a pure UI client of ik_node's ROS API; the solver runs fine
without it. The default port comes from the toolkit's centralized config
(``ikt_inverse_kinematics.dashboard_port``); CLI ``port:=`` overrides it.

Example::

    ros2 launch ikt_inverse_kinematics dashboard.launch.py port:=8160
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_FALLBACKS = {"dashboard_port": 8160, "host": "0.0.0.0"}


def _defaults():
    try:
        from ikt_common.config_manager import get_config  # type: ignore
        cfg = get_config()
        if cfg.has("ikt_inverse_kinematics"):
            sec = cfg.section("ikt_inverse_kinematics")
            return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
                    f"loaded from {cfg.config_path}")
        return (dict(_FALLBACKS), "FALLBACK (no 'ikt_inverse_kinematics:' section)")
    except Exception as exc:  # noqa: BLE001
        return (dict(_FALLBACKS), f"FALLBACK ({type(exc).__name__}: {exc})")


def generate_launch_description() -> LaunchDescription:
    d, source = _defaults()
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value=str(d["dashboard_port"])),
        LogInfo(msg=f"[ikt_inverse_kinematics dashboard] config: {source}"),
        Node(
            package="ikt_inverse_kinematics",
            executable="dashboard_node",
            name="ik_dashboard",
            output="screen",
            parameters=[{"port": LaunchConfiguration("port")}],
        ),
    ])
