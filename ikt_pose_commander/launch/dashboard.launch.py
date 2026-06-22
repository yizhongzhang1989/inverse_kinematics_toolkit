"""Launch the ikt_pose_commander web dashboard (independent of the commander).

The dashboard is a thin HTTP/ROS client: it monitors a running commander's
``~/status`` and drives it via ``~/enable`` / ``~/disable`` / ``~/target_pose``.
The commander runs fine without it. Port + base_frame defaults come from the
toolkit's centralized config (``ikt_pose_commander:`` in
``ikt_common/config/toolkit_defaults.yaml``); CLI args override.

    ros2 launch ikt_pose_commander dashboard.launch.py            # :8180
    ros2 launch ikt_pose_commander dashboard.launch.py \
        commander_ns:=/ikt_pose_commander_left base_frame:=base_link port:=8181
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

_FALLBACKS = {"dashboard_port": 8180, "dashboard_base_frame": "base_link"}


def _defaults():
    try:
        from ikt_common.config_manager import get_config  # type: ignore
        cfg = get_config()
        if cfg.has("ikt_pose_commander"):
            sec = cfg.section("ikt_pose_commander")
            return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
                    f"loaded from {cfg.config_path}")
        return (dict(_FALLBACKS), "FALLBACK (no 'ikt_pose_commander:' section)")
    except Exception as exc:  # noqa: BLE001
        return (dict(_FALLBACKS), f"FALLBACK ({type(exc).__name__}: {exc})")


def generate_launch_description():
    d, source = _defaults()
    args = [
        DeclareLaunchArgument("port", default_value=str(d["dashboard_port"])),
        DeclareLaunchArgument("commander_ns",
                              default_value="/ikt_pose_commander"),
        # Empty => single/default dashboard named ``ikt_pose_commander_dashboard``.
        # Set (e.g. ``left``/``right``) for multi-arm setups so each dashboard
        # gets a UNIQUE ROS node name and they don't collide on one name.
        DeclareLaunchArgument("instance_name", default_value=""),
        DeclareLaunchArgument("base_frame",
                              default_value=str(d["dashboard_base_frame"])),
    ]

    node = Node(
        package="ikt_pose_commander",
        executable="dashboard_node",
        name=PythonExpression(
            ["'ikt_pose_commander_dashboard' + ('_' + '",
             LaunchConfiguration("instance_name"),
             "' if '", LaunchConfiguration("instance_name"), "' else '')"]),
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("port"),
            "commander_ns": LaunchConfiguration("commander_ns"),
            "base_frame": LaunchConfiguration("base_frame"),
        }],
    )

    return LaunchDescription(
        args + [LogInfo(msg=f"[ikt_pose_commander dashboard] config: {source}"),
                node])
