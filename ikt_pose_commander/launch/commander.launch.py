"""Launch one ikt_pose_commander instance.

Defaults control the RIGHT arm via its JointTrajectoryController. Override for
the left arm (or for FPC streaming) on the command line, e.g.::

    ros2 launch ikt_pose_commander commander.launch.py \
        instance_name:=left \
        controlled_frame:=left_arm_Link7 \
        jtc_controller:=left_arm_joint_trajectory_controller \
        fpc_controller:=left_arm_forward_position_controller \
        joints:="['left_arm_joint1','left_arm_joint2','left_arm_joint3','left_arm_joint4','left_arm_joint5','left_arm_joint6','left_arm_joint7']"

The node starts DISABLED; call its ``~/enable`` service to allow motion.

Pass ``dashboard_port:=<port>`` to also bring up the web dashboard wired to this
commander instance (omit it / leave empty to run headless), e.g.::

    ros2 launch ikt_pose_commander commander.launch.py dashboard_port:=8180
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                   PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Robot-NEUTRAL launch defaults, taken from the toolkit's centralized config
# (ikt_common -> config/toolkit_defaults.yaml, section ``ikt_pose_commander``).
# The robot-specific selection (controlled_frame / joints / controllers) is NOT
# here: it defaults empty and is chosen at runtime (dashboard or ~/configure),
# auto-derived from the URDF + /controller_manager.
_FALLBACKS = {
    "command_mode": "jtc",
    "start_enabled": "false",
    "base_frame": "",
    "switch_controllers": "true",
    "controller_manager": "/controller_manager",
    "max_joint_speed": 0.5,
    "min_move_time": 0.5,
    "max_step_rad": 0.8,
    "joint_states_stale_after": 0.5,
    "status_rate_hz": 10.0,
    "dashboard_base_frame": "base_link",
}


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
        # Empty => single/default instance named ``ikt_pose_commander``.
        # Override (e.g. ``left``/``right``) for multi-arm setups, which
        # suffixes the node + dashboard namespace as ``ikt_pose_commander_<name>``.
        DeclareLaunchArgument("instance_name", default_value=""),
        # Robot-specific: empty => start UNCONFIGURED, pick the link at runtime
        # (dashboard / ~/configure); joints + controllers are auto-derived.
        DeclareLaunchArgument("controlled_frame", default_value=""),
        DeclareLaunchArgument("jtc_controller", default_value=""),
        DeclareLaunchArgument("fpc_controller", default_value=""),
        DeclareLaunchArgument("joints", default_value="['']"),
        # Robot-neutral, from central config:
        DeclareLaunchArgument("command_mode",
                              default_value=str(d["command_mode"])),
        DeclareLaunchArgument("start_enabled",
                              default_value=str(d["start_enabled"])),
        DeclareLaunchArgument("base_frame", default_value=str(d["base_frame"])),
        DeclareLaunchArgument("switch_controllers",
                              default_value=str(d["switch_controllers"])),
        DeclareLaunchArgument("controller_manager",
                              default_value=str(d["controller_manager"])),
        DeclareLaunchArgument("max_joint_speed",
                              default_value=str(d["max_joint_speed"])),
        DeclareLaunchArgument("min_move_time",
                              default_value=str(d["min_move_time"])),
        DeclareLaunchArgument("max_step_rad",
                              default_value=str(d["max_step_rad"])),
        DeclareLaunchArgument("joint_states_stale_after",
                              default_value=str(d["joint_states_stale_after"])),
        DeclareLaunchArgument("status_rate_hz",
                              default_value=str(d["status_rate_hz"])),
        # Empty => headless (no dashboard). Any port => also launch the dashboard
        # wired to this commander instance.
        DeclareLaunchArgument("dashboard_port", default_value=""),
        DeclareLaunchArgument("dashboard_base_frame",
                              default_value=str(d["dashboard_base_frame"])),
    ]

    node = Node(
        package="ikt_pose_commander",
        executable="commander_node",
        name=PythonExpression(
            ["'ikt_pose_commander' + ('_' + '",
             LaunchConfiguration("instance_name"),
             "' if '", LaunchConfiguration("instance_name"), "' else '')"]),
        output="screen",
        parameters=[{
            "controlled_frame": LaunchConfiguration("controlled_frame"),
            "jtc_controller": LaunchConfiguration("jtc_controller"),
            "fpc_controller": LaunchConfiguration("fpc_controller"),
            "command_mode": LaunchConfiguration("command_mode"),
            "start_enabled": LaunchConfiguration("start_enabled"),
            "base_frame": LaunchConfiguration("base_frame"),
            "joints": LaunchConfiguration("joints"),
            "switch_controllers": LaunchConfiguration("switch_controllers"),
            "controller_manager": LaunchConfiguration("controller_manager"),
            "max_joint_speed": LaunchConfiguration("max_joint_speed"),
            "min_move_time": LaunchConfiguration("min_move_time"),
            "max_step_rad": LaunchConfiguration("max_step_rad"),
            "joint_states_stale_after":
                LaunchConfiguration("joint_states_stale_after"),
            "status_rate_hz": LaunchConfiguration("status_rate_hz"),
        }],
    )

    # Conditionally include the dashboard when dashboard_port is non-empty.
    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ikt_pose_commander"),
            "launch", "dashboard.launch.py"])),
        launch_arguments={
            "port": LaunchConfiguration("dashboard_port"),
            "commander_ns": PythonExpression(
                ["'/ikt_pose_commander' + ('_' + '",
                 LaunchConfiguration("instance_name"),
                 "' if '", LaunchConfiguration("instance_name"), "' else '')"]),
            "base_frame": LaunchConfiguration("dashboard_base_frame"),
        }.items(),
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration("dashboard_port"), "' != ''"])),
    )

    return LaunchDescription(
        args + [LogInfo(msg=f"[ikt_pose_commander] config: {source}"),
                node, dashboard])
