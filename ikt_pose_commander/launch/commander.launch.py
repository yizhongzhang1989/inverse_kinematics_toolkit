"""Launch one ikt_pose_commander instance.

By default the node STREAMS to the ``forward_position_controller`` (``command_mode``
``fpc``): each ``~/target_pose`` is solved by IK and pushed as a joint setpoint to
``/<fpc_controller>/commands``. The controlled link is chosen at runtime (the
dashboard or ``~/configure``); the controller is auto-derived from
``/controller_manager`` unless you PIN its exact name -- via config
(``ikt_pose_commander:`` section) or the ``fpc_controller`` / ``jtc_controller``
argument, e.g.::

    ros2 launch ikt_pose_commander commander.launch.py \
        fpc_controller:=forward_position_controller \
        controlled_frame:=left_arm_Link7 instance_name:=left

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
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Robot-NEUTRAL launch defaults, taken from the toolkit's centralized config
# (ikt_common -> config/toolkit_defaults.yaml, section ``ikt_pose_commander``).
# The robot-specific selection (controlled_frame / joints / controllers) is NOT
# here: it defaults empty and is chosen at runtime (dashboard or ~/configure),
# auto-derived from the URDF + /controller_manager.
_FALLBACKS = {
    "command_mode": "fpc",
    "controlled_frame": "",
    "jtc_controller": "",
    "fpc_controller": "",
    "start_enabled": "false",
    "base_frame": "",
    "switch_controllers": "true",
    "controller_manager": "/controller_manager",
    "max_joint_speed": 0.5,
    "max_joint_accel": 3.0,
    "min_move_time": 0.5,
    "max_step_rad": 0.8,
    "joint_states_stale_after": 0.5,
    "control_rate_hz": 200.0,
    "status_rate_hz": 10.0,
    # Singularity handling (see commander_node.py / README). Near a singular
    # config the synchronized FPC stream crawls; decouple to per-joint profiles
    # there (gated on sigma_min) and cap the big joints via the per-joint maps.
    "singularity_decouple": "true",
    "singularity_sigma": 0.04,
    "singularity_exit_ratio": 2.0,
    "joint_speed_limits": "",
    "joint_accel_limits": "",
    "dashboard_port": "",
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
        # Robot-specific: from config (ikt_pose_commander: controlled_frame) or
        # this arg. Empty => start UNCONFIGURED, pick the link at runtime
        # (dashboard / ~/configure); joints + controllers are auto-derived.
        DeclareLaunchArgument("controlled_frame",
                              default_value=str(d["controlled_frame"])),
        # Controller names: "" = auto-derive from /controller_manager by matching
        # the controlled link's joints. Pin the exact name via the central
        # config (ikt_pose_commander: section) or these arguments.
        DeclareLaunchArgument("jtc_controller",
                              default_value=str(d["jtc_controller"])),
        DeclareLaunchArgument("fpc_controller",
                              default_value=str(d["fpc_controller"])),
        DeclareLaunchArgument("joints", default_value="['']"),
        # fixed_joints: joints held OUT of the IK (e.g. a lifter/torso joint on
        # the path to the tip that is driven separately). Empty = none fixed.
        # e.g.  fixed_joints:="['torso_lift_joint']"
        DeclareLaunchArgument("fixed_joints", default_value="['']"),
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
        DeclareLaunchArgument("max_joint_accel",
                              default_value=str(d["max_joint_accel"])),
        DeclareLaunchArgument("min_move_time",
                              default_value=str(d["min_move_time"])),
        DeclareLaunchArgument("max_step_rad",
                              default_value=str(d["max_step_rad"])),
        DeclareLaunchArgument("joint_states_stale_after",
                              default_value=str(d["joint_states_stale_after"])),
        DeclareLaunchArgument("control_rate_hz",
                              default_value=str(d["control_rate_hz"])),
        DeclareLaunchArgument("status_rate_hz",
                              default_value=str(d["status_rate_hz"])),
        # Singularity handling (per-joint decouple near a singularity).
        DeclareLaunchArgument("singularity_decouple",
                              default_value=str(d["singularity_decouple"])),
        DeclareLaunchArgument("singularity_sigma",
                              default_value=str(d["singularity_sigma"])),
        DeclareLaunchArgument("singularity_exit_ratio",
                              default_value=str(d["singularity_exit_ratio"])),
        # Per-joint speed/accel caps for the decoupled mode (JSON object string).
        DeclareLaunchArgument("joint_speed_limits",
                              default_value=str(d["joint_speed_limits"])),
        DeclareLaunchArgument("joint_accel_limits",
                              default_value=str(d["joint_accel_limits"])),
        # From config (ikt_pose_commander: dashboard_port) or this arg.
        # Empty => headless (no dashboard). Any port => also launch the dashboard
        # wired to this commander instance.
        DeclareLaunchArgument("dashboard_port",
                              default_value=str(d["dashboard_port"])),
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
            "fixed_joints": LaunchConfiguration("fixed_joints"),
            "switch_controllers": LaunchConfiguration("switch_controllers"),
            "controller_manager": LaunchConfiguration("controller_manager"),
            "max_joint_speed": LaunchConfiguration("max_joint_speed"),
            "max_joint_accel": LaunchConfiguration("max_joint_accel"),
            "min_move_time": LaunchConfiguration("min_move_time"),
            "max_step_rad": LaunchConfiguration("max_step_rad"),
            "joint_states_stale_after":
                LaunchConfiguration("joint_states_stale_after"),
            "control_rate_hz": LaunchConfiguration("control_rate_hz"),
            "status_rate_hz": LaunchConfiguration("status_rate_hz"),
            "singularity_decouple":
                LaunchConfiguration("singularity_decouple"),
            "singularity_sigma": LaunchConfiguration("singularity_sigma"),
            "singularity_exit_ratio":
                LaunchConfiguration("singularity_exit_ratio"),
            # Force str so launch's YAML inference doesn't parse the JSON map
            # into a dict (the node param is a string it json-decodes itself).
            "joint_speed_limits": ParameterValue(
                LaunchConfiguration("joint_speed_limits"), value_type=str),
            "joint_accel_limits": ParameterValue(
                LaunchConfiguration("joint_accel_limits"), value_type=str),
        }],
    )

    # Conditionally include the dashboard when dashboard_port is non-empty.
    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ikt_pose_commander"),
            "launch", "dashboard.launch.py"])),
        launch_arguments={
            "port": LaunchConfiguration("dashboard_port"),
            # Suffix the dashboard node name with the instance so multiple arms
            # don't collide on one ROS node name (mirrors the commander node).
            "instance_name": LaunchConfiguration("instance_name"),
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
