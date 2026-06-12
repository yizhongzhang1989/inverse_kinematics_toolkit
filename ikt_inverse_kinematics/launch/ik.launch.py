"""Launch the headless IK solver node (no dashboard).

The solver reads ``/robot_description`` and ``/joint_states`` and exposes its
JSON solve API + ``~/solution`` / ``~/status`` outputs. It is advisory only and
never commands the robot.

Launch-arg defaults come from the toolkit's centralized config under the
``ikt_inverse_kinematics:`` section (``ikt_common`` ->
``config/toolkit_defaults.yaml``, overridable by the workspace
``robot_config.yaml`` or ``ROBOT_CONFIG_PATH``). Solver tuning (tolerances,
stiffness, arm-angle chains) stays in the package's ``config/ik_defaults.yaml``.
CLI overrides win over both.

Examples::

    ros2 launch ikt_inverse_kinematics ik.launch.py
    ros2 launch ikt_inverse_kinematics ik.launch.py base_frame:=base_link
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Hard-coded fallbacks if the central config is missing or lacks a key.
_FALLBACKS = {
    "robot_description_topic": "/robot_description",
    "joint_states_topic": "/joint_states",
    "base_frame": "",
}


def _defaults():
    """Return (defaults_dict, source_str) from the centralized toolkit config."""
    try:
        from ikt_common.config_manager import get_config  # type: ignore
    except Exception as exc:  # noqa: BLE001
        return (dict(_FALLBACKS),
                f"FALLBACK (could not import ikt_common.config_manager: "
                f"{type(exc).__name__}: {exc})")
    try:
        cfg = get_config()
    except Exception as exc:  # noqa: BLE001
        return (dict(_FALLBACKS),
                f"FALLBACK (could not load config: {type(exc).__name__}: {exc})")
    if not cfg.has("ikt_inverse_kinematics"):
        return (dict(_FALLBACKS),
                f"FALLBACK (no 'ikt_inverse_kinematics:' section in "
                f"{cfg.config_path})")
    sec = cfg.section("ikt_inverse_kinematics")
    return ({k: sec.get(k, v) for k, v in _FALLBACKS.items()},
            f"loaded from {cfg.config_path}")


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("ikt_inverse_kinematics")
    params = os.path.join(pkg, "config", "ik_defaults.yaml")
    d, source = _defaults()

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=params),
        DeclareLaunchArgument("base_frame", default_value=str(d["base_frame"])),
        DeclareLaunchArgument(
            "robot_description_topic",
            default_value=str(d["robot_description_topic"])),
        DeclareLaunchArgument(
            "joint_states_topic", default_value=str(d["joint_states_topic"])),
        LogInfo(msg=f"[ikt_inverse_kinematics] config: {source}"),
        Node(
            package="ikt_inverse_kinematics",
            executable="ik_node",
            name="ik_node",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "base_frame": LaunchConfiguration("base_frame"),
                    "robot_description_topic":
                        LaunchConfiguration("robot_description_topic"),
                    "joint_states_topic":
                        LaunchConfiguration("joint_states_topic"),
                },
            ],
        ),
    ])
