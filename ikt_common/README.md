# `ikt_common`

Centralized configuration loader and workspace utilities for the
**cartesian_controllers_toolkit**. Every other package in the toolkit should
read its parameters through `ikt_common.config_manager.ConfigManager` so
there is exactly one source of truth (`config/robot_config.yaml`) for
IPs, ports, device paths, robot kinematics, etc.

> Named `ikt_common` (cartesian_controllers_toolkit common) rather than
> `common` so the toolkit can be added to any workspace as a git submodule
> without its support package colliding with a host package named `common`.

## Why a centralized config?

This workspace will host several packages that need to agree on the same
values (the F/T sensor's serial port, the robot's IP, the dashboard's
port, etc.). Without a shared config you end up either hard-coding the
same value in many launch files or duplicating launch-args in every
launch invocation. With one YAML file:

- one place to edit when you change machine, network, or hardware;
- the same values are visible to drivers, web dashboards, scripts, and
  ad-hoc tools;
- `robot_config.yaml` is gitignored, so machine-specific overrides don't
  leak into commits — the committed `robot_config.example.yaml` is the
  template.

---

## Repo layout

```
<your_workspace>/
├── config/
│   ├── robot_config.example.yaml   (committed, the template)
│   └── robot_config.yaml           (LOCAL only, gitignored)
└── src/
    └── ikt_common/                 (this package)
        ├── config/
        │   └── toolkit_defaults.yaml   (packaged default; the built-in fallback)
        └── ikt_common/
            ├── config_manager.py
            └── workspace_utils.py
```

To bring up a new machine:

```bash
cp config/robot_config.example.yaml config/robot_config.yaml
${EDITOR:-nano} config/robot_config.yaml
```

## Config resolution order

`ConfigManager` loads the **first** source that exists, in this order:

1. **A user-specified file** — the `ROBOT_CONFIG_PATH` environment
   variable (absolute path; `DUCO_CONTROL_CONFIG` is accepted as a
   legacy alias).
2. **The workspace config** — `<workspace>/config/robot_config.yaml`,
   or `robot_config.example.yaml` if the former is absent.
3. **The toolkit's packaged default** — `toolkit_defaults.yaml`, shipped
   inside this package under `ikt_common/config/`. This centralises the
   default ports / topics / frames / limits for every toolkit package so
   they are no longer hard-coded in each launch file.

If even the packaged default can't be read (e.g. `ikt_common` isn't built
yet), each launch file falls back to the hard-coded `_FALLBACKS` dict it
carries as a final safety net.

> **Customising for your robot:** don't edit the packaged default. Copy
> the sections you need into your workspace's `config/robot_config.yaml`
> (wins via step 2), or point `ROBOT_CONFIG_PATH` at your own file
> (wins via step 1).

---

## Using it from Python (drivers, nodes, scripts)

```python
from ikt_common.config_manager import get_config

cfg = get_config()                          # singleton; cheap to call
print(cfg.config_path)                      # which YAML was loaded

# dot-path access with a default
topic   = cfg.get("ft_sensor_gravity_compensation.input_topic", "/ft_sensor/wrench_raw")
gravity = cfg.get("ft_sensor_gravity_compensation.gravity",     9.80665)
web     = cfg.get("ft_sensor_dashboard.port",                   8080)

# scoped view: handy to pass into a sub-component
ft = cfg.section("ft_sensor_gravity_compensation")  # SectionView
print(ft.get("sensor_frame"))                       # "tool0"

# introspect
cfg.list_sections()                                 # ['ft_sensor_gravity_compensation', 'ft_sensor_dashboard']
cfg.has("ft_sensor_gravity_compensation.gravity")   # True
```

`get(...)` always returns the supplied `default` if any segment of the
dot path is missing, so you can roll out new keys gradually without
breaking older config files.

## Using it from a launch file

The launch files in this workspace read defaults from the central
config and let CLI overrides win:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ikt_common.config_manager import get_config

def generate_launch_description():
    cfg = get_config().section("ft_sensor_gravity_compensation")
    args = [
        DeclareLaunchArgument("input_topic",  default_value=cfg.get("input_topic", "/ft_sensor/wrench_raw")),
        DeclareLaunchArgument("sensor_frame", default_value=cfg.get("sensor_frame", "tool0")),
        DeclareLaunchArgument("gravity",      default_value=str(cfg.get("gravity", 9.80665))),
    ]
    return LaunchDescription([*args, Node(...)])
```

## Workspace utilities

`ikt_common.workspace_utils` finds the project root and standard sub-dirs
without baking in any user-specific path.

```python
from ikt_common.workspace_utils import (
    get_workspace_root,    # absolute path to the repo root, or None
    get_config_dir,        # <root>/config
    get_temp_dir,          # <root>/temp  (created on demand)
)
```

The root is located by trying, in order:

1. the `ROBOT_WORKSPACE_ROOT` env var,
2. the share directory of any installed package in this workspace,
3. walking up from this file's location (development case),
4. `COLCON_PREFIX_PATH` / `ROS_WORKSPACE`.

A directory qualifies as the project root if it contains both `src/`
and `config/`.

---

## Building

```bash
cd <your_workspace>
colcon build --symlink-install --packages-select ikt_common
source install/setup.bash
```

After sourcing, any other package can `from ikt_common.config_manager
import get_config`.

## YAML conventions

- **Top-level keys are package names**, second-level keys are the
  arguments the package's launch file exposes (so they map 1-to-1 to
  `ros2 launch <pkg> <launch_file> <key>:=<value>`). Adding a new
  package = adding a new top-level section.
- Anything under a `paths:` mapping is auto-resolved against the
  project root if it isn't already absolute.
- Strings may contain `${ENV_VAR}`; unset variables are left as-is.
- The optional top-level `version:` is reserved for future schema
  migrations and is not exposed as a "section".

## Hot-reload

`ConfigManager` is a singleton. If you change the YAML at runtime
(usually only in dev/REPL), call:

```python
from ikt_common.config_manager import get_config
get_config().reload()
```
