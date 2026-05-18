# primeu_teleop

PrimeU humanoid upper-body teleoperation using libsurvive Lighthouse
trackers and the Mink IK solver.

## Pipeline

```
HDC Tracker (hardware)
    ↓  USB + libsurvive
libsurvive_ros2  ──►  /tf  (frame: LHR-XXXXXXXX)
    ↓  tf2 lookup
primeu_teleop/tracker_retarget
    ↓  Mink IK (MuJoCo model of primeu)
/joint_states
    ↓
Your robot controller (or RViz / MuJoCo visualization)
```

## Package layout

```
primeu_teleop/
├── primeu_teleop/
│   ├── tracker_retarget_node.py   # main teleop node (Mink IK)
│   ├── calibrate_node.py          # snapshot T-pose tracker offsets
│   └── tf_inspector.py            # list available tracker frames
├── config/
│   └── primeu_libsurvive_mink_cfg.yaml   # tracker↔robot mapping
├── launch/
│   ├── teleop.launch.py           # main launch
│   └── inspect_trackers.launch.py
├── package.xml
└── setup.py
```

## Build

```sh
cd /home/lzq/pnd_teleoperation
source /opt/ros/jazzy/setup.bash
colcon build --packages-select primeu_teleop primeu_description
source install/setup.bash
```

The Python runtime dependencies (mujoco, mink, scipy, pydantic, ...)
are already installed into `pnd_teleoperation/.venv` via `uv sync`, so
you need to activate that virtualenv before running any node:

```sh
source /home/lzq/pnd_teleoperation/.venv/bin/activate
```

## Usage

### Step 1 — start libsurvive_ros2

```sh
source /home/lzq/ros2_ws/install/setup.bash
ros2 launch libsurvive_ros2 libsurvive_ros2.launch.py
```

### Step 2 — find your tracker IDs

```sh
ros2 launch primeu_teleop inspect_trackers.launch.py duration:=5.0
```

This prints every `child_frame_id` it sees on `/tf` and `/tf_static`.
Trackers are dynamic (e.g. `LHR-12345678`), base stations are static
(e.g. `LHB-XXXX`).

### Step 3 — edit the config

Open `config/primeu_libsurvive_mink_cfg.yaml` and replace the
`TRACKER_*` placeholders with the IDs from step 2:

```yaml
ik_cfg:
  - adam_link_name: "left_wrist_yaw_link"
    bone_name: "LHR-12345678"   # ← your left-hand tracker
    position_cost: 20.0
    orientation_cost: 18.0
    ...
```

You can leave any `TRACKER_*` entry as-is — the node skips placeholder
bones automatically.

### Step 4 — (optional) calibrate

Stand in a known reference pose (e.g. T-pose), then run:

```sh
ros2 run primeu_teleop calibrate --ros-args \
    -p tracker_frames:='[LHR-12345678, LHR-87654321]'
```

The captured position/orientation offsets are written to
`~/primeu_calibration.yaml` — paste them into the YAML config under
`pos_offset` / `rot_offset`.

### Step 5 — run teleop

```sh
ros2 launch primeu_teleop teleop.launch.py \
    mujoco_sim:=true \
    rviz:=true
```

Launch arguments:

| Argument      | Default | Description                                    |
| ------------- | ------- | ---------------------------------------------- |
| `robot_model` | primeu_description/mjcf/primeu_robot.xml | MuJoCo XML path |
| `ik_config`   | primeu_teleop/config/primeu_libsurvive_mink_cfg.yaml | YAML |
| `mujoco_sim`  | `true`  | Open MuJoCo passive viewer                      |
| `ik_solver`   | `daqp`  | QP solver: daqp / osqp / proxqp / quadprog      |
| `ik_iter_max` | `3`     | IK iterations per cycle                         |
| `ik_damping`  | `0.3`   | IK damping                                      |
| `rviz`        | `false` | Launch RViz                                     |

### Step 6 — verify

```sh
# Joint states flowing?
ros2 topic hz /joint_states

# Read current solution:
ros2 topic echo /joint_states --once
```

## Swapping the IK solver

Because `tracker_retarget_node.py` subclasses
`adam_mink.AdamMinkBase`, only `_solve_ik()` needs to change if you
want to drop Mink. See `adam_mink_base.py:_solve_ik` — it's ~15 lines.

A minimal skeleton for a new subclass:

```python
class PrimeUCustomIK(PrimeUTrackerRetarget):
    def _solve_ik(self) -> None:
        # self.tasks holds the FrameTasks (targets).
        # self.configuration.data.qpos is the current joint state.
        # Put your own IK here and write back to qpos.
        ...
```

Register the new executable in `setup.py`'s `entry_points`.

## Troubleshooting

**`ModuleNotFoundError: mink`** — activate the uv venv:
`source /home/lzq/pnd_teleoperation/.venv/bin/activate`

**No joint states published** — check TF with `inspect_trackers.launch.py`;
ensure the `bone_name` in the YAML matches a real frame.

**Robot jitters** — lower `ik_iter_max`, raise `ik_damping`, or tighten
`velocity_limit`.

**Wrong orientation** — adjust `rot_offset` in the YAML. Typical values
are 90°/180° rotations expressed as quaternions
(`[0.707, 0, 0, 0.707]` = 90° about Z).
