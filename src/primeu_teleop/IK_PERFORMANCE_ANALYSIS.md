# PrimeU Four-Tracker IK Performance Analysis

## Background

The current teleoperation setup uses four Lighthouse trackers from
`primeu_minimal.yaml`:

- `waist_yaw_link`
- `chest_link`
- `left_wrist_yaw_link`
- `right_wrist_yaw_link`

The intended control logic is:

- Use the waist tracker and chest tracker to estimate torso-relative motion.
- Use the chest tracker as the reference frame for both hand trackers.
- Feed only three Mink IK targets into the solver: waist, left wrist, right wrist.
- Keep the chest tracker as a relative reference, not as an independent IK target.

This matches the updated implementation: four tracker inputs, three `FrameTask`
IK targets.

## Observed Symptoms

After setting:

```bash
base_frame:=libsurvive_world
mujoco_sim:=false
rviz:=false
```

the robot can move, but `/joint_states` sometimes drops to very low frequency:

```text
average rate: 0.2-1.0 Hz
```

Runtime logs show:

```text
IK loop status=running, actual rate: 0.3 Hz, last_cycle=3070.7ms
Solve IK took 2.035s
Solve IK took 4.370s
```

Later profiling also showed:

```text
Mocap data is stale for more than 0.50s
target_err=1.186m/172.5deg
target_step=1.150m/169.9deg
joint_limit_margin=0.000
solve=508ms
```

## Execution Path

One IK loop cycle currently does:

```text
TF timer updates mocap_data
    ↓
ik_thread_loop copies mocap_data
    ↓
offset_mocap_data applies tracker offsets
    ↓
relative 4-tracker logic builds robot-space targets
    ↓
_update_ik_targets writes targets into 3 Mink FrameTasks
    ↓
_solve_ik calls mink.solve_ik(..., solver="daqp")
    ↓
_publish_joint_states publishes /joint_states
```

The relevant runtime stages are logged as:

```text
tf_update
scale
offset
relative_targets
update_targets
solve
publish
target_err
target_step
joint_limit_margin
```

## Important Finding

The IK loop does have a rate limiter:

```python
remaining = 1.0 / ik_loop_hz - cycle_time
if remaining > 0:
    time.sleep(remaining)
```

This is only an upper limit. With `ik_loop_hz:=20`, it only sleeps when a loop
finishes in less than 50 ms.

When the actual rate is 0.3-1 Hz, the loop is not being slowed by this sleep.
The loop itself is taking hundreds of milliseconds to several seconds.

## Evidence That Normal IK Is Not Always Slow

When targets are small and continuous, profiling showed a near-normal cycle:

```text
cycle=49.6ms
tf_update=0.1ms
offset=19.7ms
relative_targets=19.6ms
update_targets=0.4ms
solve=29.2ms
publish=0.3ms
```

This means the code path can run near 20 Hz under stable tracker input.

The severe slowdown appears after tracker data becomes stale or target jumps
become large.

## Root Cause

The performance issue is not simply "Mink IK is slow."

The actual failure chain is:

```text
IK solve becomes slow
    ↓
TF/mocap update becomes stale
    ↓
stale tracker data recovers with a large relative jump
    ↓
relative target changes by 1m+ or 100deg+
    ↓
robot target is far from current configuration
    ↓
joint limits and velocity limits become active
    ↓
DAQP needs much longer to solve the constrained QP
    ↓
IK loop slows further
```

The key log evidence is:

```text
Mocap data is stale
target_err=1.186m/172.5deg
target_step=1.150m/169.9deg
near_joint_limit
solve=508ms
```

and later:

```text
target_err=2.445m/72.0deg
joint_limit_margin=0.000
solve=252ms+
```

So the root cause is:

```text
TF stale + large target jumps + hard constraints cause DAQP to become slow.
```

## Secondary Issues Found

### 1. Per-stage warning logs can distort timing

High-frequency warning logs from the IK thread can block the launch output path
and make measured stage times look worse. This was reduced by adding
`warn_slow_stages` and disabling it by default.

### 2. `profile_logs` adds diagnostic work

Target error and joint margin diagnostics require extra kinematics queries.
These are useful for debugging but should not run every frame in the real-time
path.

### 3. `DEFAULT_MOCAP_STALE_TIMEOUT=0.5s` is aggressive

If one slow IK solve takes more than 0.5 s, mocap is marked stale. This can
trigger the bad recovery path and large target jumps.

## Current State

Implemented changes so far:

- Launch defaults to `primeu_minimal.yaml`.
- Four trackers are read from TF.
- Chest tracker is used as a relative reference only.
- Mink receives three actual IK targets: waist, left wrist, right wrist.
- `ik_dt` was added so IK integration uses control period instead of MuJoCo
  model timestep by default.
- Expensive profile metrics are throttled.
- `publish_tracker_relative_pose` defaults to false.
- `profile_logs` defaults to false.
- `warn_slow_stages` defaults to false.
- `qp_iter_limit` was added for DAQP.
- Low-overhead IK loop rate logging was added.

## Recommended Next Fixes

### 1. Reset relative anchors after stale mocap

When mocap becomes stale, the next valid tracker snapshot should become a new
neutral reference. Otherwise stale recovery can create a huge relative target
jump.

Suggested behavior:

```text
if mocap stale:
    clear relative-control anchors
on next valid tracker frame:
    recapture neutral tracker snapshot
```

### 2. Add target step limiting

Before writing generated targets into `mocap_data_adjusted`, clamp each target
change relative to the previous target.

Suggested limits:

```text
max_target_translation_step = 0.05 m per cycle
max_target_rotation_step = 10 deg per cycle
```

This keeps the QP input continuous even when tracker TF jumps.

### 3. Add target jump rejection

If a target jump exceeds a large threshold, skip the frame or reset anchors.

Suggested thresholds:

```text
translation jump > 0.30 m
rotation jump > 45 deg
```

This prevents impossible single-frame targets from entering DAQP.

### 4. Make mocap stale timeout configurable

Expose a parameter such as:

```text
mocap_stale_timeout:=2.0
```

The current fixed value of 0.5 s is too likely to trigger during a temporary
solver slowdown.

### 5. Keep DAQP iteration limits

`qp_iter_limit` should remain configurable. It prevents one bad QP from
blocking the entire teleoperation loop.

Good test values:

```text
qp_iter_limit:=40
qp_iter_limit:=80
qp_iter_limit:=120
```

## Recommended Debug Commands

Run with minimal output:

```bash
ros2 launch primeu_teleop teleop.launch.py \
  base_frame:=libsurvive_world \
  mujoco_sim:=false \
  rviz:=false \
  qp_iter_limit:=40
```

Run with profile logs:

```bash
ros2 launch primeu_teleop teleop.launch.py \
  base_frame:=libsurvive_world \
  mujoco_sim:=false \
  rviz:=false \
  profile_logs:=true \
  qp_iter_limit:=40
```

Check output rate:

```bash
ros2 topic hz /joint_states
```

Check tracker TF:

```bash
ros2 run tf2_ros tf2_echo libsurvive_world LHR-4622FDDD
```

Check TF publish rate:

```bash
ros2 topic hz /tf
```

## Conclusion

The current bottleneck is not the nominal three-target Mink IK problem. Under
stable targets, it can run close to the intended 20 Hz.

The main issue is instability in the target stream:

```text
stale mocap -> large relative target jump -> hard constrained QP -> slow DAQP
```

The next engineering step should be to make the target stream robust by
recapturing anchors after stale TF and limiting/rejecting target jumps before
they reach the QP solver.
