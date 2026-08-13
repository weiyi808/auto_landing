# Complete Precision-Landing Launchers Design

## Goal

Provide two independent executable shell scripts that start the complete indoor
flight-estimation and precision-landing stack on the Jetson Orin. Running a
script with no arguments starts its operational landing output; adding `test`
starts the same sensor and localization chain with landing output disabled.

The scripts are:

```text
run_companion_precision_landing.sh
run_px4_native_precision_landing.sh
```

Each script must contain a Chinese usage and safety introduction at the top so
the operator can understand all supported commands before reading its logic.

## Operator Interface

Direct execution is the complete operational mode:

```bash
./run_companion_precision_landing.sh
./run_px4_native_precision_landing.sh
```

Ground-observation mode is explicit:

```bash
./run_companion_precision_landing.sh test
./run_px4_native_precision_landing.sh test
```

Process inspection and controlled shutdown are available without separate
helper scripts:

```bash
./run_companion_precision_landing.sh status
./run_companion_precision_landing.sh stop
./run_px4_native_precision_landing.sh status
./run_px4_native_precision_landing.sh stop
```

Any other argument is rejected with the usage text and a nonzero exit status.

## Common Runtime Chain

Both scripts start only the following components, in dependency order:

1. Reuse a healthy ROS master or start one and wait for `/run_id`.
2. Start MAVROS with `/dev/ttyTHS0:3000000` and the existing optional GCS URL,
   then wait until `/mavros/state.connected` is true.
3. Start the downward D455 color stream using serial `046322251249`, 640x480 at
   30 fps, with depth and infrared disabled. Wait for color image and camera
   information topics.
4. Start `fast_lio mapping_mid360andmaping.launch`, which starts the MID360
   driver and FAST-LIO mapping. Wait for `/drone_Odometry`.
5. Start only `fly_utils px4_pos_estimator.launch`. It consumes the remapped
   `/drone_Odometry` and publishes `/mavros/vision_pose/pose`; wait for both that
   topic and `/mavros/local_position/pose` before declaring localization ready.
6. Start the selected precision-landing launch file and expose its status and
   debug topics to the operator.

The scripts must not launch Falcon exploration, an Ego/Falcon planner,
`px4_replan_sender`, `auto_control`, `pubcmd`, or any automatic arming/mode/LAND
client.

## Landing Variants

### Companion controller

The companion script starts:

```text
precision_landing/d455_companion_precision_landing.launch
```

No-argument mode passes `enable_flight:=true`. `test` mode passes
`enable_flight:=false`. The node remains internally gated on FCU connection,
fresh target data, fresh vehicle data, and manual OFFBOARD selection.

### PX4 v1.12 native precision landing

The PX4-native script starts:

```text
precision_landing/d455_px4_native_precision_landing.launch
```

No-argument mode passes `enable_target_publish:=true`. `test` mode passes
`enable_target_publish:=false`. Before operational mode starts the landing
node, the script requires `/mavros/landing_target/mav_frame` to equal
`LOCAL_NED`. It prints a blocking reminder that the operator must have already
confirmed `PLD_*` and precision-land support in QGroundControl or the PX4
MAVLink console; the script cannot infer missing capability from this custom
PX4 v1.12 firmware.

The script only enables target publication. The pilot still arms and switches
PX4 mode manually.

## Process Layout and Lifecycle

Each landing variant owns a uniquely named tmux session. Tmux is the canonical
runtime because it works both from the Jetson desktop and through SSH where
`DISPLAY` is unset. The session contains separate named windows for ROS master,
MAVROS, D455, FAST-LIO, vision-position transfer, precision landing, and a
read-only monitor.

If the script is run from a graphical terminal, it prints the tmux attach
command rather than requiring a new GUI terminal. This preserves the Falcon
multi-tab operational pattern while keeping startup reproducible over SSH.

Direct or `test` startup rejects an existing session of either landing variant
and rejects conflicting ROS nodes. It never kills unrelated user processes.
`stop` sends an interrupt to the owned tmux session and removes only that
session. `status` reports session windows, required nodes, FCU state, topic
rates, landing status, and whether a flight-facing topic currently has a
publisher.

## Readiness and Failure Handling

Fixed sleeps are not readiness evidence. Each dependent window waits with a
finite timeout for the exact node, parameter, or topic it needs. A timeout
prints the failed requirement, identifies the tmux window containing its log,
and leaves the session available for diagnosis. Operational output is not
started after an unmet prerequisite.

Preflight checks cover ROS Noetic, the autofly workspace, executable nodes and
launch files, the FCU device, D455 serial, tmux, and required ROS packages.
Scripts accept environment overrides for workspace, FCU URL, GCS URL, D455
serial, board scale, and timeout values while retaining the proven Jetson
defaults.

## Verification

Shell tests use stubbed `rosnode`, `rostopic`, `rosparam`, `roslaunch`, and tmux
commands to verify mode parsing, component selection, output-enable arguments,
timeout behavior, conflict rejection, status reporting, and scoped shutdown.

Jetson verification runs both scripts in `test` mode and confirms:

- MAVROS is connected;
- D455 image and camera information are live;
- `/drone_Odometry`, `/mavros/vision_pose/pose`, and
  `/mavros/local_position/pose` are live;
- no Falcon planning/control nodes are present;
- the companion flight output is disabled;
- the PX4-native landing-target topic has no publisher from the new node;
- `status` reports the actual disabled state;
- `stop` removes only the script-owned session.

Operational-mode verification is performed disarmed and in MANUAL. It confirms
the expected publisher exists but does not arm, change flight mode, or conduct
an autonomous flight.
