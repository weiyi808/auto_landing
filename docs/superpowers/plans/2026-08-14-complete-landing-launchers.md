# Complete Precision-Landing Launchers Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add two directly executable Jetson launch scripts that bring up MAVROS, D455, MID360/FAST-LIO, vision-position transfer, and either companion-controlled or PX4-native precision landing without any Falcon exploration or automatic-control nodes.

**Architecture:** Each self-contained Bash script owns one detached tmux session and starts its dependency windows sequentially, using message/parameter readiness checks instead of fixed delays. With no argument the selected landing output is enabled; `test` starts the identical sensing/localization chain with output disabled, while `status` and `stop` inspect or terminate only the script-owned session. A shell regression suite runs the launchers against stub ROS/tmux commands before either script is exercised on the Jetson.

**Tech Stack:** Bash, tmux, ROS1 Noetic CLI, MAVROS 1.20.1, realsense2_camera, Livox MID360, FAST-LIO, fly_utils `px4_pos_estimator`, precision_landing.

---

### Task 1: Define the Operator Contract With Failing Shell Tests

**Files:**
- Create: `test/test_complete_landing_launchers.sh`
- Create: `test/launcher_stubs/tmux`
- Create: `test/launcher_stubs/rosnode`
- Create: `test/launcher_stubs/rostopic`
- Create: `test/launcher_stubs/rosparam`
- Create: `test/launcher_stubs/rospack`
- Create: `test/launcher_stubs/rs-enumerate-devices`

- [ ] Create a temporary test root containing a fake ROS setup, fake autofly `devel/setup.bash`, fake `/dev/ttyTHS0` supplied through `FCU_DEVICE`, and a command log supplied through `LANDING_LAUNCHER_COMMAND_LOG`.
- [ ] Make the tmux stub record every invocation and emulate `has-session`, `new-session`, `new-window`, `list-windows`, `send-keys`, and `kill-session` using a session-state file. Make ROS stubs return: a healthy ROS master, connected FCU, live required topics, `LOCAL_NED`, all required packages, and D455 serial `046322251249`.
- [ ] Add assertions that invoking each missing launcher with no arguments records its operational launch argument:

```text
d455_companion_precision_landing.launch enable_flight:=true
d455_px4_native_precision_landing.launch enable_target_publish:=true
```

- [ ] Add assertions that invoking each launcher with `test` records the same full MAVROS/D455/FAST-LIO/position-transfer chain but changes only the landing output to `false`.
- [ ] Assert that both scripts record `mapping_mid360andmaping.launch` and `px4_pos_estimator.launch`, and never record `exploration_manager`, `ego_planner`, `px4_replan_sender`, `auto_control`, `pubcmd`, arming, `set_mode`, or `LAND`.
- [ ] Assert unknown arguments fail with usage, `status` calls only `tmux list-windows` and read-only ROS commands, and `stop` kills only its exact session name.
- [ ] Run `bash test/test_complete_landing_launchers.sh`; expect failure because both launcher scripts are missing.

### Task 2: Implement the Complete Companion-Controller Launcher

**Files:**
- Create: `run_companion_precision_landing.sh`
- Modify: `test/test_complete_landing_launchers.sh`

- [ ] Put a Chinese operator guide before executable logic documenting direct run, `test`, `status`, `stop`, all six runtime components, manual OFFBOARD switching, and the explicit exclusion of Falcon planning/control nodes.
- [ ] Parse exactly zero arguments, `test`, `status`, or `stop`. Set:

```bash
SESSION_NAME=landing_companion
LANDING_LAUNCH=d455_companion_precision_landing.launch
LANDING_ENABLE_NAME=enable_flight
LANDING_ENABLE_VALUE=true   # false only for test
```

- [ ] Add environment-backed proven defaults:

```bash
AUTOFLY_WS=/home/wxh/nb_code/autofly_ws
ROS_SETUP=/opt/ros/noetic/setup.bash
FCU_URL=/dev/ttyTHS0:3000000
FCU_DEVICE=/dev/ttyTHS0
GCS_URL=udp://@192.168.20.57
D455_SERIAL=046322251249
BOARD_SCALE=1.0
STARTUP_TIMEOUT_S=30
```

- [ ] Implement `require_file`, `require_command`, `require_package`, `node_exists`, `wait_for_ros_master`, `wait_for_fcu`, and `wait_for_topic_message`. Every wait uses a deadline, prints the exact failed condition, and returns nonzero without starting dependent windows.
- [ ] Reject either owned landing tmux session, `/precision_landing`, `/px4_landing_target`, `/px4_pos_estimator`, `/laserMapping`, or an existing MAVROS/D455 node before starting. Do not kill any of them.
- [ ] Create a detached tmux session, reuse an existing healthy ROS master or create an owned `roscore` window, and then create these named windows in order:

```text
mavros   roslaunch mavros px4.launch fcu_url:=${FCU_URL} gcs_url:=${GCS_URL}
d455     roslaunch realsense2_camera rs_camera.launch camera:=camera serial_no:=${D455_SERIAL} enable_color:=true color_width:=640 color_height:=480 color_fps:=30 enable_depth:=false enable_infra1:=false enable_infra2:=false initial_reset:=true
fastlio  roslaunch fast_lio mapping_mid360andmaping.launch
vision   roslaunch fly_utils px4_pos_estimator.launch
landing  roslaunch precision_landing d455_companion_precision_landing.launch enable_flight:=${LANDING_ENABLE_VALUE} board_scale:=${BOARD_SCALE}
monitor  watch the FCU state, localization topics, landing status, and debug rate without publishing
```

- [ ] Wait after each window for, respectively, connected MAVROS; D455 image and camera info; `/drone_Odometry`; `/mavros/vision_pose/pose` and `/mavros/local_position/pose`; and `/precision_landing/status`.
- [ ] Implement `status` to show tmux windows, `/mavros/state`, rates for `/drone_Odometry`, `/mavros/vision_pose/pose`, `/mavros/local_position/pose`, `/precision_landing/debug/image_raw`, `/precision_landing/status`, and publishers on `/mavros/setpoint_raw/local`.
- [ ] Implement `stop` by sending Ctrl-C to every pane in `landing_companion`, waiting briefly, and killing only `landing_companion`.
- [ ] Add a test-only `LANDING_LAUNCHER_DRY_RUN=1` path that records window commands and treats readiness probes through stubs, without changing the documented operator interface.
- [ ] Run `bash -n run_companion_precision_landing.sh` and the shell suite; require the companion cases to pass while PX4-native cases still fail because that script is missing.

### Task 3: Implement the Complete PX4-Native Launcher

**Files:**
- Create: `run_px4_native_precision_landing.sh`
- Modify: `test/test_complete_landing_launchers.sh`

- [ ] Use the same self-contained preflight, tmux lifecycle, MAVROS, D455, FAST-LIO, position-transfer, timeout, `status`, and `stop` behavior as the companion launcher, with:

```bash
SESSION_NAME=landing_px4_native
LANDING_LAUNCH=d455_px4_native_precision_landing.launch
LANDING_ENABLE_NAME=enable_target_publish
LANDING_ENABLE_VALUE=true   # false only for test
```

- [ ] Before creating the landing window in direct mode, require:

```bash
rosparam get /mavros/landing_target/mav_frame  # exactly LOCAL_NED
```

and print the PX4 v1.12 reminder that `PLD_*`/precision-land support must already have been confirmed in QGroundControl or MAVLink console. In `test` mode, report a non-`LOCAL_NED` value but keep output disabled and allow observation startup.
- [ ] Wait for `/precision_landing/px4_native/status`. In direct mode, require the node to advertise `/mavros/landing_target/pose`; in `test`, require that the new node does not advertise that topic.
- [ ] Make native `status` show the landing-target frame, native state, relevant localization rates, publishers on `/mavros/landing_target/pose`, and publishers on `/mavros/setpoint_raw/local` so an unintended companion control path is visible.
- [ ] Make native startup reject `/precision_landing` and any existing companion session before target output can start.
- [ ] Run `bash -n` for both scripts and `bash test/test_complete_landing_launchers.sh`; require every shell test to pass.

### Task 4: Document the Two Direct-Run Workflows

**Files:**
- Modify: `src/precision_landing/README.md`

- [ ] Add a first-run section showing that the no-argument commands are complete operational startup:

```bash
cd /home/wxh/nb_code/autofly_ws
./run_companion_precision_landing.sh
# or, never simultaneously:
./run_px4_native_precision_landing.sh
```

- [ ] Put the test commands immediately below, followed by `status`, `stop`, and `tmux attach -t landing_companion` / `tmux attach -t landing_px4_native`.
- [ ] Document the common data flow exactly as `/drone_Odometry -> px4_pos_estimator -> /mavros/vision_pose/pose -> PX4 -> /mavros/local_position/pose`.
- [ ] State that direct mode enables only the selected landing output, but arming and mode selection remain manual. Repeat that Falcon exploration/control nodes are excluded.
- [ ] Run `git diff --check` and scan documentation/scripts for contradictory commands.

### Task 5: Deploy and Validate Both Test Modes on the Jetson

**Files:**
- Sync: `run_companion_precision_landing.sh`
- Sync: `run_px4_native_precision_landing.sh`
- Sync: `src/precision_landing/README.md`

- [ ] Stop the existing `px4_native_observe` session only after confirming the aircraft is disarmed and in MANUAL. Preserve the already-running standalone MAVROS/D455 processes until their ownership is understood, then stop only known duplicate launch sessions before running the all-in-one scripts.
- [ ] Copy the scripts to `/home/wxh/nb_code/autofly_ws/`, set executable bits, and run `bash -n` plus the shell regression suite on the Jetson.
- [ ] Run `./run_companion_precision_landing.sh test`. Require connected MAVROS, D455 topics, `/drone_Odometry`, `/mavros/vision_pose/pose`, `/mavros/local_position/pose`, joint-PnP debug near its camera rate, `enable_flight=false`, and no Falcon exploration/control nodes.
- [ ] Run companion `status`, capture the evidence, then run companion `stop` and verify its session disappears without stopping unrelated ROS processes.
- [ ] Run `./run_px4_native_precision_landing.sh test`. Require the same sensor/localization chain, `enable_target_publish=false`, native status `publishing_disabled`, and no publisher from the new node on `/mavros/landing_target/pose`.
- [ ] Run native `status`, capture the evidence, then run native `stop` and verify scoped shutdown.

### Task 6: Perform Disarmed Operational-Mode Interface Checks

**Files:**
- No source changes expected

- [ ] Confirm `/mavros/state` reports `armed: False` and `mode: MANUAL` before each check. Abort if either condition is false.
- [ ] Start the companion script with no arguments, without presenting the AprilTag and without changing mode. Confirm its launch parameter is `enable_flight=true`, its safety state prevents motion, it never calls arming/mode/LAND services, and only the expected companion setpoint publisher exists. Stop it with its own script.
- [ ] Confirm `LOCAL_NED` and the operator's PX4 `PLD_*` capability acknowledgment before starting the native script with no arguments. Keep the vehicle disarmed and MANUAL. Confirm `enable_target_publish=true`, `/mavros/landing_target/pose` has only the native node as publisher, and `/mavros/setpoint_raw/local` has no native publisher. Stop it with its own script.
- [ ] Do not arm, switch modes, or conduct a flight during this implementation validation.

### Task 7: Final Verification, Commit, and Publish

**Files:**
- All launcher, test, and documentation changes

- [ ] Run fresh verification:

```bash
bash -n run_companion_precision_landing.sh
bash -n run_px4_native_precision_landing.sh
bash test/test_complete_landing_launchers.sh
git diff --check
```

- [ ] Scan both scripts for prohibited Falcon components and any arming, mode-change, or LAND service call; only negative-list validation/help text may mention them.
- [ ] Run the existing Jetson precision_landing tests and require zero failures so launcher changes do not regress the package.
- [ ] Commit with `feat: add complete precision landing launchers`.
- [ ] Push `feature/apriltag-board-joint-pnp` to origin without merging or modifying remote `master` or `main`.
- [ ] Leave the Jetson stopped or in explicit `test` mode, report which state remains active, and provide the two direct commands plus their test/status/stop commands.
