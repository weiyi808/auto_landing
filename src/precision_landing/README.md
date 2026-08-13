# D455 AprilTag Precision Landing

This ROS1 Noetic package detects the supplied AprilTag 36h11 landing board with
OpenCV 4.5.5 and estimates one joint board pose. The board-frame origin is the
physical center of ID 0. It does not depend on `apriltag_ros` or `cv_bridge`.

## Complete Jetson startup scripts

The no-argument commands start the complete MAVROS, D455, MID360/FAST-LIO,
external-vision transfer, and selected precision-landing chain. Run exactly one
landing implementation at a time:

```bash
cd /home/wxh/nb_code/autofly_ws
./run_companion_precision_landing.sh
# or, never simultaneously:
./run_px4_native_precision_landing.sh
```

Direct mode enables only the selected landing output. Arming and flight-mode
selection remain manual. These scripts do not start Falcon exploration, an
Ego/Falcon planner, `px4_replan_sender`, `auto_control`, or `pubcmd`.

For a complete ground test with landing output disabled:

```bash
./run_companion_precision_landing.sh test
./run_px4_native_precision_landing.sh test
```

Inspect or stop the script-owned tmux session:

```bash
./run_companion_precision_landing.sh status
tmux attach -t landing_companion
./run_companion_precision_landing.sh stop

./run_px4_native_precision_landing.sh status
tmux attach -t landing_px4_native
./run_px4_native_precision_landing.sh stop
```

The common indoor localization flow is:

```text
MID360/FAST-LIO
  -> /drone_Odometry
  -> px4_pos_estimator
  -> /mavros/vision_pose/pose
  -> PX4
  -> /mavros/local_position/pose
```

Each dependent component starts only after the preceding connection or topic
has produced data. If startup times out, attach to the named tmux session and
inspect the indicated window; then use the matching `stop` command before
retrying.

## Print scale

The pose model uses the detected black-square edges, not the surrounding white
quiet border. Nominal black-square sizes are 68 mm (IDs 10-13), 28.8 mm
(IDs 20-23), and 14.4 mm (ID 0).

If the complete page was uniformly resized, measure the black square of ID 0:

```text
board_scale = measured_ID0_black_side_mm / 14.4
```

The same scale is applied to tag sizes and center spacing. Uniform scaling does
not move the target pixel, but correct scale is needed for metric height and
control magnitude.

## Build

On the Jetson:

```bash
cd /home/wxh/nb_code/autofly_ws
./build_precision_landing_opencv455.sh
```

The script pins `/usr/local/opencv-455-cuda` and rejects OpenCV 4.2 linkage.

## Two separate landing implementations

The two flight implementations are deliberately separated and must not run at
the same time:

- `d455_companion_precision_landing.launch` publishes BODY_NED velocity
  setpoints from the companion controller.
- `d455_px4_native_precision_landing.launch` only publishes an absolute local
  landing target and lets PX4 v1.12 perform precision landing.

Both launch files default their flight-facing output to `false`. Neither node
arms the vehicle, changes mode, requests PRECLAND, or calls LAND. The pilot
switches flight mode manually.

## Companion-controller ground observation

Start the downward D455 color stream, then run with flight output disabled:

```bash
source /home/wxh/nb_code/autofly_ws/devel/setup.bash
roslaunch precision_landing d455_companion_precision_landing.launch \
  enable_flight:=false board_scale:=1.0
rqt_image_view /precision_landing/debug/image_raw
```

The large red point is the joint estimate of the ID 0 center. The overlay also
shows valid tag count, reprojection RMS, FRD error, guide velocity, actual
BODY_NED command, and flight direction. CSV output is written to:

```text
/home/wxh/nb_code/autofly_ws/log/precision_landing_joint_pnp_error.csv
```

Do not switch to OFFBOARD until ground movement confirms that the magenta arrow
always points from the aircraft toward the physical ID 0 center. This package
never arms, changes PX4 mode, or calls LAND; those actions remain manual.

## PX4 v1.12 native precision-landing comparison

This path publishes `geometry_msgs/PoseStamped` to the MAVROS outgoing topic
`/mavros/landing_target/pose`. MAVROS must be configured with:

```text
/mavros/landing_target/mav_frame: LOCAL_NED
fcu_protocol: v2.0
```

The node constructs one absolute local-ENU target from the D455 observation and
the vehicle pose. MAVROS converts it to the `LOCAL_NED` absolute XYZ
`LANDING_TARGET` message expected by PX4 v1.12. During a vision dropout, the
cached world target remains fixed for at most 0.20 s; after that, publishing
stops. It does not chase a moving target computed from stale camera data.

First run observation-only:

```bash
source /home/wxh/nb_code/autofly_ws/devel/setup.bash
roslaunch precision_landing d455_px4_native_precision_landing.launch \
  enable_target_publish:=false board_scale:=1.0
rostopic echo /precision_landing/px4_native/status
rostopic echo /precision_landing/px4_native/target_local_enu
```

Before enabling target publication, verify in QGroundControl or the PX4 MAVLink
console that this custom PX4 v1.12 firmware exposes its `PLD_*` parameters and
has the precision-land / landing-target estimator support enabled. The current
airframe did not return `PLD_*` through MAVROS parameter lookup, so flight use
remains blocked until that capability is confirmed.

After ground checks, target publication can be enabled while disarmed:

```bash
roslaunch precision_landing d455_px4_native_precision_landing.launch \
  enable_target_publish:=true board_scale:=1.0
```

The node refuses publication if FCU connection or local pose is stale, the
MAVROS frame is not `LOCAL_NED`, or the companion node `/precision_landing` is
running. Detailed airborne-session CSV files and one touchdown summary are
written under:

```text
/home/wxh/nb_code/autofly_ws/log/px4_native/
```

`summary.csv` uses the newest valid board observation in the final 1.0 s before
the MAVROS landed-state transition to `ON_GROUND`. If no such observation
exists, the row is explicitly marked invalid instead of reusing a stale error.
