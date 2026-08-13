# Root README Operations Guide Design

## Goal

Create a Chinese root `README.md` that acts as the GitHub-facing operating
manual for the uploaded Jetson Orin precision-landing implementation. It must
let an operator find the correct command and log file without reading source or
design documents.

## Content Order

The guide starts with the system scope and the shared localization chain:

```text
MID360/FAST-LIO -> /drone_Odometry -> px4_pos_estimator
-> /mavros/vision_pose/pose -> PX4 -> /mavros/local_position/pose
```

It then documents, in operational order:

1. Jetson paths, hardware assumptions, and OpenCV 4.5.5 build command.
2. Companion-controller direct, `test`, `status`, `stop`, and tmux commands.
3. PX4-native direct, `test`, `status`, `stop`, and tmux commands.
4. The rule that the two variants must never run simultaneously.
5. Image/debug topics and both CSV log locations.
6. The local AprilTag A4 PNG/PDF asset paths and scale formula.
7. Preflight checks: manual arming/mode switching, output behavior in `test`,
   `LOCAL_NED`, and the unresolved PX4 v1.12 `PLD_*` capability check.
8. A short troubleshooting table for startup timeout, session conflict, missing
   D455, absent odometry, and locating component logs in tmux.

## Accuracy and Safety Wording

The README distinguishes “publisher exists” from “command accepted by PX4.” It
does not imply that running a script arms, changes mode, or starts a flight.
Companion direct mode enables the controller but remains gated until the pilot
manually selects OFFBOARD. PX4-native direct mode enables landing-target
publication but the pilot still chooses the PX4 landing mode manually.

The PX4-native section explicitly blocks any claim of flight readiness until
the custom PX4 v1.12 firmware exposes and supports its `PLD_*` precision-land
capability in QGroundControl or the MAVLink console.

## Log Locations

The guide records these exact paths:

```text
/home/wxh/nb_code/autofly_ws/log/precision_landing_joint_pnp_error.csv
/home/wxh/nb_code/autofly_ws/log/px4_native/flight_YYYYMMDD_HHMMSS.csv
/home/wxh/nb_code/autofly_ws/log/px4_native/summary.csv
```

It explains that the joint-PnP CSV contains detection/reprojection/FRD and
guide/control values, while the native detail and summary logs contain flight
state, cached target, final forward/right/down error, horizontal error, validity,
and touchdown-time selection.

## Verification

Check every documented command against the actual executable scripts and launch
files. Check every documented path against source defaults. Run Markdown link
and placeholder scans, `git diff --check`, and the existing launcher contract
test before committing and pushing the README update.
