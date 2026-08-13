# PX4 v1.12 Native Precision Landing Target Design

## Objective and Safety Boundary

Add a second landing method for A/B comparison with the existing companion-side
velocity controller. The new method delegates trajectory generation and landing
control to PX4 v1.12 and only supplies a vision landing target.

The companion node must not arm, change flight mode, send PRECLAND or LAND
commands, or publish velocity/position control setpoints. The pilot manually
selects PX4 Precision Land. The node publishes target information only.

## Verified Compatibility Context

The connected flight controller reports PX4 v1.12. The installed MAVROS is
1.20.1, uses MAVLink v2.0, and configures the landing target frame as
`LOCAL_NED`.

PX4 v1.12 accepts a MAVLink `LANDING_TARGET` position only when
`position_valid` is true and its frame is `MAV_FRAME_LOCAL_NED`. The live MAVROS
node subscribes to `/mavros/landing_target/pose`; this is therefore the outgoing
ROS input. `/mavros/landing_target/pose_in` is the incoming/diagnostic direction
and must not be used as the publisher destination.

The live `/mavros/param/get` calls currently do not return the standard
`PLD_*` parameters. Before flight, the operator must confirm in QGroundControl
or the PX4 MAVLink console that the firmware includes the precision-land and
landing-target-estimator modules and exposes the expected parameters. A missing
capability is a preflight failure, not a condition the ROS node may repair by
changing firmware or parameters automatically.

## Components and Data Flow

### Shared vision node

The OpenCV 4.5.5 joint-board PnP node remains the single vision source and
publishes the camera-relative physical ID 0 center on
`/precision_landing/board_center_camera`.

### PX4 landing-target publisher

A new `px4_landing_target_node` subscribes to:

- `/precision_landing/board_center_camera`;
- `/mavros/local_position/pose`;
- `/mavros/state`;
- `/mavros/extended_state`.

For a camera observation `(camera_x, camera_y, camera_z)`, the centered downward
D455 mapping to body FLU is:

```text
body_flu = (-camera_y, -camera_x, -camera_z)
```

Configurable camera translation in body FLU is added before rotation. The
vehicle quaternion from `/mavros/local_position/pose` rotates the body-FLU
vector into local ENU. The vehicle local-ENU position is then added:

```text
target_local_enu = vehicle_local_enu
                 + q_local_from_body * (camera_offset_body_flu + body_flu)
```

The result is published as `geometry_msgs/PoseStamped` with `frame_id=map` on
`/mavros/landing_target/pose`. MAVROS performs ENU-to-NED conversion and sends a
MAVLink v2 `LANDING_TARGET` with valid absolute position fields.

The publish rate is capped at 10 Hz to match the landing-target plugin
configuration. Vehicle pose and vision timestamps must both be fresh and
finite.

## Target-Loss State Machine

The publisher has four states:

- `WAITING`: FCU/local pose/vision prerequisites have not yet been satisfied.
- `TRACKING`: a fresh board observation has produced a valid absolute target.
- `SHORT_HOLD`: vision is temporarily absent; the last absolute world target is
  republished unchanged for at most 0.2 seconds.
- `LOST`: the hold window expired; target publication stops completely.

The held value is an absolute local-map target, not a stale camera-relative
measurement recombined with new vehicle motion. This prevents the landing point
from following the aircraft during a dropout.

Stopping publication intentionally lets PX4 v1.12 apply its own target timeout,
search, or fallback behavior. The node never substitutes companion velocity
control during loss. `short_hold_s` is configurable but defaults to 0.2 seconds
and must remain less than the PX4 landing-target timeout.

## A/B Mutual Exclusion

Provide separate launches:

- vision plus existing companion velocity controller;
- vision plus PX4 landing-target publisher.

The PX4-native launch must not start `precision_landing_node`, and the companion
controller launch must not start `px4_landing_target_node`. Both use the same
vision topic, but only one downstream landing method may be active for a test.

The PX4-native publisher also refuses to start if it detects an active publisher
from the companion controller on `/mavros/setpoint_raw/local` with the
`/precision_landing` node name.

## Accuracy Logging

Each airborne session creates a detailed CSV with a timestamped filename under
`/home/wxh/nb_code/autofly_ws/log/px4_native/`. Each valid or loss-state sample
contains:

- ROS timestamp and PX4 mode;
- tracking state and target age;
- camera-frame center and body-FRD forward/right/down error;
- horizontal error magnitude;
- vehicle local ENU position;
- published target local ENU position;
- landed state.

When `/mavros/extended_state` transitions from an airborne/landing state to
`LANDED_STATE_ON_GROUND`, the node appends one row to a persistent summary CSV.
The row contains method name `px4_native`, session start/end time, final valid
forward/right/horizontal error, final target age, valid sample count, PX4 mode,
and the detailed-log filename.

Because the marker can disappear immediately before touchdown, "final" is the
most recent valid observation no older than a configurable 1.0-second accuracy
window. If none exists, the summary explicitly records the final accuracy as
invalid instead of reusing an old measurement.

## Preflight and Failure Handling

The launch defaults to `enable_target_publish=false`. Observation mode still
computes conversions, states, and logs but does not publish to MAVROS. Publishing
requires all of the following:

- explicit `enable_target_publish=true`;
- FCU connected;
- current MAVROS landing-target frame parameter equals `LOCAL_NED`;
- fresh local pose;
- a `TRACKING` or allowed `SHORT_HOLD` target;
- no active companion precision-landing controller conflict.

The node reports each failed gate on a status topic and throttled ROS log. It
does not write PX4 parameters.

## Verification

- Unit tests cover camera-optical to body-FLU conversion, quaternion rotation,
  absolute-target calculation, stale local pose, short hold, publication stop,
  and touchdown summary selection.
- A ROS observation test runs with `enable_target_publish=false` and verifies no
  message is published on the MAVROS outgoing landing-target input.
- A gated bench test enables publication while disarmed, verifies 10 Hz output
  on `/mavros/landing_target/pose`, and confirms no output appears on
  `/mavros/setpoint_raw/local` from either new node.
- Existing joint-PnP and companion-controller tests remain green.
- The OpenCV 4.5.5-only build/link checks remain mandatory.
- Flight testing is blocked until PX4 v1.12 `PLD_*` capability is confirmed and
  ground movement verifies the absolute target remains fixed in local ENU.
