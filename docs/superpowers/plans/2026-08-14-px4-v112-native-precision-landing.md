# PX4 v1.12 Native Precision Landing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a target-only PX4 v1.12 precision-landing node with deterministic target-loss handling, A/B launch isolation, and automatic touchdown-accuracy logs.

**Architecture:** A pure C++ core converts a fresh D455 board observation plus MAVROS local pose into one cached absolute local-ENU landing point and controls `WAITING/TRACKING/SHORT_HOLD/LOST` publication state. A ROS adapter publishes that point only to MAVROS `/mavros/landing_target/pose`, enforces manual-mode and conflict gates, and records detailed and summary CSV files. The existing OpenCV joint-PnP vision node is shared, while PX4-native and companion-controller launches remain mutually exclusive.

**Tech Stack:** ROS1 Noetic, C++14, Eigen3, geometry_msgs, mavros_msgs, PX4 v1.12, MAVLink v2 LandingTarget via MAVROS 1.20.1, GoogleTest.

---

### Task 1: Define Target Conversion and Loss-State Contracts

**Files:**
- Create: `src/precision_landing/include/precision_landing/px4_landing_target_core.hpp`
- Create: `src/precision_landing/test/test_px4_landing_target_core.cpp`
- Modify: `src/precision_landing/CMakeLists.txt`

- [ ] Write a failing conversion test with camera optical `(0.20, 0.10, 1.00)`, identity vehicle quaternion, vehicle ENU `(2,3,4)`, and zero camera offset. Require body FLU `(-0.10,-0.20,-1.00)` and target ENU `(1.90,2.80,3.00)`.
- [ ] Write a failing tilted-pose test using an Eigen quaternion and verify `target_enu = vehicle_enu + q * body_flu` numerically.
- [ ] Write failing state tests that require a new fresh observation to enter `TRACKING`, cache one absolute target, preserve that exact target in `SHORT_HOLD`, and stop publication in `LOST` after 0.2 seconds.
- [ ] Write failing safety-gate tests requiring `should_publish=false` when `enable_target_publish=false`, FCU disconnected, local pose stale, MAVROS frame not `LOCAL_NED`, or companion-controller conflict is present.
- [ ] Run `catkin_make test_px4_landing_target_core`; expect failure because the core API is missing.

The test-facing API is:

```cpp
enum class NativeTargetState { WAITING, TRACKING, SHORT_HOLD, LOST, BLOCKED };
struct NativeTargetInput {
  double now_s, vision_stamp_s, local_pose_age_s;
  bool vision_valid, fcu_connected, enable_target_publish;
  bool mavros_local_ned, controller_conflict;
  Eigen::Vector3d camera_center;
  Eigen::Vector3d vehicle_position_enu;
  Eigen::Quaterniond local_from_body;
};
struct NativeTargetOutput {
  NativeTargetState state;
  bool target_valid, should_publish;
  Eigen::Vector3d target_enu, body_flu, body_frd;
  double target_age_s;
  std::string reason;
};
class Px4LandingTargetCore {
 public:
  Px4LandingTargetCore(double fresh_s, double short_hold_s,
                       double local_pose_timeout_s,
                       Eigen::Vector3d camera_offset_body_flu);
  NativeTargetOutput step(const NativeTargetInput& input);
};
```

### Task 2: Implement the Minimal Stateful Core

**Files:**
- Create: `src/precision_landing/src/px4_landing_target_core.cpp`
- Modify: `src/precision_landing/CMakeLists.txt`

- [ ] Implement camera conversion exactly as `body_frd=(-camera_y,camera_x,camera_z)` and `body_flu=(body_frd.x,-body_frd.y,-body_frd.z)`.
- [ ] Normalize and validate the vehicle quaternion; reject non-finite vectors, non-positive camera depth, stale local pose, and invalid time ordering.
- [ ] Process a vision sample only when `vision_stamp_s` is newer than the last processed stamp. Cache the resulting absolute ENU target so aircraft motion during a dropout cannot move the held landing point.
- [ ] Return `TRACKING` while target age is within `fresh_s`, `SHORT_HOLD` until `short_hold_s`, and `LOST` afterwards. Set `should_publish` only when all safety gates pass.
- [ ] Run `test_px4_landing_target_core`; expect all conversion, cache, loss, and gate tests to pass.

### Task 3: Define Touchdown Accuracy Selection

**Files:**
- Create: `src/precision_landing/include/precision_landing/landing_accuracy.hpp`
- Create: `src/precision_landing/test/test_landing_accuracy.cpp`

- [ ] Write a failing test with valid samples at touchdown minus 1.2, 0.8, and 0.1 seconds. With a 1.0-second window, require selection of the 0.1-second sample as final and count only the two in-window samples.
- [ ] Write a failing test where all observations are older than 1.0 seconds and require `valid=false` rather than stale-value reuse.
- [ ] Implement `selectFinalAccuracy(samples, touchdown_s, window_s)` as a pure header utility returning the newest in-window sample, horizontal error via `hypot(forward,right)`, and in-window sample count.
- [ ] Run `test_landing_accuracy`; expect both cases to pass.

### Task 4: Add the Target-Only ROS Adapter and CSV Logger

**Files:**
- Create: `src/precision_landing/src/px4_landing_target_node.cpp`
- Modify: `src/precision_landing/CMakeLists.txt`
- Modify: `src/precision_landing/package.xml`

- [ ] Subscribe to board center, MAVROS local pose, state, and extended state. Read `/mavros/landing_target/mav_frame` and require the value `LOCAL_NED`.
- [ ] Detect `/precision_landing` in the ROS master node list as a companion-controller conflict. Publish status on `/precision_landing/px4_native/status` and the computed target on `/precision_landing/px4_native/target_local_enu` for inspection.
- [ ] Publish `geometry_msgs/PoseStamped` to `/mavros/landing_target/pose` at no more than 10 Hz only when the core returns `should_publish=true`. Never advertise `/mavros/setpoint_raw/local` and never create arming, mode, PRECLAND, or LAND service clients.
- [ ] Start a timestamped detailed CSV when airborne state begins. Log state, age, camera coordinates, FRD error, vehicle ENU, target ENU, mode, and landed state.
- [ ] On transition to `LANDED_STATE_ON_GROUND`, append a `px4_native` row to `summary.csv` using `selectFinalAccuracy`; record invalid when the final valid observation is outside the 1.0-second window.
- [ ] Compile `px4_landing_target_node` with `-Wall -Wextra -Wpedantic` and resolve all warnings.

### Task 5: Split A/B Launches and Document PX4 v1.12 Preconditions

**Files:**
- Create: `src/precision_landing/launch/d455_vision_only.launch`
- Create: `src/precision_landing/launch/d455_companion_precision_landing.launch`
- Create: `src/precision_landing/launch/d455_px4_native_precision_landing.launch`
- Modify: `src/precision_landing/launch/d455_precision_landing.launch`
- Modify: `src/precision_landing/README.md`

- [ ] Put only the OpenCV 4.5.5 joint-PnP node in `d455_vision_only.launch`.
- [ ] Include vision plus `precision_landing_node` in the companion launch; keep `enable_flight=false` by default.
- [ ] Include vision plus `px4_landing_target_node` in the PX4-native launch; keep `enable_target_publish=false` by default and expose `fresh_s=0.10`, `short_hold_s=0.20`, `final_accuracy_window_s=1.0`, and camera-offset arguments.
- [ ] Keep the legacy `d455_precision_landing.launch` as a compatibility include of the companion launch.
- [ ] Document that the correct MAVROS outgoing topic is `/mavros/landing_target/pose`, PX4 v1.12 requires MAVLink2 and LOCAL_NED absolute XYZ, the pilot switches mode manually, and flight is blocked until `PLD_*` capability is visible in QGC or the PX4 console.

### Task 6: Jetson Build, Test, and Observation Validation

**Files:**
- Sync package to `/home/wxh/nb_code/autofly_ws/src/precision_landing`

- [ ] Run the new core tests before implementation in the isolated Jetson `/tmp` catkin workspace and record the expected RED failure.
- [ ] Run `/home/wxh/nb_code/autofly_ws/build_precision_landing_opencv455.sh`; require OpenCV `.so.405` only and no 4.2/cv_bridge linkage.
- [ ] Run `catkin_make run_tests_precision_landing` and `catkin_test_results`; require zero errors and zero failures.
- [ ] Stop the companion-controller observation session and launch PX4-native observation with `enable_target_publish=false`. Verify joint-PnP debug remains near 15 Hz and `/mavros/landing_target/pose` has no publisher from the new node.
- [ ] While moving the board/aircraft on the ground, inspect `/precision_landing/px4_native/target_local_enu` and require the absolute ENU target to remain approximately fixed when local pose changes.
- [ ] Launch while disarmed with `enable_target_publish=true` only after confirming no `/precision_landing` controller node. Verify the new node publishes at at most 10 Hz to `/mavros/landing_target/pose`, publishes nothing to `/mavros/setpoint_raw/local`, and stops target publication after simulated/physical vision loss exceeds 0.2 seconds.
- [ ] Confirm touchdown-log code using synthetic ExtendedState unit/rostest inputs; do not perform an autonomous flight in this implementation session.

### Task 7: Finish the Existing Joint-PnP Branch and Publish

**Files:**
- All current feature-branch changes

- [ ] Run `git diff --check` and scan source for prohibited service/mode calls.
- [ ] Run fresh complete Jetson tests, OpenCV linkage checks, and both observation-mode launch checks.
- [ ] Commit implementation with `feat: add PX4 v1.12 native precision landing target`.
- [ ] Push `feature/apriltag-board-joint-pnp` to `origin` without changing remote `master` or `main`.
- [ ] Report branch, commit SHA, build commands, both A/B observation commands, PX4 capability blocker, and remaining manual ground/flight validation.
