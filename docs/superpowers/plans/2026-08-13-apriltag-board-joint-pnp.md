# AprilTag Board Joint-PnP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace per-tag center averaging with one board-level OpenCV 4.5.5 PnP estimate whose origin is always the physical center of ID 0.

**Architecture:** A pure board-geometry module maps each supported tag ID to four board-frame points in OpenCV's detected canonical corner order. A board-pose estimator collects all visible correspondences, runs one `SOLVEPNP_ITERATIVE`, validates depth and RMS reprojection error, and returns ID 0 directly as `tvec`. The vision/debug ROS node consumes that result while the existing FRD controller and `enable_flight` safety gate remain unchanged.

**Tech Stack:** ROS1 Noetic, C++14, OpenCV 4.5.5 aruco/calib3d, Eigen3, MAVROS, GoogleTest.

---

### Task 1: Import the Safety-Gated Precision Landing Package

**Files:**
- Create: `src/precision_landing/CMakeLists.txt`
- Create: `src/precision_landing/package.xml`
- Create: `src/precision_landing/include/precision_landing/controller_core.hpp`
- Create: `src/precision_landing/src/controller_core.cpp`
- Create: `src/precision_landing/src/precision_landing_node.cpp`
- Create: `src/precision_landing/test/test_controller_core.cpp`

- [ ] Copy the already validated safety-gated controller into the repository, preserving `FRAME_BODY_NED`, `body_frd = (-camera_y, camera_x, camera_z)`, `enable_flight=false`, and the prohibition on arming/mode/LAND service calls.
- [ ] Configure CMake with exact OpenCV 4.5.5 from `/usr/local/opencv-455-cuda/lib/cmake/opencv4`, without `apriltag_ros` or `cv_bridge` catkin dependencies.
- [ ] Run `git grep -nE 'set_mode|arming|CommandBool|CommandTOL|cv_bridge|apriltag_ros' -- src/precision_landing` and expect no active source dependency or flight-mode service call.

### Task 2: Define Board Geometry in Detected-Corner Order

**Files:**
- Create: `src/precision_landing/include/precision_landing/board_geometry.hpp`
- Create: `src/precision_landing/test/test_board_geometry.cpp`

- [ ] Write failing tests for `blackMarkerSizeMeters(id, scale)` and `boardObjectCorners(id, scale, &points)`. Anchor ID 10 at page center `(-0.101,-0.0575)` and require detected-corner order `BR, BL, TL, TR` with black side 0.068 m; also cover ID 20 at 0.0288 m and ID 0 at 0.0144 m.
- [ ] Run `catkin_make test_board_geometry && ./devel/lib/precision_landing/test_board_geometry`; expect compilation failure because the new geometry API does not exist.
- [ ] Implement a table with page-frame centers for IDs 10-13, 20-23, and 0. Multiply both center offsets and black-square side lengths by `scale`. Return four points as:

```cpp
{cx + h, cy + h, 0.0},  // detector corner 0: page bottom-right
{cx - h, cy + h, 0.0},  // detector corner 1: page bottom-left
{cx - h, cy - h, 0.0},  // detector corner 2: page top-left
{cx + h, cy - h, 0.0}   // detector corner 3: page top-right
```

- [ ] Re-run the geometry test and expect all cases to pass.

### Task 3: Add the Joint Board-Pose Estimator

**Files:**
- Create: `src/precision_landing/include/precision_landing/board_pose_estimator.hpp`
- Create: `src/precision_landing/src/board_pose_estimator.cpp`
- Create: `src/precision_landing/test/test_board_pose_estimator.cpp`
- Modify: `src/precision_landing/CMakeLists.txt`

- [ ] Write a failing synthetic test that projects one outer tag from a known tilted board pose, calls `estimateBoardPose`, and requires recovered ID 0 translation within 1 mm and RMS below 0.1 px.
- [ ] Add a second failing test using IDs 10, 21, 22, and 13 simultaneously and require the same board origin within 1 mm.
- [ ] Run `catkin_make test_board_pose_estimator`; expect failure because `estimateBoardPose` is missing.
- [ ] Implement `BoardPoseEstimate estimateBoardPose(ids, image_corners, K, D, board_scale, max_rms_px)` that skips unsupported IDs, collects matching points, calls `cv::solvePnP(..., cv::SOLVEPNP_ITERATIVE)`, computes RMS with `cv::projectPoints`, and rejects non-finite values, `z <= 0`, or RMS above the threshold.
- [ ] Run both estimator tests and the geometry test; expect all to pass.

### Task 4: Replace Per-Tag Fusion in the Vision Node

**Files:**
- Create: `src/precision_landing/src/debug_overlay_node.cpp`
- Create: `src/precision_landing/config/precision_landing.yaml`
- Create: `src/precision_landing/launch/d455_precision_landing.launch`
- Modify: `src/precision_landing/CMakeLists.txt`

- [ ] Subscribe directly to D455 `sensor_msgs/Image` and `CameraInfo`, detect `DICT_APRILTAG_36h11`, and call the joint estimator once per frame.
- [ ] Publish estimator `tvec` as `/precision_landing/board_center_camera`; do not apply tag-local offsets or average tag poses.
- [ ] Add `board_scale` (default 1.0) and `max_reprojection_error_px` (default 3.0) ROS parameters.
- [ ] Draw tag outlines/IDs, a large red projected board origin, FRD error, visible tag count, RMS, guide/actual commands, and direction arrow. On rejection, draw a reason and publish no center pose.
- [ ] Keep launch default `enable_flight=false`; verify launch contains no arming, set-mode, or LAND node/service.

### Task 5: Add Reproducible OpenCV 4.5.5 Build and Usage Docs

**Files:**
- Create: `build_precision_landing_opencv455.sh`
- Create: `src/precision_landing/README.md`
- Create: `docs/superpowers/specs/2026-08-13-apriltag-board-joint-pnp-design.md`

- [ ] Add an executable build script that pins `OpenCV_DIR=/usr/local/opencv-455-cuda/lib/cmake/opencv4`, rebuilds only the generated package cache, and rejects `.so.4.2` or missing `.so.405` linkage.
- [ ] Document `board_scale = measured ID0 black-square side / 14.4 mm`, observation-mode launch, debug topic, log path, and the rule not to switch OFFBOARD until ground direction validation passes.
- [ ] Run `bash -n build_precision_landing_opencv455.sh`; expect exit 0.

### Task 6: Jetson Red-Green, Runtime, and Safety Verification

**Files:**
- Sync repository package into `/home/wxh/nb_code/autofly_ws/src/precision_landing`

- [ ] Sync tests before implementation as required by Tasks 2-3 and record their expected RED failures.
- [ ] Sync the implementation and run `/home/wxh/nb_code/autofly_ws/build_precision_landing_opencv455.sh`; expect `SUCCESS` and only `.so.405` OpenCV libraries.
- [ ] Run `catkin_make run_tests_precision_landing` and `catkin_test_results build/test_results/precision_landing`; expect zero errors and zero failures.
- [ ] Restart only the precision-landing tmux session with `enable_flight:=false` while leaving D455 and MAVROS running.
- [ ] Confirm `/precision_landing/debug/image_raw` publishes near the D455 frame rate, `/mavros/setpoint_raw/local` uses coordinate frame 8 with zero XYZ velocity, and no precision-landing process links OpenCV 4.2.
- [ ] Observe a frame with multiple IDs and confirm RMS is below threshold and the red point projects to the physical ID 0 center.

### Task 7: Commit and Publish the Feature Branch

**Files:**
- All files introduced above

- [ ] Run `git diff --check`, `git status --short`, the complete Jetson tests, and the runtime safety checks once more.
- [ ] Commit with message `feat: add OpenCV 4.5 joint-PnP precision landing`.
- [ ] Push `feature/apriltag-board-joint-pnp` to `origin` without modifying remote `master` or `main`.
- [ ] Report the exact branch name, commit SHA, build command, observation launch command, and remaining ground-validation requirement.
