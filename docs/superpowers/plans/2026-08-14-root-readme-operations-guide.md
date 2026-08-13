# Root README Operations Guide Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a concise Chinese root README that accurately documents how to build, run, inspect, stop, and find logs for both uploaded precision-landing variants.

**Architecture:** The root README is an operator-first entry point that links to detailed package documentation but keeps all daily commands and log paths inline. Every command, topic, session name, default path, and safety statement is verified directly against the two launchers, launch files, and logger source before publication.

**Tech Stack:** Markdown, Bash command examples, ROS1 Noetic, tmux, GitHub.

---

### Task 1: Verify Documentation Facts From Source

**Files:**
- Read: `run_companion_precision_landing.sh`
- Read: `run_px4_native_precision_landing.sh`
- Read: `build_precision_landing_opencv455.sh`
- Read: `src/precision_landing/src/debug_overlay_node.cpp`
- Read: `src/precision_landing/src/px4_landing_target_node.cpp`
- Read: `src/precision_landing/launch/*.launch`

- [ ] Extract exact direct/test/status/stop commands, tmux session names, hardware defaults, workspace path, build command, launch arguments, debug topics, and the common localization chain.
- [ ] Extract the exact joint-PnP CSV path, PX4-native log directory, detail filename pattern, summary filename, and logged measurement meanings.
- [ ] Confirm test-mode output behavior from launch files: no companion control node when `enable_flight=false`, and no native landing-target publisher when `enable_target_publish=false`.
- [ ] Confirm neither complete launcher invokes arming, mode-change, LAND, Falcon exploration/control, or Ego planning commands.

### Task 2: Write the GitHub-Facing Root README

**Files:**
- Create: `README.md`

- [ ] Start with a Chinese project overview, supported hardware/software, and the exact localization flow.
- [ ] Put the OpenCV 4.5.5 build command before runtime commands.
- [ ] Document companion direct/test/status/stop/tmux commands and manual OFFBOARD behavior.
- [ ] Document PX4-native direct/test/status/stop/tmux commands, `LOCAL_NED`, manual PX4 mode switching, and the `PLD_*` capability prerequisite.
- [ ] State prominently that the variants are mutually exclusive and the scripts never arm, switch mode, or call LAND.
- [ ] Add debug image/topic commands and the three exact CSV log paths with a concise field-purpose description.
- [ ] Add the AprilTag A4 local asset paths and `board_scale = measured_ID0_black_side_mm / 14.4`.
- [ ] Add a compact troubleshooting table for timeout, existing session/node conflicts, D455 serial failure, missing `/drone_Odometry`, missing local PX4 pose, and tmux log inspection.
- [ ] Link to `src/precision_landing/README.md` for lower-level parameter details.

### Task 3: Verify, Commit, and Push

**Files:**
- Verify: `README.md`

- [ ] Compare all README command tokens and paths against source using `rg`.
- [ ] Run `bash test/test_complete_landing_launchers.sh`; require `PASS: complete landing launcher contract`.
- [ ] Run placeholder and stale-path scans, verify Markdown relative links resolve, and run `git diff --check`.
- [ ] Commit with `docs: add precision landing operations guide`.
- [ ] Push `feature/apriltag-board-joint-pnp` without modifying `master` or `main`.
- [ ] Report the README link, commit SHA, and the three recorded log locations.
