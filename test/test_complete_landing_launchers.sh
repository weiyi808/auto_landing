#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEST_ROOT="$(mktemp -d)"
trap 'rm -rf "$TEST_ROOT"' EXIT

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

assert_contains() {
  local file="$1"
  local pattern="$2"
  local shell_escaped_pattern="${pattern// /\\ }"
  if ! grep -Fq -- "$pattern" "$file" &&
      ! grep -Fq -- "$shell_escaped_pattern" "$file"; then
    fail "missing '$pattern' in $file"
  fi
}

assert_not_contains() {
  local file="$1"
  local pattern="$2"
  if grep -Fq -- "$pattern" "$file"; then
    fail "unexpected '$pattern' in $file"
  fi
}

mkdir -p "$TEST_ROOT/ws/devel" "$TEST_ROOT/state" "$TEST_ROOT/bin"
touch "$TEST_ROOT/ros_setup.bash" "$TEST_ROOT/ws/devel/setup.bash" \
  "$TEST_ROOT/ttyTHS0"
cp "$REPO_ROOT"/test/launcher_stubs/* "$TEST_ROOT/bin/"
chmod +x "$TEST_ROOT/bin"/*

export PATH="$TEST_ROOT/bin:$PATH"
export AUTOFLY_WS="$TEST_ROOT/ws"
export ROS_SETUP="$TEST_ROOT/ros_setup.bash"
export FCU_DEVICE="$TEST_ROOT/ttyTHS0"
export FCU_URL="$TEST_ROOT/ttyTHS0:3000000"
export LANDING_LAUNCHER_DRY_RUN=1
export LANDING_LAUNCHER_COMMAND_LOG="$TEST_ROOT/commands.log"
export LANDING_LAUNCHER_STATE_DIR="$TEST_ROOT/state"
export STARTUP_TIMEOUT_S=1

companion_launch="$REPO_ROOT/src/precision_landing/launch/d455_companion_precision_landing.launch"
grep -Fq 'if="$(arg enable_flight)"' "$companion_launch" || \
  fail "companion test mode must not start the control publisher"

run_variant() {
  local script="$1"
  local session="$2"
  local launch="$3"
  local enable_name="$4"
  local layout="$5"

  [[ -x "$REPO_ROOT/$script" ]] || fail "$script is missing or not executable"

  : > "$LANDING_LAUNCHER_COMMAND_LOG"
  "$REPO_ROOT/$script"
  assert_contains "$LANDING_LAUNCHER_COMMAND_LOG" "roslaunch mavros px4.launch"
  assert_contains "$LANDING_LAUNCHER_COMMAND_LOG" "roslaunch realsense2_camera rs_camera.launch"
  assert_contains "$LANDING_LAUNCHER_COMMAND_LOG" "roslaunch fast_lio mapping_mid360andmaping.launch"
  assert_contains "$LANDING_LAUNCHER_COMMAND_LOG" "roslaunch fly_utils px4_pos_estimator.launch"
  assert_contains "$LANDING_LAUNCHER_COMMAND_LOG" "$launch"
  assert_contains "$LANDING_LAUNCHER_COMMAND_LOG" "$enable_name:=true"
  if [[ "$layout" == "split" ]]; then
    assert_contains "$LANDING_LAUNCHER_COMMAND_LOG" "d455_vision_only.launch"
    assert_contains "$LANDING_LAUNCHER_COMMAND_LOG" "precision_landing_control.launch"
  fi
  for prohibited in exploration_manager ego_planner px4_replan_sender auto_control pubcmd; do
    assert_not_contains "$LANDING_LAUNCHER_COMMAND_LOG" "$prohibited"
  done

  "$REPO_ROOT/$script" stop
  assert_contains "$LANDING_LAUNCHER_COMMAND_LOG" "tmux list-panes -s -t $session"
  assert_contains "$LANDING_LAUNCHER_COMMAND_LOG" "tmux kill-session -t $session"

  : > "$LANDING_LAUNCHER_COMMAND_LOG"
  "$REPO_ROOT/$script" test
  assert_contains "$LANDING_LAUNCHER_COMMAND_LOG" "mapping_mid360andmaping.launch"
  assert_contains "$LANDING_LAUNCHER_COMMAND_LOG" "px4_pos_estimator.launch"
  if [[ "$layout" == "split" ]]; then
    assert_contains "$LANDING_LAUNCHER_COMMAND_LOG" "d455_vision_only.launch"
    assert_not_contains "$LANDING_LAUNCHER_COMMAND_LOG" "precision_landing_control.launch"
  else
    assert_contains "$LANDING_LAUNCHER_COMMAND_LOG" "$enable_name:=false"
  fi

  : > "$LANDING_LAUNCHER_COMMAND_LOG"
  "$REPO_ROOT/$script" status
  assert_contains "$LANDING_LAUNCHER_COMMAND_LOG" "tmux list-windows -t $session"

  : > "$LANDING_LAUNCHER_COMMAND_LOG"
  "$REPO_ROOT/$script" stop
  assert_contains "$LANDING_LAUNCHER_COMMAND_LOG" "tmux list-panes -s -t $session"
  assert_contains "$LANDING_LAUNCHER_COMMAND_LOG" "tmux kill-session -t $session"

  if "$REPO_ROOT/$script" invalid-argument >"$TEST_ROOT/invalid.out" 2>&1; then
    fail "$script accepted invalid argument"
  fi
  assert_contains "$TEST_ROOT/invalid.out" "用法"
}

run_variant run_companion_precision_landing.sh landing_companion \
  precision_landing_control.launch enable_flight split
run_variant run_px4_native_precision_landing.sh landing_px4_native \
  d455_px4_native_precision_landing.launch enable_target_publish combined

echo "PASS: complete landing launcher contract"
