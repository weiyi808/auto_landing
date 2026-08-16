#!/usr/bin/env bash
# 伴随控制精准降落完整启动器
#
# 直接完整运行（启用伴随降落控制输出，飞行员仍手动解锁和切换 OFFBOARD）：
#   ./run_companion_precision_landing.sh
# 地面测试（完整启动传感器和定位，但禁用降落控制输出）：
#   ./run_companion_precision_landing.sh test
# 查看状态和日志窗口：
#   ./run_companion_precision_landing.sh status
#   tmux attach -t landing_companion
# 只重启降落控制节点（改 yaml 后用这个，不必停相机/定位/MAVROS）：
#   ./run_companion_precision_landing.sh restart-landing
# 停止本脚本拥有的全部进程：
#   ./run_companion_precision_landing.sh stop
#
# 启动链路：MAVROS -> D455 -> MID360/FAST-LIO -> /drone_Odometry ->
# px4_pos_estimator -> /mavros/vision_pose/pose -> PX4 本地位置 -> 伴随降落。
# 不启动 Falcon/Ego 探索规划、px4_replan_sender、auto_control 或 pubcmd；
# 本脚本不自动解锁、不自动切 OFFBOARD。到达终点后由 precision_landing 节点请求 AUTO.LAND。

set -euo pipefail

SESSION_NAME="landing_companion"
OTHER_SESSION_NAME="landing_px4_native"
AUTOFLY_WS="${AUTOFLY_WS:-/home/wxh/nb_code/autofly_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
FCU_DEVICE="${FCU_DEVICE:-/dev/ttyTHS0}"
FCU_URL="${FCU_URL:-/dev/ttyTHS0:3000000}"
GCS_URL="${GCS_URL-udp://@192.168.20.57}"
D455_SERIAL="${D455_SERIAL:-046322251249}"
BOARD_SCALE="${BOARD_SCALE:-1.0}"
STARTUP_TIMEOUT_S="${STARTUP_TIMEOUT_S:-30}"
D455_STARTUP_TIMEOUT_S="${D455_STARTUP_TIMEOUT_S:-90}"
FASTLIO_STARTUP_TIMEOUT_S="${FASTLIO_STARTUP_TIMEOUT_S:-60}"
DRY_RUN="${LANDING_LAUNCHER_DRY_RUN:-0}"

usage() {
  cat <<'EOF'
用法：
  ./run_companion_precision_landing.sh          # 完整运行，启用伴随降落输出
  ./run_companion_precision_landing.sh test     # 完整测试链路，禁用降落输出
  ./run_companion_precision_landing.sh status   # 查看会话、定位和控制状态
  ./run_companion_precision_landing.sh restart-landing  # 只重启降落控制节点，重载 yaml
  ./run_companion_precision_landing.sh stop     # 仅停止本脚本创建的会话
EOF
}

stamp() {
  date +%H:%M:%S
}

log() {
  echo "[伴随降落 $(stamp)] $*"
}

log_err() {
  echo "[伴随降落 $(stamp)] 错误：$*" >&2
}

die() {
  log_err "$*"
  exit 1
}

require_file() {
  [[ -f "$1" ]] || die "缺少文件：$1"
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "缺少命令：$1"
}

require_package() {
  rospack find "$1" >/dev/null 2>&1 || die "缺少 ROS 包：$1"
}

source_environment() {
  set +u
  source "$ROS_SETUP"
  source "$AUTOFLY_WS/devel/setup.bash"
  set -u
}

session_exists() {
  tmux has-session -t "$1" 2>/dev/null
}

node_exists() {
  rosnode list 2>/dev/null | grep -Fxq "$1"
}

deadline_reached() {
  (( $(date +%s) >= $1 ))
}

dump_wait_failure() {
  local description="$1"
  local topic="${2:-}"
  local elapsed="$3"
  local timeout_s="$4"
  log_err "$description 失败（已等 ${elapsed}s / ${timeout_s}s）"
  echo "-------- 启动失败诊断：$description --------" >&2
  if [[ -n "$topic" ]]; then
    echo "话题：$topic" >&2
    if rostopic list 2>/dev/null | grep -Fxq "$topic"; then
      echo "话题已注册，但一直没有收到消息。" >&2
      rostopic info "$topic" >&2 || true
    else
      echo "话题尚未出现（节点可能没起来，或名字不对）。" >&2
      echo "当前话题：" >&2
      rostopic list 2>/dev/null >&2 || echo "rostopic list 失败" >&2
    fi
  fi
  echo "当前节点：" >&2
  rosnode list 2>/dev/null >&2 || echo "rosnode list 失败" >&2
  echo "tmux 窗口：" >&2
  tmux list-windows -t "$SESSION_NAME" >&2 || true
  echo "请执行：tmux attach -t $SESSION_NAME 查看对应窗口的报错" >&2
  echo "----------------------------------------------" >&2
}

wait_for_ros_master() {
  local timeout_s="${1:-$STARTUP_TIMEOUT_S}"
  local start elapsed
  local deadline=$(( $(date +%s) + timeout_s ))
  start=$(date +%s)
  log "等待 ROS master ..."
  until rosparam get /run_id >/dev/null 2>&1; do
    elapsed=$(( $(date +%s) - start ))
    if deadline_reached "$deadline"; then
      dump_wait_failure "ROS master" "" "$elapsed" "$timeout_s"
      return 1
    fi
    log "仍在等待 ROS master（已 ${elapsed}s / ${timeout_s}s）"
    sleep 0.5
  done
  log "ROS master 已就绪（$(( $(date +%s) - start ))s）"
}

wait_for_fcu() {
  local timeout_s="${1:-$STARTUP_TIMEOUT_S}"
  local start elapsed state connected
  local deadline=$(( $(date +%s) + timeout_s ))
  start=$(date +%s)
  log "等待飞控连接 /mavros/state ..."
  while true; do
    state="$(timeout 2 rostopic echo -n 1 /mavros/state 2>/dev/null || true)"
    if printf '%s\n' "$state" | grep -q 'connected: True'; then
      log "飞控已连接（$(( $(date +%s) - start ))s）"
      return 0
    fi
    elapsed=$(( $(date +%s) - start ))
    if deadline_reached "$deadline"; then
      dump_wait_failure "飞控连接" "/mavros/state" "$elapsed" "$timeout_s"
      if [[ -n "$state" ]]; then
        echo "最后一次 /mavros/state：" >&2
        printf '%s\n' "$state" >&2
      else
        echo "一直没有收到 /mavros/state。" >&2
      fi
      return 1
    fi
    connected="$(printf '%s\n' "$state" | grep -E 'connected:|mode:' || true)"
    if [[ -n "$connected" ]]; then
      log "飞控尚未连接（已 ${elapsed}s / ${timeout_s}s）：$connected"
    else
      log "仍在等待 /mavros/state（已 ${elapsed}s / ${timeout_s}s，尚无消息）"
    fi
    sleep 0.5
  done
}

wait_for_topic_message() {
  local topic="$1"
  local description="${2:-$topic}"
  local timeout_s="${3:-$STARTUP_TIMEOUT_S}"
  local start elapsed
  local deadline=$(( $(date +%s) + timeout_s ))
  start=$(date +%s)
  log "等待 $description（$topic）..."
  until timeout 2 rostopic echo -n 1 "$topic" >/dev/null 2>&1; do
    elapsed=$(( $(date +%s) - start ))
    if deadline_reached "$deadline"; then
      dump_wait_failure "$description" "$topic" "$elapsed" "$timeout_s"
      return 1
    fi
    log "仍在等待 $description（已 ${elapsed}s / ${timeout_s}s）"
    sleep 0.5
  done
  log "就绪：$description（$(( $(date +%s) - start ))s）"
}

shell_prefix() {
  printf 'source %q && source %q' "$ROS_SETUP" "$AUTOFLY_WS/devel/setup.bash"
}

start_window() {
  local name="$1"
  local command_text="$2"
  local prefix full_command quoted_command
  prefix="$(shell_prefix)"
  full_command="$prefix && $command_text; exec bash"
  printf -v quoted_command '%q' "$full_command"
  log "启动窗口 $name：$command_text"
  tmux new-window -d -t "$SESSION_NAME" -n "$name" \
    "bash -lc $quoted_command"
}

window_exists() {
  tmux list-windows -t "$SESSION_NAME" -F '#{window_name}' 2>/dev/null | grep -Fxq "$1"
}

store_session_option() {
  tmux set-option -t "$SESSION_NAME" "$1" "$2"
}

read_session_option() {
  tmux show-option -qv -t "$SESSION_NAME" "$1" 2>/dev/null || true
}

control_launch_command() {
  printf 'roslaunch precision_landing precision_landing_control.launch enable_flight:=true'
}

vision_launch_command() {
  printf 'roslaunch precision_landing d455_vision_only.launch board_scale:=%s' "$1"
}

combined_launch_command() {
  printf 'roslaunch precision_landing d455_companion_precision_landing.launch enable_flight:=%s board_scale:=%s' "$1" "$2"
}

kill_named_window() {
  local name="$1"
  window_exists "$name" || return 0
  tmux send-keys -t "$SESSION_NAME:$name" C-c
  if [[ "$DRY_RUN" != "1" ]]; then
    sleep 1
  fi
  tmux kill-window -t "$SESSION_NAME:$name" 2>/dev/null || true
}

wait_for_node_gone() {
  local node_name="$1"
  local deadline=$(( $(date +%s) + 8 ))
  while node_exists "$node_name"; do
    rosnode kill "$node_name" >/dev/null 2>&1 || true
    deadline_reached "$deadline" && return 1
    sleep 0.3
  done
}

start_landing_windows() {
  local enable_flight="$1"
  local board_scale="$2"
  store_session_option @companion_enable_flight "$enable_flight"
  store_session_option @companion_board_scale "$board_scale"
  start_window landing_vision "$(vision_launch_command "$board_scale")"
  if [[ "$enable_flight" == "true" ]]; then
    start_window landing "$(control_launch_command)"
  fi
}

show_topic_rate() {
  local topic="$1"
  echo "--- $topic"
  timeout 3 rostopic hz "$topic" 2>/dev/null || true
}

show_status() {
  require_command tmux
  if ! session_exists "$SESSION_NAME"; then
    echo "[伴随降落] 会话未运行：$SESSION_NAME"
    return 1
  fi
  source_environment
  echo "[伴随降落] tmux 窗口"
  tmux list-windows -t "$SESSION_NAME"
  echo "[伴随降落] 飞控状态"
  timeout 3 rostopic echo -n 1 /mavros/state || true
  show_topic_rate /drone_Odometry
  show_topic_rate /mavros/vision_pose/pose
  show_topic_rate /mavros/local_position/pose
  show_topic_rate /precision_landing/debug/image_raw
  echo "[伴随降落] 降落状态"
  timeout 3 rostopic echo -n 1 /precision_landing/status || true
  echo "[伴随降落] BODY_NED 输出发布者"
  rostopic info /mavros/setpoint_raw/local || true
}

restart_landing() {
  require_command tmux
  session_exists "$SESSION_NAME" || die "会话未运行：$SESSION_NAME。请先完整启动。"
  source_environment
  local enable_flight board_scale
  enable_flight="$(read_session_option @companion_enable_flight)"
  board_scale="$(read_session_option @companion_board_scale)"
  [[ -n "$enable_flight" ]] || enable_flight="true"
  [[ -n "$board_scale" ]] || board_scale="$BOARD_SCALE"

  log "重启降落视觉+控制节点，enable_flight=$enable_flight（MAVROS/相机/定位保持）"
  if [[ "$enable_flight" == "true" ]]; then
    local use_split=0
    window_exists landing_vision && use_split=1
    kill_named_window landing
    if (( use_split )); then
      kill_named_window landing_vision
    fi
    if [[ "$DRY_RUN" != "1" ]]; then
      wait_for_node_gone /precision_landing || \
        log_err "旧控制节点仍在，继续拉起新节点"
      wait_for_node_gone /precision_landing_debug_overlay || true
    fi
    if (( use_split )); then
      start_window landing_vision "$(vision_launch_command "$board_scale")"
      start_window landing "$(control_launch_command)"
    else
      log "当前是旧窗口布局，视觉叠加会一起重启"
      start_window landing "$(combined_launch_command "$enable_flight" "$board_scale")"
    fi
    if [[ "$DRY_RUN" != "1" ]]; then
      wait_for_topic_message /precision_landing/debug/image_raw "降落调试画面" || \
        die "降落视觉重启失败，请查看 $SESSION_NAME:landing_vision"
      wait_for_topic_message /precision_landing/status "降落控制状态" || \
        die "降落控制节点重启失败，请查看 $SESSION_NAME:landing"
    fi
  else
    kill_named_window landing_vision
    if window_exists landing; then
      kill_named_window landing
    fi
    start_window landing_vision "$(vision_launch_command "$board_scale")"
    if [[ "$DRY_RUN" != "1" ]]; then
      wait_for_topic_message /precision_landing/debug/image_raw "降落调试画面" || \
        die "降落视觉重启失败，请查看 $SESSION_NAME:landing_vision"
    fi
  fi
  log "降落节点已重启，yaml 已重新加载"
}

stop_session() {
  require_command tmux
  if ! session_exists "$SESSION_NAME"; then
    log "会话已经停止：$SESSION_NAME"
    return 0
  fi
  local pane pane_pid
  while read -r pane pane_pid; do
    [[ -n "$pane" ]] || continue
    tmux send-keys -t "$pane" C-c
  done < <(tmux list-panes -s -t "$SESSION_NAME" -F '#{pane_id} #{pane_pid}')
  if [[ "$DRY_RUN" != "1" ]]; then
    sleep 2
  fi
  tmux kill-session -t "$SESSION_NAME"
  log "已停止会话：$SESSION_NAME"
}

mode="run"
if (( $# > 1 )); then
  usage
  exit 2
elif (( $# == 1 )); then
  case "$1" in
    test|status|stop|restart-landing|reload) mode="$1" ;;
    *) usage; exit 2 ;;
  esac
fi

if [[ "$mode" == "status" ]]; then
  show_status
  exit $?
elif [[ "$mode" == "stop" ]]; then
  stop_session
  exit 0
elif [[ "$mode" == "restart-landing" || "$mode" == "reload" ]]; then
  restart_landing
  exit 0
fi

enable_flight="true"
[[ "$mode" == "test" ]] && enable_flight="false"

log "开始启动，enable_flight=$enable_flight"
require_file "$ROS_SETUP"
require_file "$AUTOFLY_WS/devel/setup.bash"
log "加载 ROS 环境..."
source_environment
for command_name in tmux rosparam rosnode rostopic rospack rs-enumerate-devices timeout; do
  require_command "$command_name"
done
for package_name in mavros realsense2_camera fast_lio fly_utils precision_landing; do
  log "检查 ROS 包 $package_name ..."
  require_package "$package_name"
done
[[ -r "$FCU_DEVICE" && -w "$FCU_DEVICE" ]] || \
  die "飞控串口不可读写：$FCU_DEVICE（请检查 dialout 权限）"
log "枚举 RealSense（最多 20s），寻找 $D455_SERIAL ..."
if ! timeout 20 rs-enumerate-devices -s 2>/dev/null | grep -Fq "$D455_SERIAL"; then
  die "没有找到 D455：$D455_SERIAL（设备未插好，或 rs-enumerate-devices 超时/卡住）"
fi
log "已找到 D455：$D455_SERIAL"

session_exists "$SESSION_NAME" && die "会话已存在：$SESSION_NAME"
session_exists "$OTHER_SESSION_NAME" && die "另一降落方案正在运行：$OTHER_SESSION_NAME"

if rosparam get /run_id >/dev/null 2>&1; then
  log "检测到已有 ROS master，检查冲突节点..."
  for conflicting_node in /mavros /camera/realsense2_camera /precision_landing \
      /px4_landing_target /px4_pos_estimator /laserMapping; do
    node_exists "$conflicting_node" && die "发现冲突节点：$conflicting_node"
  done
fi

log "创建 tmux 会话 $SESSION_NAME"
tmux new-session -d -s "$SESSION_NAME" -n overview \
  "bash -lc 'echo 伴随控制精准降落启动会话; exec bash'"

if ! rosparam get /run_id >/dev/null 2>&1; then
  start_window roscore "roscore"
  wait_for_ros_master || die "ROS master 启动失败，请查看 $SESSION_NAME:roscore"
else
  log "复用已有 ROS master"
fi

mavros_command="roslaunch mavros px4.launch fcu_url:=$FCU_URL"
[[ -n "$GCS_URL" ]] && mavros_command+=" gcs_url:=$GCS_URL"
start_window mavros "$mavros_command"
wait_for_fcu || die "飞控连接失败，请查看 $SESSION_NAME:mavros"

start_window d455 "roslaunch realsense2_camera rs_camera.launch camera:=camera serial_no:=$D455_SERIAL enable_color:=true color_width:=640 color_height:=480 color_fps:=30 enable_depth:=false enable_infra1:=false enable_infra2:=false initial_reset:=true"
wait_for_topic_message /camera/color/image_raw "D455 彩色图像" "$D455_STARTUP_TIMEOUT_S" || \
  die "D455 彩色图像失败（initial_reset 可能较慢），请查看 $SESSION_NAME:d455"
wait_for_topic_message /camera/color/camera_info "D455 CameraInfo" "$D455_STARTUP_TIMEOUT_S" || \
  die "D455 CameraInfo 失败，请查看 $SESSION_NAME:d455"

start_window fastlio "roslaunch fast_lio mapping_mid360andmaping.launch"
wait_for_topic_message /drone_Odometry "FAST-LIO 里程计" "$FASTLIO_STARTUP_TIMEOUT_S" || \
  die "FAST-LIO /drone_Odometry 失败，请查看 $SESSION_NAME:fastlio"

start_window vision "roslaunch fly_utils px4_pos_estimator.launch"
wait_for_topic_message /mavros/vision_pose/pose "视觉位置注入" || \
  die "视觉位置传输失败，请查看 $SESSION_NAME:vision"
wait_for_topic_message /mavros/local_position/pose "PX4 本地位置" || \
  die "PX4 本地位置失败，请检查 EKF2 外部视觉配置和 $SESSION_NAME:vision"

start_landing_windows "$enable_flight" "$BOARD_SCALE"
if [[ "$enable_flight" == "true" ]]; then
  wait_for_topic_message /precision_landing/debug/image_raw "降落调试画面" || \
    die "伴随降落视觉失败，请查看 $SESSION_NAME:landing_vision"
  wait_for_topic_message /precision_landing/status "降落控制状态" || \
    die "伴随降落节点失败，请查看 $SESSION_NAME:landing"
else
  wait_for_topic_message /precision_landing/debug/image_raw "降落调试画面" || \
    die "伴随降落视觉失败，请查看 $SESSION_NAME:landing_vision"
fi

start_window monitor "watch -n 1 'rostopic echo -n 1 /mavros/state; rostopic echo -n 1 /precision_landing/status'"

log "完整链路已启动，enable_flight=$enable_flight"
log "查看窗口：tmux attach -t $SESSION_NAME"
if [[ "$mode" == "test" ]]; then
  log "当前为 test 模式，不输出飞行控制。"
else
  log "控制输出已允许；解锁和 OFFBOARD 切换仍由飞行员手动完成。"
fi
