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
# 停止本脚本拥有的全部进程：
#   ./run_companion_precision_landing.sh stop
#
# 启动链路：MAVROS -> D455 -> MID360/FAST-LIO -> /drone_Odometry ->
# px4_pos_estimator -> /mavros/vision_pose/pose -> PX4 本地位置 -> 伴随降落。
# 不启动 Falcon/Ego 探索规划、px4_replan_sender、auto_control 或 pubcmd；
# 本脚本也不会自动解锁、切换飞行模式或调用 LAND。

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
DRY_RUN="${LANDING_LAUNCHER_DRY_RUN:-0}"

usage() {
  cat <<'EOF'
用法：
  ./run_companion_precision_landing.sh          # 完整运行，启用伴随降落输出
  ./run_companion_precision_landing.sh test     # 完整测试链路，禁用降落输出
  ./run_companion_precision_landing.sh status   # 查看会话、定位和控制状态
  ./run_companion_precision_landing.sh stop     # 仅停止本脚本创建的会话
EOF
}

die() {
  echo "[伴随降落] 错误：$*" >&2
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

wait_for_ros_master() {
  local deadline=$(( $(date +%s) + STARTUP_TIMEOUT_S ))
  until rosparam get /run_id >/dev/null 2>&1; do
    deadline_reached "$deadline" && return 1
    sleep 0.5
  done
}

wait_for_fcu() {
  local deadline=$(( $(date +%s) + STARTUP_TIMEOUT_S ))
  until timeout 2 rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q 'connected: True'; do
    deadline_reached "$deadline" && return 1
    sleep 0.5
  done
}

wait_for_topic_message() {
  local topic="$1"
  local deadline=$(( $(date +%s) + STARTUP_TIMEOUT_S ))
  until timeout 2 rostopic echo -n 1 "$topic" >/dev/null 2>&1; do
    deadline_reached "$deadline" && return 1
    sleep 0.5
  done
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
  tmux new-window -d -t "$SESSION_NAME" -n "$name" \
    "bash -lc $quoted_command"
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

stop_session() {
  require_command tmux
  if ! session_exists "$SESSION_NAME"; then
    echo "[伴随降落] 会话已经停止：$SESSION_NAME"
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
  echo "[伴随降落] 已停止会话：$SESSION_NAME"
}

mode="run"
if (( $# > 1 )); then
  usage
  exit 2
elif (( $# == 1 )); then
  case "$1" in
    test|status|stop) mode="$1" ;;
    *) usage; exit 2 ;;
  esac
fi

if [[ "$mode" == "status" ]]; then
  show_status
  exit $?
elif [[ "$mode" == "stop" ]]; then
  stop_session
  exit 0
fi

enable_flight="true"
[[ "$mode" == "test" ]] && enable_flight="false"

require_file "$ROS_SETUP"
require_file "$AUTOFLY_WS/devel/setup.bash"
source_environment
for command_name in tmux rosparam rosnode rostopic rospack rs-enumerate-devices timeout; do
  require_command "$command_name"
done
for package_name in mavros realsense2_camera fast_lio fly_utils precision_landing; do
  require_package "$package_name"
done
[[ -r "$FCU_DEVICE" && -w "$FCU_DEVICE" ]] || \
  die "飞控串口不可读写：$FCU_DEVICE（请检查 dialout 权限）"
rs-enumerate-devices -s 2>/dev/null | grep -Fq "$D455_SERIAL" || \
  die "没有找到 D455：$D455_SERIAL"

session_exists "$SESSION_NAME" && die "会话已存在：$SESSION_NAME"
session_exists "$OTHER_SESSION_NAME" && die "另一降落方案正在运行：$OTHER_SESSION_NAME"

if rosparam get /run_id >/dev/null 2>&1; then
  for conflicting_node in /mavros /camera/realsense2_camera /precision_landing \
      /px4_landing_target /px4_pos_estimator /laserMapping; do
    node_exists "$conflicting_node" && die "发现冲突节点：$conflicting_node"
  done
fi

tmux new-session -d -s "$SESSION_NAME" -n overview \
  "bash -lc 'echo 伴随控制精准降落启动会话; exec bash'"

if ! rosparam get /run_id >/dev/null 2>&1; then
  start_window roscore "roscore"
  wait_for_ros_master || die "ROS master 启动超时，请查看 $SESSION_NAME:roscore"
fi

mavros_command="roslaunch mavros px4.launch fcu_url:=$FCU_URL"
[[ -n "$GCS_URL" ]] && mavros_command+=" gcs_url:=$GCS_URL"
start_window mavros "$mavros_command"
wait_for_fcu || die "MAVROS 未在 ${STARTUP_TIMEOUT_S}s 内连接飞控，请查看 $SESSION_NAME:mavros"

start_window d455 "roslaunch realsense2_camera rs_camera.launch camera:=camera serial_no:=$D455_SERIAL enable_color:=true color_width:=640 color_height:=480 color_fps:=30 enable_depth:=false enable_infra1:=false enable_infra2:=false initial_reset:=true"
wait_for_topic_message /camera/color/image_raw || \
  die "D455 彩色图像超时，请查看 $SESSION_NAME:d455"
wait_for_topic_message /camera/color/camera_info || \
  die "D455 CameraInfo 超时，请查看 $SESSION_NAME:d455"

start_window fastlio "roslaunch fast_lio mapping_mid360andmaping.launch"
wait_for_topic_message /drone_Odometry || \
  die "FAST-LIO /drone_Odometry 超时，请查看 $SESSION_NAME:fastlio"

start_window vision "roslaunch fly_utils px4_pos_estimator.launch"
wait_for_topic_message /mavros/vision_pose/pose || \
  die "视觉位置传输超时，请查看 $SESSION_NAME:vision"
wait_for_topic_message /mavros/local_position/pose || \
  die "PX4 本地位置超时，请检查 EKF2 外部视觉配置和 $SESSION_NAME:vision"

start_window landing "roslaunch precision_landing d455_companion_precision_landing.launch enable_flight:=$enable_flight board_scale:=$BOARD_SCALE"
if [[ "$enable_flight" == "true" ]]; then
  wait_for_topic_message /precision_landing/status || \
    die "伴随降落节点状态超时，请查看 $SESSION_NAME:landing"
else
  wait_for_topic_message /precision_landing/debug/image_raw || \
    die "伴随降落视觉状态超时，请查看 $SESSION_NAME:landing"
fi

start_window monitor "watch -n 1 'rostopic echo -n 1 /mavros/state; rostopic echo -n 1 /precision_landing/status'"

echo "[伴随降落] 完整链路已启动，enable_flight=$enable_flight"
echo "[伴随降落] 查看窗口：tmux attach -t $SESSION_NAME"
if [[ "$mode" == "test" ]]; then
  echo "[伴随降落] 当前为 test 模式，不输出飞行控制。"
else
  echo "[伴随降落] 控制输出已允许；解锁和 OFFBOARD 切换仍由飞行员手动完成。"
fi
