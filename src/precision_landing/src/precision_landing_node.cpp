#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "precision_landing/controller_core.hpp"
#include "precision_landing/heading_from_board.hpp"
#include "precision_landing/landing_target_anchor.hpp"

namespace {

float yawNedDeg360(double yaw_enu) {
  double deg = precision_landing::yawEnuToNed(yaw_enu) * 180.0 / M_PI;
  while (deg < 0.0) deg += 360.0;
  while (deg >= 360.0) deg -= 360.0;
  return static_cast<float>(deg);
}

bool setPx4Param(const std::string& name, float value) {
  ros::NodeHandle nh;
  auto client = nh.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
  mavros_msgs::ParamSet request;
  request.request.param_id = name;
  request.request.value.real = value;
  request.request.value.integer = 0;
  if (!client.call(request) || !request.response.success) {
    ROS_WARN("failed to set %s=%.1f", name.c_str(), value);
    return false;
  }
  ROS_INFO("set %s=%.1f", name.c_str(), value);
  return true;
}

float getPx4Param(const std::string& name, float fallback) {
  ros::NodeHandle nh;
  auto client = nh.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");
  mavros_msgs::ParamGet request;
  request.request.param_id = name;
  if (!client.call(request) || !request.response.success) {
    ROS_WARN("failed to get %s, use %.1f", name.c_str(), fallback);
    return fallback;
  }
  if (std::abs(request.response.value.real) > 1e-6) {
    return static_cast<float>(request.response.value.real);
  }
  return static_cast<float>(request.response.value.integer);
}

precision_landing::Limits loadLimits(const ros::NodeHandle& nh) {
  precision_landing::Limits limits;
  nh.param("tag_timeout_s", limits.tag_timeout_s, limits.tag_timeout_s);
  nh.param("vehicle_timeout_s", limits.vehicle_timeout_s,
           limits.vehicle_timeout_s);
  nh.param("align_radius_m", limits.align_radius_m, limits.align_radius_m);
  nh.param("final_radius_m", limits.final_radius_m, limits.final_radius_m);
  nh.param("final_height_m", limits.final_height_m, limits.final_height_m);
  nh.param("descend_start_height_m", limits.descend_start_height_m,
           limits.descend_start_height_m);
  nh.param("descent_xy_stop_m", limits.descent_xy_stop_m,
           limits.descent_xy_stop_m);
  nh.param("commit_height_m", limits.commit_height_m, limits.commit_height_m);
  nh.param("commit_down_vel_mps", limits.commit_down_vel_mps,
           limits.commit_down_vel_mps);
  nh.param("max_xy_vel_mps", limits.max_xy_vel_mps, limits.max_xy_vel_mps);
  nh.param("max_forward_vel_mps", limits.max_forward_vel_mps,
           limits.max_forward_vel_mps);
  nh.param("max_right_vel_mps", limits.max_right_vel_mps,
           limits.max_right_vel_mps);
  nh.param("max_down_vel_mps", limits.max_down_vel_mps, limits.max_down_vel_mps);
  nh.param("min_forward_vel_mps", limits.min_forward_vel_mps,
           limits.min_forward_vel_mps);
  nh.param("min_right_vel_mps", limits.min_right_vel_mps,
           limits.min_right_vel_mps);
  nh.param("min_vel_error_m", limits.min_vel_error_m, limits.min_vel_error_m);
  nh.param("xy_deadzone_m", limits.xy_deadzone_m, limits.xy_deadzone_m);
  nh.param("require_xy_align_before_land", limits.require_xy_align_before_land,
           limits.require_xy_align_before_land);
  nh.param("land_align_radius_m", limits.land_align_radius_m,
           limits.land_align_radius_m);
  nh.param("pad_commit_height_m", limits.pad_commit_height_m,
           limits.pad_commit_height_m);
  nh.param("floor_align_timeout_frames", limits.floor_align_timeout_frames,
           limits.floor_align_timeout_frames);
  nh.param("max_accel_mps2", limits.max_accel_mps2, limits.max_accel_mps2);
  nh.param("max_z_accel_mps2", limits.max_z_accel_mps2, limits.max_z_accel_mps2);
  nh.param("kp_xy", limits.kp_xy, limits.kp_xy);
  nh.param("kp_forward", limits.kp_forward, limits.kp_forward);
  nh.param("kp_right", limits.kp_right, limits.kp_right);
  nh.param("final_kp_xy", limits.final_kp_xy, limits.final_kp_xy);
  nh.param("final_kp_forward", limits.final_kp_forward, limits.final_kp_forward);
  nh.param("final_kp_right", limits.final_kp_right, limits.final_kp_right);
  nh.param("kd_forward", limits.kd_forward, limits.kd_forward);
  nh.param("kd_right", limits.kd_right, limits.kd_right);
  nh.param("final_stable_frames", limits.final_stable_frames,
           limits.final_stable_frames);
  ROS_INFO(
      "precision_landing limits: final_r=%.3f land_align_r=%.3f "
      "require_xy_align=%s stop_xy=%.3f final_h=%.3f "
      "commit_h=%.3f commit_vd=%.3f vf=%.3f vr=%.3f vd=%.3f "
      "min_vf=%.3f min_vr=%.3f kpf=%.3f kpr=%.3f fkpf=%.3f fkpr=%.3f",
      limits.final_radius_m, limits.land_align_radius_m,
      limits.require_xy_align_before_land ? "true" : "false",
      limits.descent_xy_stop_m, limits.final_height_m,
      limits.commit_height_m, limits.commit_down_vel_mps,
      limits.max_forward_vel_mps, limits.max_right_vel_mps,
      limits.max_down_vel_mps, limits.min_forward_vel_mps,
      limits.min_right_vel_mps, limits.kp_forward, limits.kp_right,
      limits.final_kp_forward, limits.final_kp_right);
  return limits;
}

precision_landing::AnchorConfig loadAnchorConfig(const ros::NodeHandle& nh) {
  precision_landing::AnchorConfig config;
  nh.param("anchor_process_std_xy_mps", config.process_std_xy_mps,
           config.process_std_xy_mps);
  nh.param("anchor_process_std_z_mps", config.process_std_z_mps,
           config.process_std_z_mps);
  nh.param("anchor_meas_std_xy_at_1m", config.meas_std_xy_at_1m,
           config.meas_std_xy_at_1m);
  nh.param("anchor_meas_std_xy_min_m", config.meas_std_xy_min_m,
           config.meas_std_xy_min_m);
  nh.param("anchor_innov_gate_sigma", config.innov_gate_sigma,
           config.innov_gate_sigma);
  nh.param("anchor_min_init_height_m", config.min_init_height_m,
           config.min_init_height_m);
  if (!nh.hasParam("anchor_min_init_height_m")) {
    nh.param("anchor_freeze_height_m", config.min_init_height_m,
             config.min_init_height_m);
  }
  nh.param("anchor_relocate_xy_m", config.relocate_xy_m, config.relocate_xy_m);
  nh.param("camera_offset_forward_m", config.camera_offset_body_flu.x(),
           config.camera_offset_body_flu.x());
  nh.param("camera_offset_left_m", config.camera_offset_body_flu.y(),
           config.camera_offset_body_flu.y());
  nh.param("camera_offset_up_m", config.camera_offset_body_flu.z(),
           config.camera_offset_body_flu.z());
  ROS_INFO(
      "precision_landing anchor ekf: q_xy=%.3f r_xy1m=%.3f gate=%.1f "
      "init_h=%.2f relocate=%.2f",
      config.process_std_xy_mps, config.meas_std_xy_at_1m,
      config.innov_gate_sigma, config.min_init_height_m, config.relocate_xy_m);
  return config;
}

bool isAutoLand(const std::string& mode) { return mode == "AUTO.LAND"; }

// 速度先从机体 FRD 转到 ENU，再用 FRAME_LOCAL_NED 发出去。
// 本机 MAVROS 的 LOCAL_NED 会把 ENU yaw 转成 PX4 NED（π/2 - enu），所以这里发 ENU。
// 不要用 FRAME_BODY_NED 发航向：那条转换会偏约 90°。
double yawEnuFromQuat(const Eigen::Quaterniond& q) {
  const Eigen::Vector3d body_x = q * Eigen::Vector3d::UnitX();
  return std::atan2(body_x.y(), body_x.x());
}

bool ensureDirectory(const std::string& path) {
  if (path.empty()) return false;
  std::string current;
  for (const char character : path) {
    current.push_back(character);
    if (character != '/' || current.size() == 1U) continue;
    if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) return false;
  }
  return ::mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

std::string wallTimestamp() {
  const std::time_t now = std::time(nullptr);
  std::tm local_time;
  localtime_r(&now, &local_time);
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return stream.str();
}

bool fileIsEmpty(const std::string& path) {
  struct stat status;
  return ::stat(path.c_str(), &status) != 0 || status.st_size == 0;
}

}  // namespace

class PrecisionLandingNode {
 public:
  PrecisionLandingNode()
      : private_nh_("~"),
        limits_(loadLimits(private_nh_)),
        controller_(limits_),
        anchor_(loadAnchorConfig(private_nh_)) {
    private_nh_.param("enable_flight", enable_flight_, false);
    private_nh_.param("auto_land", auto_land_, true);
    private_nh_.param("use_world_anchor", use_world_anchor_, true);
    private_nh_.param("hold_heading", hold_heading_, true);
    private_nh_.param("use_px4_auto_land", use_px4_auto_land_, true);
    private_nh_.param("land_yaw_rate_max_dps", land_yaw_rate_max_dps_, 1.0);
    private_nh_.param("landed_disarm_height_m", landed_disarm_height_m_, 0.105);
    private_nh_.param("landed_disarm_xy_m", landed_disarm_xy_m_, 0.12);
    private_nh_.param("landed_disarm_frames", landed_disarm_frames_, 3);
    private_nh_.param("align_heading_to_board", align_heading_to_board_, true);
    double board_yaw_offset_deg = 0.0;
    private_nh_.param("board_yaw_offset_deg", board_yaw_offset_deg, 0.0);
    board_yaw_offset_rad_ = board_yaw_offset_deg * M_PI / 180.0;
    private_nh_.param("align_heading_min_height_m", align_heading_min_height_m_,
                      0.40);
    private_nh_.param("align_heading_samples", align_heading_samples_, 5);
    private_nh_.param<std::string>(
        "log_directory", log_directory_,
        "/home/wxh/nb_code/autofly_ws/log/companion");
    arming_client_ =
        nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    command_client_ =
        nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    center_sub_ = nh_.subscribe("/precision_landing/board_center_camera", 5,
                                &PrecisionLandingNode::centerCallback, this);
    state_sub_ = nh_.subscribe("/mavros/state", 5,
                               &PrecisionLandingNode::stateCallback, this);
    odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 5,
                              &PrecisionLandingNode::odomCallback, this);
    setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 10);
    status_pub_ = nh_.advertise<std_msgs::String>("/precision_landing/status", 5);
    ready_pub_ = nh_.advertise<std_msgs::Bool>(
        "/precision_landing/ready_to_land", 1);
    target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/precision_landing/target_local_enu", 5);
    last_tick_ = ros::Time::now();
    ROS_INFO(
        "precision_landing auto_land=%s px4_auto_land=%s land_yaw_rate=%.1f "
        "world_anchor=%s hold_heading=%s align_board=%s yaw_off=%.0fdeg "
        "log_dir=%s",
        auto_land_ ? "true" : "false", use_px4_auto_land_ ? "true" : "false",
        land_yaw_rate_max_dps_, use_world_anchor_ ? "true" : "false",
        hold_heading_ ? "true" : "false",
        align_heading_to_board_ ? "true" : "false",
        board_yaw_offset_rad_ * 180.0 / M_PI, log_directory_.c_str());
  }

  ~PrecisionLandingNode() {
    if (session_active_) finishFlightSession("node_shutdown");
  }

  void tick() {
    const ros::Time now = ros::Time::now();
    const bool armed = fcu_state_.armed;
    const std::string& mode = fcu_state_.mode;

    if (land_requested_ && !isAutoLand(mode) && mode != "OFFBOARD") {
      ROS_WARN("auto-land latch cleared: mode is %s, not fighting POSITION",
               mode.c_str());
      resetControlState();
    }
    if (was_armed_ && !armed) {
      if (session_active_) finishFlightSession("disarmed");
      resetControlState();
      ROS_INFO("disarmed: landing latch reset, pad target kept (%d samples)",
               anchor_.samples());
    }
    was_armed_ = armed;

    if (enable_flight_ && armed && mode == "OFFBOARD" && !session_active_) {
      startFlightSession();
    }
    if (session_active_ && hold_heading_) updateHoldYaw();

    const bool tag_fresh =
        !center_stamp_.isZero() &&
        (now - center_stamp_).toSec() <= limits_.tag_timeout_s;
    const bool odom_fresh =
        !odom_stamp_.isZero() &&
        (now - odom_stamp_).toSec() <= limits_.vehicle_timeout_s;
    const double dt = std::max(0.001, (now - last_tick_).toSec());
    Eigen::Vector3d control_frd = center_frd_;
    bool world_target_valid = false;
    last_world_target_locked_ = false;
    if (use_world_anchor_ && odom_fresh) {
      if (tag_fresh) {
        anchor_.observe(center_frd_, vehicle_enu_, local_from_body_, dt);
      }
      Eigen::Vector3d fused_frd = Eigen::Vector3d::Zero();
      if (anchor_.errorBodyFrd(vehicle_enu_, local_from_body_, &fused_frd)) {
        control_frd = fused_frd;
        world_target_valid = true;
        last_world_target_locked_ = anchor_.locked();
        geometry_msgs::PoseStamped target;
        target.header.stamp = now;
        target.header.frame_id = "map";
        target.pose.position.x = anchor_.targetEnu().x();
        target.pose.position.y = anchor_.targetEnu().y();
        target.pose.position.z = anchor_.targetEnu().z();
        target.pose.orientation.w = 1.0;
        target_pub_.publish(target);
      }
    }

    precision_landing::Input input;
    input.enable_flight = enable_flight_;
    input.fcu_connected = fcu_state_.connected;
    input.fcu_offboard = mode == "OFFBOARD";
    input.tag_valid = !center_stamp_.isZero();
    input.world_target_valid = world_target_valid;
    input.world_target_locked = last_world_target_locked_;
    input.transform_valid = true;
    input.tag_age_s = (now - center_stamp_).toSec();
    input.vehicle_age_s = (now - odom_stamp_).toSec();
    input.dt_s = dt;
    input.tag_position_body_frd = control_frd;
    last_tick_ = now;
    last_control_frd_ = control_frd;
    last_tag_fresh_ = tag_fresh;
    last_world_target_valid_ = world_target_valid;

    const precision_landing::Output output = controller_.step(input);
    updateControlSample();
    const bool on_pad_now = gearOnGround();
    if (mode == "OFFBOARD" && auto_land_ &&
        (output.ready_to_land || on_pad_now)) {
      if (!land_requested_) {
        hold_yaw_enu_ = last_vehicle_yaw_enu_;
        have_hold_yaw_cmd_ = true;
        have_hold_yaw_ = true;
        ROS_INFO("%s, freeze yaw_enu=%.1f deg down=%.3f xy=%.3f",
                 on_pad_now && !output.ready_to_land ? "gear on pad"
                                                    : "ready to land",
                 hold_yaw_enu_ * 180.0 / M_PI, contactDown(), contactXy());
      }
      land_requested_ = true;
    }
    maybeFinishLanding(now);

    mavros_msgs::PositionTarget setpoint;
    setpoint.header.stamp = now;
    setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                         mavros_msgs::PositionTarget::IGNORE_PY |
                         mavros_msgs::PositionTarget::IGNORE_PZ |
                         mavros_msgs::PositionTarget::IGNORE_AFX |
                         mavros_msgs::PositionTarget::IGNORE_AFY |
                         mavros_msgs::PositionTarget::IGNORE_AFZ |
                         mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    if (hold_heading_ && have_hold_yaw_cmd_) {
      setpoint.yaw = static_cast<float>(hold_yaw_enu_);
      setpoint.yaw_rate = 0.0f;
      setpoint.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    } else {
      setpoint.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW;
      setpoint.yaw = std::numeric_limits<float>::quiet_NaN();
      setpoint.yaw_rate = std::numeric_limits<float>::quiet_NaN();
    }
    // 模式真正变成 AUTO.LAND 之前继续发 ENU yaw，空 SetMode 会把航向打到北。
    if (!isAutoLand(mode)) {
      Eigen::Vector3d velocity_body = output.velocity_body_frd;
      if (land_requested_ && !use_px4_auto_land_) {
        velocity_body.z() =
            std::max(velocity_body.z(), limits_.commit_down_vel_mps);
      }
      if (land_requested_ && use_px4_auto_land_) {
        if (gearOnGround()) {
          velocity_body.setZero();
        } else if (contactXy() > limits_.land_align_radius_m) {
          velocity_body.z() = 0.0;
        } else {
          velocity_body.z() = std::min(std::max(velocity_body.z(), 0.06), 0.12);
        }
      }
      const Eigen::Vector3d velocity_enu =
          precision_landing::bodyFrdVelocityToEnu(velocity_body,
                                                  local_from_body_);
      setpoint.velocity.x = velocity_enu.x();
      setpoint.velocity.y = velocity_enu.y();
      setpoint.velocity.z = velocity_enu.z();
      setpoint_pub_.publish(setpoint);
    }

    std_msgs::String status;
    if (isAutoLand(mode) && armed) {
      status.data = "AUTO.LAND active";
      if (!saw_auto_land_) {
        const double delta_deg =
            precision_landing::wrapPi(last_vehicle_yaw_enu_ - hold_yaw_enu_) *
            180.0 / M_PI;
        ROS_INFO(
            "AUTO.LAND active veh_enu=%.1f hold_enu=%.1f delta=%.1f deg "
            "(jump ~90 toward north means NAV_LAND dropped yaw)",
            last_vehicle_yaw_enu_ * 180.0 / M_PI,
            hold_yaw_enu_ * 180.0 / M_PI, delta_deg);
      }
      saw_auto_land_ = true;
    } else if (land_requested_ && mode == "OFFBOARD") {
      status.data = (use_px4_auto_land_ && !gearOnGround())
                        ? "AUTO.LAND requested"
                        : "pad_disarm";
    } else if (world_target_valid && !tag_fresh) {
      status.data = output.reason + std::string(" (hold_target)");
    } else {
      status.data = output.reason;
    }
    status_pub_.publish(status);
    std_msgs::Bool ready;
    ready.data = output.ready_to_land || land_requested_;
    ready_pub_.publish(ready);

    logTick(now, output, setpoint, status.data);
  }

 private:
  double contactDown() const {
    const double raw = center_frd_.z();
    if (last_tag_fresh_ && raw > -0.06 && raw < 0.25) return raw;
    return last_control_frd_.z();
  }

  double contactXy() const { return last_control_frd_.head<2>().norm(); }

  bool gearOnGround() const {
    const double down = contactDown();
    return down > -0.06 && down <= landed_disarm_height_m_;
  }

  bool contactOnPad() const {
    return gearOnGround() && contactXy() <= landed_disarm_xy_m_;
  }

  void updateControlSample() {
    if (!last_world_target_valid_ && center_stamp_.isZero()) return;
    last_forward_ = last_control_frd_.x();
    last_right_ = last_control_frd_.y();
    last_down_ = last_control_frd_.z();
    last_xy_ = last_control_frd_.head<2>().norm();
    have_sample_ = true;
    min_xy_ = std::min(min_xy_, last_xy_);
    max_xy_ = std::max(max_xy_, last_xy_);
  }

  void maybeFinishLanding(const ros::Time& now) {
    if (!auto_land_ || !land_requested_ || !enable_flight_) return;
    if (!fcu_state_.armed) return;
    if (fcu_state_.mode != "OFFBOARD") return;
    if (!have_land_request_pose_ && have_sample_) {
      xy_at_land_request_ = last_xy_;
      down_at_land_request_ = last_down_;
      have_land_request_pose_ = true;
    }
    if (use_px4_auto_land_) {
      if (gearOnGround()) {
        requestPadDisarm();
        return;
      }
      if (contactXy() > limits_.land_align_radius_m + 0.005) return;
      requestPx4AutoLand(now);
      return;
    }
    landed_ok_frames_ = contactOnPad() ? landed_ok_frames_ + 1 : 0;
    if (landed_ok_frames_ < landed_disarm_frames_) return;
    if (land_call_running_.load()) return;
    if (!last_land_request_.isZero() &&
        (now - last_land_request_).toSec() < 0.80) {
      return;
    }
    last_land_request_ = now;
    requestPadDisarm();
  }

  void requestPadDisarm() {
    if (land_call_running_.exchange(true)) return;
    saw_land_request_ = true;
    const double down = contactDown();
    const double xy = contactXy();
    ROS_INFO("pad disarm async down=%.3f xy=%.3f (force, skip AUTO.LAND)", down,
             xy);
    std::thread([this, down, xy]() {
      ros::NodeHandle nh;
      auto command =
          nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
      mavros_msgs::CommandLong force;
      force.request.command = mavros_msgs::CommandCode::COMPONENT_ARM_DISARM;
      force.request.confirmation = 1;
      force.request.param1 = 0.0;
      force.request.param2 = 21196.0;
      bool ok = command.call(force) && force.response.success;
      if (!ok) {
        auto arming =
            nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        mavros_msgs::CommandBool arm;
        arm.request.value = false;
        ok = arming.call(arm) && arm.response.success;
      }
      if (ok) {
        saw_auto_land_ = true;
        ROS_INFO("disarm on pad ok down=%.3f xy=%.3f", down, xy);
      } else {
        ROS_WARN("disarm on pad failed, will retry");
      }
      land_call_running_.store(false);
    }).detach();
  }

  void restorePx4YawLimits() {
    const float yawrate = saved_mc_yawrate_max_.load();
    const float yawrauto = saved_mpc_yawrauto_max_.load();
    std::thread([yawrate, yawrauto]() {
      setPx4Param("MC_YAWRATE_MAX", yawrate);
      setPx4Param("MPC_YAWRAUTO_MAX", yawrauto);
    }).detach();
  }

  void requestPx4AutoLand(const ros::Time& now) {
    if (land_call_running_.load()) return;
    if (!last_land_request_.isZero() && (now - last_land_request_).toSec() < 1.0) {
      return;
    }
    last_land_request_ = now;
    saw_land_request_ = true;
    const double yaw_enu =
        have_hold_yaw_cmd_ ? hold_yaw_enu_ : last_vehicle_yaw_enu_;
    const float yaw_ned_deg = yawNedDeg360(yaw_enu);
    const float yaw_rate_cap = static_cast<float>(land_yaw_rate_max_dps_);
    land_call_running_.store(true);
    ROS_INFO(
        "AUTO.LAND async hold_enu=%.1f; freeze MC_YAWRATE_MAX=%.1f then switch "
        "(commander drops NAV_LAND yaw, rate cap blocks the 90 deg snap)",
        yaw_enu * 180.0 / M_PI, yaw_rate_cap);
    std::thread([this, yaw_ned_deg, yaw_rate_cap]() {
      if (!have_saved_yaw_params_.exchange(true)) {
        saved_mc_yawrate_max_.store(getPx4Param("MC_YAWRATE_MAX", 200.0f));
        saved_mpc_yawrauto_max_.store(getPx4Param("MPC_YAWRAUTO_MAX", 45.0f));
      }
      const bool froze_rate = setPx4Param("MC_YAWRATE_MAX", yaw_rate_cap);
      setPx4Param("MPC_YAWRAUTO_MAX", 0.0f);
      if (froze_rate) {
        ros::Duration(0.20).sleep();
      } else {
        ROS_WARN("MC_YAWRATE_MAX not applied, AUTO.LAND may still yaw north");
      }
      ros::NodeHandle nh;
      auto mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
      mavros_msgs::SetMode mode;
      mode.request.custom_mode = "AUTO.LAND";
      bool sent = mode_client.call(mode) && mode.response.mode_sent;
      if (!sent) {
        auto land_client =
            nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
        mavros_msgs::CommandTOL land;
        land.request.min_pitch = 0.0;
        land.request.yaw = yaw_ned_deg;
        land.request.latitude = std::numeric_limits<float>::quiet_NaN();
        land.request.longitude = std::numeric_limits<float>::quiet_NaN();
        land.request.altitude = std::numeric_limits<float>::quiet_NaN();
        sent = land_client.call(land) && land.response.success;
      }
      if (!sent) {
        ROS_WARN("AUTO.LAND request failed, keep OFFBOARD hold yaw");
      }
      land_call_running_.store(false);
    }).detach();
  }

  void updateHoldYaw() {
    if (have_hold_yaw_ || odom_stamp_.isZero()) return;
    const double vehicle_yaw = last_vehicle_yaw_enu_;
    if (!have_hold_yaw_cmd_) {
      hold_yaw_enu_ = vehicle_yaw;
      have_hold_yaw_cmd_ = true;
    }
    const double height =
        last_control_frd_.z() > 1e-4 ? last_control_frd_.z() : center_frd_.z();
    Eigen::Vector3d board_forward_frd = Eigen::Vector3d::Zero();
    const bool tag_fresh =
        !center_stamp_.isZero() &&
        (ros::Time::now() - center_stamp_).toSec() <= limits_.tag_timeout_s;
    const bool have_board_yaw =
        align_heading_to_board_ && tag_fresh &&
        precision_landing::boardForwardBodyFrd(center_q_cam_,
                                              &board_forward_frd);

    if (have_board_yaw) {
      const double desired = precision_landing::yawEnuToFaceBoard(
          vehicle_yaw, board_forward_frd, board_yaw_offset_rad_);
      yaw_align_samples_.push_back(desired);
      if (static_cast<int>(yaw_align_samples_.size()) > align_heading_samples_) {
        yaw_align_samples_.erase(yaw_align_samples_.begin());
      }
      const bool enough =
          static_cast<int>(yaw_align_samples_.size()) >= align_heading_samples_;
      const bool too_low =
          height > 1e-4 && height < align_heading_min_height_m_;
      if (enough || too_low) {
        hold_yaw_enu_ = precision_landing::circularMeanYaw(yaw_align_samples_);
        have_hold_yaw_ = true;
        ROS_INFO("lock board heading yaw_enu=%.1f deg (pad front, %zu samples)",
                 hold_yaw_enu_ * 180.0 / M_PI, yaw_align_samples_.size());
      }
      return;
    }

    if (height > 1e-4 && height < align_heading_min_height_m_) {
      have_hold_yaw_ = true;
      ROS_INFO("hold heading yaw_enu=%.1f deg (no board yaw at %.2f m)",
               hold_yaw_enu_ * 180.0 / M_PI, height);
    }
  }

  void startFlightSession() {
    session_active_ = true;
    have_hold_yaw_ = false;
    have_hold_yaw_cmd_ = false;
    yaw_align_samples_.clear();
    saw_land_request_ = false;
    saw_auto_land_ = false;
    land_call_running_.store(false);
    have_sample_ = false;
    have_land_request_pose_ = false;
    landed_ok_frames_ = 0;
    min_xy_ = 1e9;
    max_xy_ = 0.0;
    last_forward_ = last_right_ = last_down_ = last_xy_ = 0.0;
    xy_at_land_request_ = down_at_land_request_ = 0.0;
    session_start_ = ros::Time::now();
    if (use_px4_auto_land_ && !have_saved_yaw_params_.load()) {
      std::thread([this]() {
        if (!have_saved_yaw_params_.exchange(true)) {
          saved_mc_yawrate_max_.store(getPx4Param("MC_YAWRATE_MAX", 200.0f));
          saved_mpc_yawrauto_max_.store(getPx4Param("MPC_YAWRAUTO_MAX", 45.0f));
        }
      }).detach();
    }
    ensureDirectory(log_directory_);
    detail_path_ = log_directory_ + "/flight_" + wallTimestamp() + ".csv";
    detail_stream_.open(detail_path_, std::ios::out | std::ios::trunc);
    if (!detail_stream_) {
      ROS_ERROR_STREAM("Cannot open companion landing log: " << detail_path_);
      return;
    }
    detail_stream_ << std::fixed;
    writeParamHeader(detail_stream_);
    detail_stream_
        << "timestamp,state,mode,armed,forward_m,right_m,down_m,error_xy_m,"
           "raw_forward_m,raw_right_m,raw_down_m,raw_xy_m,"
           "cmd_vx,cmd_vy,cmd_vz,cmd_yaw,veh_yaw_enu,hold_yaw_enu,"
           "land_requested,anchor_valid,tag_fresh,anchor_samples\n";
    ROS_INFO_STREAM("companion flight log: " << detail_path_);
  }

  void writeParamHeader(std::ostream& stream) {
    stream << "# companion_precision_landing params\n";
    stream << "# kp_forward=" << limits_.kp_forward
           << " kp_right=" << limits_.kp_right
           << " final_kp_forward=" << limits_.final_kp_forward
           << " final_kp_right=" << limits_.final_kp_right << '\n';
    stream << "# kd_forward=" << limits_.kd_forward
           << " kd_right=" << limits_.kd_right
           << " max_forward_vel=" << limits_.max_forward_vel_mps
           << " max_right_vel=" << limits_.max_right_vel_mps << '\n';
    stream << "# max_down_vel=" << limits_.max_down_vel_mps
           << " commit_down_vel=" << limits_.commit_down_vel_mps
           << " commit_height=" << limits_.commit_height_m
           << " min_forward_vel=" << limits_.min_forward_vel_mps
           << " min_right_vel=" << limits_.min_right_vel_mps << '\n';
    stream << "# final_radius=" << limits_.final_radius_m
           << " land_align_radius=" << limits_.land_align_radius_m
           << " require_xy_align=" << (limits_.require_xy_align_before_land ? 1 : 0)
           << " final_height=" << limits_.final_height_m
           << " descent_xy_stop=" << limits_.descent_xy_stop_m
           << " align_radius=" << limits_.align_radius_m << '\n';
    stream << "# descend_start=" << limits_.descend_start_height_m
           << " max_accel=" << limits_.max_accel_mps2
           << " max_z_accel=" << limits_.max_z_accel_mps2
           << " final_stable_frames=" << limits_.final_stable_frames
           << " auto_land=" << (auto_land_ ? 1 : 0)
           << " px4_auto_land=" << (use_px4_auto_land_ ? 1 : 0)
           << " use_world_anchor=" << (use_world_anchor_ ? 1 : 0)
           << " hold_heading=" << (hold_heading_ ? 1 : 0)
           << " align_board=" << (align_heading_to_board_ ? 1 : 0)
           << " yaw_off_deg=" << (board_yaw_offset_rad_ * 180.0 / M_PI)
           << " anchor_q_xy=" << anchor_.config().process_std_xy_mps
           << " anchor_r_xy1m=" << anchor_.config().meas_std_xy_at_1m << '\n';
  }

  void logTick(const ros::Time& now, const precision_landing::Output& output,
               const mavros_msgs::PositionTarget& setpoint,
               const std::string& status) {
    if (!session_active_ || !detail_stream_) return;
    const double xy = last_control_frd_.head<2>().norm();
    const double raw_xy = center_frd_.head<2>().norm();
    if (last_world_target_valid_ || !center_stamp_.isZero()) {
      last_forward_ = last_control_frd_.x();
      last_right_ = last_control_frd_.y();
      last_down_ = last_control_frd_.z();
      last_xy_ = xy;
      have_sample_ = true;
      min_xy_ = std::min(min_xy_, xy);
      max_xy_ = std::max(max_xy_, xy);
    }
    detail_stream_ << std::setprecision(6) << now.toSec() << ',' << status << ','
                   << fcu_state_.mode << ',' << (fcu_state_.armed ? 1 : 0) << ','
                   << last_forward_ << ',' << last_right_ << ',' << last_down_
                   << ',' << last_xy_ << ',' << center_frd_.x() << ','
                   << center_frd_.y() << ',' << center_frd_.z() << ',' << raw_xy
                   << ',' << setpoint.velocity.x << ',' << setpoint.velocity.y
                   << ',' << setpoint.velocity.z << ',' << setpoint.yaw << ','
                   << last_vehicle_yaw_enu_ << ',' << hold_yaw_enu_ << ','
                   << (land_requested_ ? 1 : 0) << ','
                   << (last_world_target_valid_ ? 1 : 0) << ','
                   << (last_tag_fresh_ ? 1 : 0) << ',' << anchor_.samples()
                   << '\n';
    (void)output;
  }

  void finishFlightSession(const std::string& reason) {
    if (have_saved_yaw_params_.load()) restorePx4YawLimits();
    const double duration =
        session_start_.isZero() ? 0.0
                                : (ros::Time::now() - session_start_).toSec();
    if (detail_stream_) detail_stream_.close();
    const std::string summary_path = log_directory_ + "/summary.csv";
    const bool write_header = fileIsEmpty(summary_path);
    std::ofstream summary(summary_path, std::ios::out | std::ios::app);
    if (summary) {
      if (write_header) {
        summary << "wall_time,end_reason,duration_s,disarmed,land_requested,"
                   "auto_land_active,final_forward_m,final_right_m,final_down_m,"
                   "final_xy_m,min_xy_m,max_xy_m,xy_at_land_request_m,"
                   "down_at_land_request_m,kp_forward,kp_right,final_kp_forward,"
                   "final_kp_right,kd_forward,kd_right,max_down_vel_mps,"
                   "final_radius_m,final_height_m,descent_xy_stop_m,"
                   "align_radius_m,max_forward_vel_mps,max_right_vel_mps,"
                   "detail_file\n";
      }
      summary << std::fixed << std::setprecision(6) << wallTimestamp() << ','
              << reason << ',' << duration << ','
              << (fcu_state_.armed ? 0 : 1) << ','
              << (saw_land_request_ ? 1 : 0) << ','
              << (saw_auto_land_ ? 1 : 0) << ',';
      if (have_sample_) {
        summary << last_forward_ << ',' << last_right_ << ',' << last_down_
                << ',' << last_xy_ << ','
                << (min_xy_ < 1e8 ? min_xy_ : last_xy_) << ',' << max_xy_ << ',';
      } else {
        summary << "nan,nan,nan,nan,nan,nan,";
      }
      if (have_land_request_pose_) {
        summary << xy_at_land_request_ << ',' << down_at_land_request_ << ',';
      } else {
        summary << "nan,nan,";
      }
      summary << limits_.kp_forward << ',' << limits_.kp_right << ','
              << limits_.final_kp_forward << ',' << limits_.final_kp_right << ','
              << limits_.kd_forward << ',' << limits_.kd_right << ','
              << limits_.max_down_vel_mps << ',' << limits_.final_radius_m << ','
              << limits_.final_height_m << ',' << limits_.descent_xy_stop_m << ','
              << limits_.align_radius_m << ',' << limits_.max_forward_vel_mps
              << ',' << limits_.max_right_vel_mps << ',' << detail_path_
              << '\n';
    } else {
      ROS_ERROR_STREAM("Cannot append companion landing summary: "
                       << summary_path);
    }
    ROS_INFO(
        "flight summary %s: final_xy=%.3f m down=%.3f m land=%d file=%s",
        reason.c_str(), have_sample_ ? last_xy_ : -1.0,
        have_sample_ ? last_down_ : -1.0, saw_auto_land_ ? 1 : 0,
        detail_path_.c_str());
    session_active_ = false;
  }

  void centerCallback(const geometry_msgs::PoseStampedConstPtr& message) {
    const auto& p = message->pose.position;
    center_frd_ = Eigen::Vector3d(-p.y, p.x, p.z);
    const auto& orientation = message->pose.orientation;
    center_q_cam_ = Eigen::Quaterniond(orientation.w, orientation.x,
                                       orientation.y, orientation.z);
    center_stamp_ = message->header.stamp;
  }
  void stateCallback(const mavros_msgs::StateConstPtr& message) {
    fcu_state_ = *message;
  }
  void odomCallback(const nav_msgs::OdometryConstPtr& message) {
    odom_stamp_ = ros::Time::now();
    const auto& p = message->pose.pose.position;
    const auto& q = message->pose.pose.orientation;
    vehicle_enu_ = Eigen::Vector3d(p.x, p.y, p.z);
    local_from_body_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
    last_vehicle_yaw_enu_ = yawEnuFromQuat(local_from_body_);
  }

  void resetControlState() {
    land_requested_ = false;
    last_land_request_ = ros::Time();
    landed_ok_frames_ = 0;
    have_hold_yaw_ = false;
    have_hold_yaw_cmd_ = false;
    yaw_align_samples_.clear();
    controller_.reset();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  precision_landing::Limits limits_;
  precision_landing::ControllerCore controller_;
  precision_landing::LandingTargetAnchor anchor_;
  ros::Subscriber center_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher setpoint_pub_;
  ros::Publisher status_pub_;
  ros::Publisher ready_pub_;
  ros::Publisher target_pub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient command_client_;
  mavros_msgs::State fcu_state_;
  ros::Time center_stamp_;
  ros::Time odom_stamp_;
  ros::Time last_tick_;
  ros::Time last_land_request_;
  ros::Time session_start_;
  Eigen::Vector3d center_frd_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_control_frd_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d vehicle_enu_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond local_from_body_ = Eigen::Quaterniond::Identity();
  bool enable_flight_ = false;
  bool auto_land_ = true;
  bool use_px4_auto_land_ = true;
  double land_yaw_rate_max_dps_ = 1.0;
  std::atomic<bool> land_call_running_{false};
  std::atomic<bool> have_saved_yaw_params_{false};
  std::atomic<float> saved_mc_yawrate_max_{200.0f};
  std::atomic<float> saved_mpc_yawrauto_max_{45.0f};
  bool use_world_anchor_ = true;
  bool hold_heading_ = true;
  bool align_heading_to_board_ = true;
  bool have_hold_yaw_ = false;
  bool have_hold_yaw_cmd_ = false;
  int align_heading_samples_ = 5;
  double hold_yaw_enu_ = 0.0;
  double last_vehicle_yaw_enu_ = 0.0;
  double landed_disarm_height_m_ = 0.105;
  double landed_disarm_xy_m_ = 0.08;
  int landed_disarm_frames_ = 3;
  int landed_ok_frames_ = 0;
  double board_yaw_offset_rad_ = 0.0;
  double align_heading_min_height_m_ = 0.40;
  Eigen::Quaterniond center_q_cam_ = Eigen::Quaterniond::Identity();
  std::vector<double> yaw_align_samples_;
  bool last_tag_fresh_ = false;
  bool last_world_target_valid_ = false;
  bool last_world_target_locked_ = false;
  bool land_requested_ = false;
  bool was_armed_ = false;
  bool session_active_ = false;
  bool saw_land_request_ = false;
  bool saw_auto_land_ = false;
  bool have_sample_ = false;
  bool have_land_request_pose_ = false;
  double min_xy_ = 1e9;
  double max_xy_ = 0.0;
  double last_forward_ = 0.0;
  double last_right_ = 0.0;
  double last_down_ = 0.0;
  double last_xy_ = 0.0;
  double xy_at_land_request_ = 0.0;
  double down_at_land_request_ = 0.0;
  std::string log_directory_;
  std::string detail_path_;
  std::ofstream detail_stream_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "precision_landing");
  PrecisionLandingNode node;
  ros::Rate rate(20);
  while (ros::ok()) {
    ros::spinOnce();
    node.tick();
    rate.sleep();
  }
}
