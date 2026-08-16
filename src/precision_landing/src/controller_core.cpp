#include "precision_landing/controller_core.hpp"

#include <algorithm>
#include <cmath>

namespace precision_landing {
namespace {

double clamp(double value, double low, double high) {
  return std::min(high, std::max(low, value));
}

double clampAbs(double value, double limit) {
  return clamp(value, -limit, limit);
}

}  // namespace

ControllerCore::ControllerCore(Limits limits) : limits_(limits) {}

void ControllerCore::reset() {
  floor_align_frames_ = 0;
  floor_align_timed_out_ = false;
  hold(State::WAIT_FOR_DATA, "reset");
}

const char* stateName(State state) {
  switch (state) {
    case State::WAIT_FOR_DATA: return "WAIT_FOR_DATA";
    case State::SEARCH_HOLD: return "SEARCH_HOLD";
    case State::ALIGN: return "ALIGN";
    case State::DESCEND: return "DESCEND";
    case State::READY_TO_LAND: return "READY_TO_LAND";
    case State::STALE_HOLD: return "STALE_HOLD";
    default: return "ABORT_HOLD";
  }
}

Output ControllerCore::hold(State state, const std::string& reason) {
  state_ = state;
  stable_frames_ = 0;
  last_velocity_.setZero();
  have_error_history_ = false;
  error_rate_.setZero();
  return {state, Eigen::Vector3d::Zero(), false, reason};
}

double ControllerCore::descentScale(double lateral_error) const {
  const double full = std::max(1e-6, limits_.final_radius_m);
  const double stop = std::max(full + 1e-6, limits_.descent_xy_stop_m);
  if (lateral_error <= full) return 1.0;
  if (lateral_error >= stop) return 0.0;
  const double t = (stop - lateral_error) / (stop - full);
  return t * t;
}

Output ControllerCore::step(const Input& input) {
  if (!input.enable_flight) return hold(State::WAIT_FOR_DATA, "flight gate disabled");
  if (!input.fcu_connected) {
    return hold(State::WAIT_FOR_DATA, "FCU not connected or not OFFBOARD");
  }
  if (!input.fcu_offboard) {
    if (state_ == State::READY_TO_LAND) {
      return {state_, last_velocity_, true, "ready_to_land"};
    }
    return hold(State::WAIT_FOR_DATA, "FCU not connected or not OFFBOARD");
  }
  if (input.vehicle_age_s > limits_.vehicle_timeout_s) {
    return hold(State::STALE_HOLD, "stale vehicle state");
  }
  if (!input.tag_valid && !input.world_target_valid) {
    return hold(State::SEARCH_HOLD, "tag not detected");
  }
  if (!input.transform_valid) {
    return hold(State::ABORT_HOLD, "camera-to-body transform invalid");
  }
  if (!input.world_target_valid && input.tag_age_s > limits_.tag_timeout_s) {
    return hold(State::STALE_HOLD, "stale tag pose");
  }

  const double height = input.tag_position_body_frd.z();
  const double lateral_error = input.tag_position_body_frd.head<2>().norm();
  const bool already_ready = state_ == State::READY_TO_LAND;
  const bool contact_ok =
      already_ready && height > -0.08 && height <= 0.03 && lateral_error <= 0.25;
  if ((!contact_ok && height <= 0.03) || height > 5.0 || lateral_error > 2.0) {
    return hold(State::ABORT_HOLD, "relative pose outside safety envelope");
  }

  const double land_xy = limits_.require_xy_align_before_land
                             ? limits_.land_align_radius_m
                             : limits_.final_radius_m;
  bool ready = state_ == State::READY_TO_LAND;
  const bool final =
      lateral_error <= land_xy && height <= limits_.final_height_m;
  if (!ready) {
    if (final && ++stable_frames_ >= limits_.final_stable_frames) {
      ready = true;
    }
    if (!final) stable_frames_ = 0;
  }

  const double dt = std::max(0.001, input.dt_s);
  const Eigen::Vector3d error = input.tag_position_body_frd;
  if (have_error_history_) {
    const Eigen::Vector2d raw_rate =
        (error.head<2>() - last_error_.head<2>()) / dt;
    error_rate_ = 0.55 * error_rate_ + 0.45 * raw_rate;
  } else {
    error_rate_.setZero();
    have_error_history_ = true;
  }
  last_error_ = error;

  const bool low = height < 0.35;
  const double kp_f = low ? limits_.final_kp_forward : limits_.kp_forward;
  const double kp_r = low ? limits_.final_kp_right : limits_.kp_right;
  auto axisCommand = [&](double axis_error, double rate, double kp, double kd,
                         double vel_limit, double min_vel) {
    const double p = kp * axis_error;
    double command = p;
    if (axis_error * rate < 0.0) {
      command = p + kd * rate;
      if (command * axis_error < 0.0) command = 0.0;
    }
    if (std::abs(axis_error) <= limits_.xy_deadzone_m) {
      command = 0.0;
    } else if (std::abs(axis_error) >= limits_.min_vel_error_m &&
               std::abs(command) < min_vel) {
      command = std::copysign(min_vel, axis_error);
    }
    return clampAbs(command, vel_limit);
  };
  Eigen::Vector3d desired(
      axisCommand(error.x(), error_rate_.x(), kp_f, limits_.kd_forward,
                  limits_.max_forward_vel_mps, limits_.min_forward_vel_mps),
      axisCommand(error.y(), error_rate_.y(), kp_r, limits_.kd_right,
                  limits_.max_right_vel_mps, limits_.min_right_vel_mps),
      0.0);

  const bool want_floor_align =
      limits_.require_xy_align_before_land && !ready &&
      height <= limits_.final_height_m && lateral_error > land_xy;
  if (want_floor_align && !floor_align_timed_out_ &&
      limits_.floor_align_timeout_frames > 0) {
    ++floor_align_frames_;
    if (floor_align_frames_ >= limits_.floor_align_timeout_frames) {
      floor_align_timed_out_ = true;
    }
  }
  if (floor_align_timed_out_ && height <= limits_.final_height_m &&
      lateral_error <= land_xy) {
    ready = true;
  }
  const bool wait_xy_at_floor =
      want_floor_align && !floor_align_timed_out_ &&
      height > limits_.pad_commit_height_m;
  const bool committed = input.world_target_locked &&
                         height <= limits_.commit_height_m && !wait_xy_at_floor;
  double z_scale = descentScale(lateral_error);
  if (committed) z_scale = std::max(z_scale, 0.70);
  const bool high = height > limits_.descend_start_height_m;
  const bool hold_for_lock =
      !input.world_target_locked && height <= limits_.commit_height_m &&
      lateral_error > limits_.align_radius_m;
  state_ = ((high && lateral_error > limits_.align_radius_m) || hold_for_lock ||
            wait_xy_at_floor)
               ? State::ALIGN
               : State::DESCEND;
  if (height > 0.03 && !wait_xy_at_floor &&
      !((high && lateral_error > limits_.align_radius_m) || hold_for_lock)) {
    desired.z() = limits_.max_down_vel_mps * z_scale;
    if (committed) {
      desired.z() = std::max(desired.z(), limits_.commit_down_vel_mps);
    }
  }
  if (ready) {
    state_ = State::READY_TO_LAND;
    desired.z() = std::max(desired.z(), limits_.commit_down_vel_mps);
  }

  const double max_xy = limits_.max_accel_mps2 * dt;
  const double max_z = limits_.max_z_accel_mps2 * dt;
  Eigen::Vector2d delta_xy = desired.head<2>() - last_velocity_.head<2>();
  if (delta_xy.norm() > max_xy) delta_xy *= max_xy / delta_xy.norm();
  const double delta_z = clamp(desired.z() - last_velocity_.z(), -max_z, max_z);
  last_velocity_.x() += delta_xy.x();
  last_velocity_.y() += delta_xy.y();
  last_velocity_.z() += delta_z;
  return {state_, last_velocity_, ready,
          ready ? "ready_to_land" : stateName(state_)};
}

}  // namespace precision_landing
