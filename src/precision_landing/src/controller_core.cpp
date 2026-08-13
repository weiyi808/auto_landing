#include "precision_landing/controller_core.hpp"

#include <algorithm>
#include <cmath>

namespace precision_landing {

ControllerCore::ControllerCore(Limits limits) : limits_(limits) {}

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
  return {state, Eigen::Vector3d::Zero(), false, reason};
}

Output ControllerCore::step(const Input& input) {
  if (!input.enable_flight) return hold(State::WAIT_FOR_DATA, "flight gate disabled");
  if (!input.fcu_connected || !input.fcu_offboard) {
    return hold(State::WAIT_FOR_DATA, "FCU not connected or not OFFBOARD");
  }
  if (input.vehicle_age_s > limits_.vehicle_timeout_s) {
    return hold(State::STALE_HOLD, "stale vehicle state");
  }
  if (!input.tag_valid) return hold(State::SEARCH_HOLD, "tag not detected");
  if (!input.transform_valid) {
    return hold(State::ABORT_HOLD, "camera-to-body transform invalid");
  }
  if (input.tag_age_s > limits_.tag_timeout_s) {
    return hold(State::STALE_HOLD, "stale tag pose");
  }

  const double height = input.tag_position_body_frd.z();
  const double lateral_error = input.tag_position_body_frd.head<2>().norm();
  if (height <= 0.03 || height > 5.0 || lateral_error > 2.0) {
    return hold(State::ABORT_HOLD, "relative pose outside safety envelope");
  }

  const bool final = lateral_error <= limits_.final_radius_m &&
                     height <= limits_.final_height_m;
  if (final && ++stable_frames_ >= limits_.final_stable_frames) {
    state_ = State::READY_TO_LAND;
    last_velocity_.setZero();
    return {state_, Eigen::Vector3d::Zero(), true, "pilot may now select LAND"};
  }
  if (!final) stable_frames_ = 0;

  const double gain = height < 0.35 ? limits_.final_kp_xy : limits_.kp_xy;
  Eigen::Vector3d desired(input.tag_position_body_frd.x() * gain,
                          input.tag_position_body_frd.y() * gain, 0.0);
  state_ = lateral_error > limits_.align_radius_m ? State::ALIGN : State::DESCEND;
  if (state_ == State::DESCEND && height > limits_.final_height_m) {
    desired.z() = limits_.max_down_vel_mps;
  }
  desired.head<2>() *= std::min(
      1.0, limits_.max_xy_vel_mps / std::max(1e-9, desired.head<2>().norm()));

  const double max_delta = limits_.max_accel_mps2 * std::max(0.001, input.dt_s);
  Eigen::Vector3d delta = desired - last_velocity_;
  if (delta.norm() > max_delta) delta *= max_delta / delta.norm();
  last_velocity_ += delta;
  return {state_, last_velocity_, false, stateName(state_)};
}

}  // namespace precision_landing
