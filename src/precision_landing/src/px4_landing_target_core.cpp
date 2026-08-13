#include "precision_landing/px4_landing_target_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace precision_landing {
namespace {

bool finiteVector(const Eigen::Vector3d& value) {
  return value.array().isFinite().all();
}

bool finiteQuaternion(const Eigen::Quaterniond& value) {
  return std::isfinite(value.w()) && std::isfinite(value.x()) &&
         std::isfinite(value.y()) && std::isfinite(value.z());
}

}  // namespace

Px4LandingTargetCore::Px4LandingTargetCore(
    double fresh_s, double short_hold_s, double local_pose_timeout_s,
    Eigen::Vector3d camera_offset_body_flu)
    : fresh_s_(fresh_s),
      short_hold_s_(short_hold_s),
      local_pose_timeout_s_(local_pose_timeout_s),
      camera_offset_body_flu_(std::move(camera_offset_body_flu)) {
  if (!std::isfinite(fresh_s_) || !std::isfinite(short_hold_s_) ||
      !std::isfinite(local_pose_timeout_s_) || fresh_s_ < 0.0 ||
      short_hold_s_ < fresh_s_ || local_pose_timeout_s_ <= 0.0 ||
      !finiteVector(camera_offset_body_flu_)) {
    throw std::invalid_argument("invalid PX4 landing-target core configuration");
  }
}

NativeTargetOutput Px4LandingTargetCore::step(const NativeTargetInput& input) {
  NativeTargetOutput output;

  const bool vision_time_finite =
      std::isfinite(input.now_s) && std::isfinite(input.vision_stamp_s);
  const bool local_pose_fresh =
      std::isfinite(input.local_pose_age_s) &&
      input.local_pose_age_s >= 0.0 &&
      input.local_pose_age_s <= local_pose_timeout_s_ &&
      finiteVector(input.vehicle_position_enu);
  const double quaternion_norm = input.local_from_body.norm();
  const bool pose_orientation_valid =
      finiteQuaternion(input.local_from_body) &&
      std::isfinite(quaternion_norm) && quaternion_norm > 1e-9;

  const bool sample_is_new = input.vision_stamp_s > last_vision_stamp_s_;
  const bool sample_time_valid = vision_time_finite &&
                                 input.vision_stamp_s >= 0.0 &&
                                 input.vision_stamp_s <= input.now_s;
  const bool sample_geometry_valid =
      input.vision_valid && finiteVector(input.camera_center) &&
      input.camera_center.z() > 0.0;
  const double observation_age_s = input.now_s - input.vision_stamp_s;
  output.observation_fresh =
      sample_time_valid && sample_geometry_valid &&
      std::isfinite(observation_age_s) && observation_age_s <= fresh_s_;
  if (output.observation_fresh) {
    output.body_frd = Eigen::Vector3d(-input.camera_center.y(),
                                      input.camera_center.x(),
                                      input.camera_center.z());
    output.body_flu = Eigen::Vector3d(output.body_frd.x(),
                                      -output.body_frd.y(),
                                      -output.body_frd.z());
  }

  if (sample_is_new && sample_time_valid && sample_geometry_valid &&
      local_pose_fresh && pose_orientation_valid) {
    cached_body_frd_ = output.body_frd;
    cached_body_flu_ = output.body_flu;
    cached_target_enu_ =
        input.vehicle_position_enu + input.local_from_body.normalized() *
                                         (cached_body_flu_ +
                                          camera_offset_body_flu_);
    last_vision_stamp_s_ = input.vision_stamp_s;
    have_target_ = true;
  }

  if (have_target_) {
    output.target_age_s = input.now_s - last_vision_stamp_s_;
    output.target_enu = cached_target_enu_;
    if (!output.observation_fresh) {
      output.body_flu = cached_body_flu_;
      output.body_frd = cached_body_frd_;
    }
    if (std::isfinite(output.target_age_s) && output.target_age_s >= 0.0 &&
        output.target_age_s <= fresh_s_) {
      output.state = NativeTargetState::TRACKING;
      output.target_valid = true;
      output.reason = "tracking";
    } else if (std::isfinite(output.target_age_s) &&
               output.target_age_s > fresh_s_ &&
               output.target_age_s <= short_hold_s_) {
      output.state = NativeTargetState::SHORT_HOLD;
      output.target_valid = true;
      output.reason = "short_hold";
    } else {
      output.state = NativeTargetState::LOST;
      output.reason = "target_lost";
    }
  }

  std::string gate_reason;
  if (!input.enable_target_publish) {
    gate_reason = "publishing_disabled";
  } else if (!input.fcu_connected) {
    gate_reason = "fcu_disconnected";
  } else if (!input.mavros_local_ned) {
    gate_reason = "mavros_frame_not_local_ned";
  } else if (input.controller_conflict) {
    gate_reason = "companion_controller_conflict";
  } else if (!local_pose_fresh || !pose_orientation_valid) {
    gate_reason = "local_pose_invalid_or_stale";
  }

  if (!gate_reason.empty()) {
    output.state = NativeTargetState::BLOCKED;
    output.reason = gate_reason;
    output.should_publish = false;
  } else {
    output.should_publish = output.target_valid;
  }
  return output;
}

const char* nativeTargetStateName(NativeTargetState state) {
  switch (state) {
    case NativeTargetState::WAITING:
      return "WAITING";
    case NativeTargetState::TRACKING:
      return "TRACKING";
    case NativeTargetState::SHORT_HOLD:
      return "SHORT_HOLD";
    case NativeTargetState::LOST:
      return "LOST";
    case NativeTargetState::BLOCKED:
      return "BLOCKED";
  }
  return "UNKNOWN";
}

}  // namespace precision_landing
