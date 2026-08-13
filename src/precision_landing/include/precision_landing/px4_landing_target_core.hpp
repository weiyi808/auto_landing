#pragma once

#include <Eigen/Geometry>

#include <limits>
#include <string>

namespace precision_landing {

enum class NativeTargetState { WAITING, TRACKING, SHORT_HOLD, LOST, BLOCKED };

struct NativeTargetInput {
  double now_s = 0.0;
  double vision_stamp_s = 0.0;
  double local_pose_age_s = std::numeric_limits<double>::infinity();
  bool vision_valid = false;
  bool fcu_connected = false;
  bool enable_target_publish = false;
  bool mavros_local_ned = false;
  bool controller_conflict = false;
  Eigen::Vector3d camera_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d vehicle_position_enu = Eigen::Vector3d::Zero();
  Eigen::Quaterniond local_from_body = Eigen::Quaterniond::Identity();
};

struct NativeTargetOutput {
  NativeTargetState state = NativeTargetState::WAITING;
  bool observation_fresh = false;
  bool target_valid = false;
  bool should_publish = false;
  Eigen::Vector3d target_enu = Eigen::Vector3d::Zero();
  Eigen::Vector3d body_flu = Eigen::Vector3d::Zero();
  Eigen::Vector3d body_frd = Eigen::Vector3d::Zero();
  double target_age_s = std::numeric_limits<double>::infinity();
  std::string reason = "waiting_for_target";
};

class Px4LandingTargetCore {
 public:
  Px4LandingTargetCore(double fresh_s, double short_hold_s,
                       double local_pose_timeout_s,
                       Eigen::Vector3d camera_offset_body_flu);

  NativeTargetOutput step(const NativeTargetInput& input);

 private:
  double fresh_s_;
  double short_hold_s_;
  double local_pose_timeout_s_;
  Eigen::Vector3d camera_offset_body_flu_;
  bool have_target_ = false;
  double last_vision_stamp_s_ = -std::numeric_limits<double>::infinity();
  Eigen::Vector3d cached_target_enu_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d cached_body_flu_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d cached_body_frd_ = Eigen::Vector3d::Zero();
};

const char* nativeTargetStateName(NativeTargetState state);

}  // namespace precision_landing
