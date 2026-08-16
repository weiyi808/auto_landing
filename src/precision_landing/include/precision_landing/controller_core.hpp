#pragma once

#include <Eigen/Core>
#include <string>

namespace precision_landing {

enum class State {
  WAIT_FOR_DATA,
  SEARCH_HOLD,
  ALIGN,
  DESCEND,
  READY_TO_LAND,
  STALE_HOLD,
  ABORT_HOLD
};

struct Limits {
  double tag_timeout_s = 0.20;
  double vehicle_timeout_s = 0.20;
  double align_radius_m = 0.12;
  double final_radius_m = 0.035;
  double final_height_m = 0.15;
  double descend_start_height_m = 0.80;
  double descent_xy_stop_m = 0.08;
  double commit_height_m = 0.30;
  double commit_down_vel_mps = 0.18;
  double max_xy_vel_mps = 0.45;
  double max_forward_vel_mps = 0.18;
  double max_right_vel_mps = 0.25;
  double max_down_vel_mps = 0.22;
  double min_forward_vel_mps = 0.08;
  double min_right_vel_mps = 0.10;
  double min_vel_error_m = 0.04;
  double xy_deadzone_m = 0.012;
  bool require_xy_align_before_land = false;
  double land_align_radius_m = 0.020;
  double pad_commit_height_m = 0.105;
  int floor_align_timeout_frames = 24;
  double max_accel_mps2 = 0.30;
  double max_z_accel_mps2 = 0.30;
  double kp_xy = 0.8;
  double kp_forward = 0.50;
  double kp_right = 0.70;
  double final_kp_xy = 0.35;
  double final_kp_forward = 0.14;
  double final_kp_right = 0.20;
  double kd_forward = 0.22;
  double kd_right = 0.28;
  int final_stable_frames = 10;
};

struct Input {
  bool enable_flight = false;
  bool fcu_connected = false;
  bool fcu_offboard = false;
  bool tag_valid = false;
  bool world_target_valid = false;
  bool world_target_locked = false;
  bool transform_valid = false;
  double tag_age_s = 1e9;
  double vehicle_age_s = 1e9;
  double dt_s = 0.05;
  Eigen::Vector3d tag_position_body_frd = Eigen::Vector3d::Zero();
};

struct Output {
  State state = State::WAIT_FOR_DATA;
  Eigen::Vector3d velocity_body_frd = Eigen::Vector3d::Zero();
  bool ready_to_land = false;
  std::string reason;
};

class ControllerCore {
 public:
  explicit ControllerCore(Limits limits);
  Output step(const Input& input);
  void reset();

 private:
  Output hold(State state, const std::string& reason);
  double descentScale(double lateral_error) const;
  Limits limits_;
  State state_ = State::WAIT_FOR_DATA;
  Eigen::Vector3d last_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_error_ = Eigen::Vector3d::Zero();
  Eigen::Vector2d error_rate_ = Eigen::Vector2d::Zero();
  bool have_error_history_ = false;
  int stable_frames_ = 0;
  int floor_align_frames_ = 0;
  bool floor_align_timed_out_ = false;
};

const char* stateName(State state);

}  // namespace precision_landing
