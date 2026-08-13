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
  double final_height_m = 0.18;
  double descend_start_height_m = 0.80;
  double max_xy_vel_mps = 0.25;
  double max_down_vel_mps = 0.12;
  double max_accel_mps2 = 0.40;
  double kp_xy = 0.8;
  double final_kp_xy = 0.45;
  int final_stable_frames = 20;
};

struct Input {
  bool enable_flight = false;
  bool fcu_connected = false;
  bool fcu_offboard = false;
  bool tag_valid = false;
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

 private:
  Output hold(State state, const std::string& reason);
  Limits limits_;
  State state_ = State::WAIT_FOR_DATA;
  Eigen::Vector3d last_velocity_ = Eigen::Vector3d::Zero();
  int stable_frames_ = 0;
};

const char* stateName(State state);

}  // namespace precision_landing
