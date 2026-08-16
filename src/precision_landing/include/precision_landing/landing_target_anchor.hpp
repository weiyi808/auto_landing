#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace precision_landing {

struct AnchorConfig {
  double process_std_xy_mps = 0.035;
  double process_std_z_mps = 0.040;
  double meas_std_xy_at_1m = 0.030;
  double meas_std_xy_min_m = 0.008;
  double meas_std_z_frac = 0.04;
  double meas_std_z_min_m = 0.010;
  double innov_gate_sigma = 3.0;
  double min_init_height_m = 0.40;
  double relocate_xy_m = 0.50;
  int lock_samples = 8;
  Eigen::Vector3d camera_offset_body_flu = Eigen::Vector3d::Zero();
};

class LandingTargetAnchor {
 public:
  explicit LandingTargetAnchor(AnchorConfig config = {});

  void reset();
  bool observe(const Eigen::Vector3d& tag_body_frd,
               const Eigen::Vector3d& vehicle_enu,
               const Eigen::Quaterniond& local_from_body, double dt_s = 0.05);
  bool errorBodyFrd(const Eigen::Vector3d& vehicle_enu,
                    const Eigen::Quaterniond& local_from_body,
                    Eigen::Vector3d* error_frd) const;

  bool valid() const { return valid_; }
  bool locked() const {
    return valid_ && seen_high_ && samples_ >= config_.lock_samples;
  }
  int samples() const { return samples_; }
  const AnchorConfig& config() const { return config_; }
  Eigen::Vector3d targetEnu() const { return target_enu_; }

  static Eigen::Vector3d frdToFlu(const Eigen::Vector3d& frd);
  static Eigen::Vector3d fluToFrd(const Eigen::Vector3d& flu);
  static Eigen::Vector3d observationEnu(
      const Eigen::Vector3d& tag_body_frd,
      const Eigen::Vector3d& vehicle_enu,
      const Eigen::Quaterniond& local_from_body,
      const Eigen::Vector3d& camera_offset_body_flu);

 private:
  Eigen::Matrix3d measurementCov(double height) const;
  void startFilter(const Eigen::Vector3d& observation, double height);
  bool updateFilter(const Eigen::Vector3d& observation, double height,
                    double dt_s);

  AnchorConfig config_;
  bool valid_ = false;
  bool seen_high_ = false;
  int samples_ = 0;
  Eigen::Vector3d target_enu_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d cov_ = Eigen::Matrix3d::Identity();
};

}  // namespace precision_landing
