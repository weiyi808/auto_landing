#include "precision_landing/landing_target_anchor.hpp"

#include <algorithm>
#include <cmath>

namespace precision_landing {
namespace {

bool finiteVector(const Eigen::Vector3d& value) {
  return value.array().isFinite().all();
}

bool finiteMatrix(const Eigen::Matrix3d& value) {
  return value.array().isFinite().all();
}

}  // namespace

LandingTargetAnchor::LandingTargetAnchor(AnchorConfig config)
    : config_(std::move(config)) {}

void LandingTargetAnchor::reset() {
  valid_ = false;
  seen_high_ = false;
  samples_ = 0;
  target_enu_.setZero();
  cov_ = Eigen::Matrix3d::Identity();
}

Eigen::Vector3d LandingTargetAnchor::frdToFlu(const Eigen::Vector3d& frd) {
  return Eigen::Vector3d(frd.x(), -frd.y(), -frd.z());
}

Eigen::Vector3d LandingTargetAnchor::fluToFrd(const Eigen::Vector3d& flu) {
  return Eigen::Vector3d(flu.x(), -flu.y(), -flu.z());
}

Eigen::Vector3d LandingTargetAnchor::observationEnu(
    const Eigen::Vector3d& tag_body_frd, const Eigen::Vector3d& vehicle_enu,
    const Eigen::Quaterniond& local_from_body,
    const Eigen::Vector3d& camera_offset_body_flu) {
  const Eigen::Vector3d body_flu =
      frdToFlu(tag_body_frd) + camera_offset_body_flu;
  return vehicle_enu + local_from_body.normalized() * body_flu;
}

Eigen::Matrix3d LandingTargetAnchor::measurementCov(double height) const {
  const double h = std::max(0.05, height);
  const double std_xy =
      std::max(config_.meas_std_xy_min_m, config_.meas_std_xy_at_1m * h);
  const double std_z =
      std::max(config_.meas_std_z_min_m, config_.meas_std_z_frac * h);
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  R(0, 0) = std_xy * std_xy;
  R(1, 1) = std_xy * std_xy;
  R(2, 2) = std_z * std_z;
  return R;
}

void LandingTargetAnchor::startFilter(const Eigen::Vector3d& observation,
                                      double height) {
  target_enu_ = observation;
  cov_ = measurementCov(height);
  valid_ = true;
  samples_ = 1;
}

bool LandingTargetAnchor::updateFilter(const Eigen::Vector3d& observation,
                                       double height, double dt_s) {
  const double dt = std::max(0.001, dt_s);
  const double q_xy = config_.process_std_xy_mps;
  const double q_z = config_.process_std_z_mps;
  Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
  Q(0, 0) = q_xy * q_xy * dt;
  Q(1, 1) = q_xy * q_xy * dt;
  Q(2, 2) = q_z * q_z * dt;
  cov_ += Q;

  const Eigen::Matrix3d R = measurementCov(height);
  const Eigen::Vector3d innov = observation - target_enu_;
  const Eigen::Matrix3d S = cov_ + R;
  const Eigen::Matrix2d S_xy = S.topLeftCorner<2, 2>();
  const double det = S_xy.determinant();
  if (!std::isfinite(det) || std::abs(det) < 1e-18) return false;
  const double mahal =
      innov.head<2>().transpose() * S_xy.inverse() * innov.head<2>();
  const double gate = config_.innov_gate_sigma * config_.innov_gate_sigma;
  if (!std::isfinite(mahal) || mahal > gate) return false;

  const Eigen::Matrix3d K = cov_ * S.ldlt().solve(Eigen::Matrix3d::Identity());
  target_enu_ += K * innov;
  cov_ = (Eigen::Matrix3d::Identity() - K) * cov_;
  cov_ = 0.5 * (cov_ + cov_.transpose());
  if (!finiteVector(target_enu_) || !finiteMatrix(cov_)) {
    startFilter(observation, height);
    return true;
  }
  ++samples_;
  return true;
}

bool LandingTargetAnchor::observe(const Eigen::Vector3d& tag_body_frd,
                                  const Eigen::Vector3d& vehicle_enu,
                                  const Eigen::Quaterniond& local_from_body,
                                  double dt_s) {
  if (!finiteVector(tag_body_frd) || !finiteVector(vehicle_enu) ||
      !finiteVector(config_.camera_offset_body_flu)) {
    return false;
  }
  const double qn = local_from_body.norm();
  if (!std::isfinite(qn) || qn < 1e-9) return false;
  const double height = tag_body_frd.z();
  const double lateral = tag_body_frd.head<2>().norm();
  if (height <= 0.03 || height > 5.0 || lateral > 2.0) return false;

  const Eigen::Vector3d observation = observationEnu(
      tag_body_frd, vehicle_enu, local_from_body,
      config_.camera_offset_body_flu);
  if (!finiteVector(observation)) return false;

  const bool high = height >= config_.min_init_height_m;
  if (!valid_) {
    if (!high) return false;
    startFilter(observation, height);
    seen_high_ = true;
    return true;
  }

  const double delta_xy = (observation - target_enu_).head<2>().norm();
  if (high && delta_xy > config_.relocate_xy_m) {
    startFilter(observation, height);
    seen_high_ = true;
    return true;
  }
  if (high) seen_high_ = true;
  return updateFilter(observation, height, dt_s);
}

bool LandingTargetAnchor::errorBodyFrd(
    const Eigen::Vector3d& vehicle_enu,
    const Eigen::Quaterniond& local_from_body,
    Eigen::Vector3d* error_frd) const {
  if (!valid_ || error_frd == nullptr) return false;
  if (!finiteVector(vehicle_enu) || !finiteVector(target_enu_)) return false;
  const double qn = local_from_body.norm();
  if (!std::isfinite(qn) || qn < 1e-9) return false;
  const Eigen::Vector3d error_flu =
      local_from_body.normalized().inverse() * (target_enu_ - vehicle_enu);
  *error_frd = fluToFrd(error_flu);
  return error_frd->array().isFinite().all();
}

}  // namespace precision_landing
