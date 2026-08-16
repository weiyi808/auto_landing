#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Geometry>

namespace precision_landing {

inline double wrapPi(double yaw) {
  while (yaw > M_PI) yaw -= 2.0 * M_PI;
  while (yaw < -M_PI) yaw += 2.0 * M_PI;
  return yaw;
}

inline Eigen::Vector3d cameraToFrd(const Eigen::Vector3d& camera) {
  return Eigen::Vector3d(-camera.y(), camera.x(), camera.z());
}

// Board frame: +X right on the page, +Y down. Page top / pad front is -Y.
inline Eigen::Vector3d boardForwardInBoard() {
  return Eigen::Vector3d(0.0, -1.0, 0.0);
}

inline bool boardForwardBodyFrd(const Eigen::Quaterniond& camera_from_board,
                                Eigen::Vector3d* forward_frd) {
  if (forward_frd == nullptr || camera_from_board.norm() < 0.5) return false;
  const Eigen::Vector3d camera =
      camera_from_board.normalized() * boardForwardInBoard();
  *forward_frd = cameraToFrd(camera);
  return std::hypot(forward_frd->x(), forward_frd->y()) > 0.20;
}

// Positive FRD yaw-error means the pad front is to the right of the nose.
// ENU yaw is CCW, so subtract that error to point the nose at the pad front.
inline double yawEnuToFaceBoard(double vehicle_yaw_enu,
                                const Eigen::Vector3d& board_forward_frd,
                                double yaw_offset_rad = 0.0) {
  const double yaw_error =
      std::atan2(board_forward_frd.y(), board_forward_frd.x());
  return wrapPi(vehicle_yaw_enu - yaw_error + yaw_offset_rad);
}

inline double circularMeanYaw(const std::vector<double>& yaws) {
  if (yaws.empty()) return 0.0;
  double sine = 0.0;
  double cosine = 0.0;
  for (const double yaw : yaws) {
    sine += std::sin(yaw);
    cosine += std::cos(yaw);
  }
  return std::atan2(sine, cosine);
}

// PX4 本体要 NED（0=北）。本机 MAVROS LOCAL_NED 已经做 ned = π/2 - enu，
// 节点发 setpoint 时仍用 ENU；只有直连 PX4 才调用这个函数。
inline double yawEnuToNed(double yaw_enu) {
  return wrapPi(M_PI / 2.0 - yaw_enu);
}

// FRD body velocity -> ENU world velocity. FRAME_LOCAL_NED 必须发 ENU 速度。
// BODY_NED 那条航向转换会偏约 90°。
inline Eigen::Vector3d bodyFrdVelocityToEnu(
    const Eigen::Vector3d& velocity_body_frd,
    const Eigen::Quaterniond& local_from_body) {
  const Eigen::Vector3d velocity_body_flu(velocity_body_frd.x(),
                                          -velocity_body_frd.y(),
                                          -velocity_body_frd.z());
  return local_from_body.normalized() * velocity_body_flu;
}

}  // namespace precision_landing
