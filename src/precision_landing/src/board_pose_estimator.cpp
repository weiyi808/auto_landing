#include "precision_landing/board_pose_estimator.hpp"

#include <cmath>
#include <limits>

#include <opencv2/calib3d.hpp>

#include "precision_landing/board_geometry.hpp"

namespace precision_landing {
namespace {

bool finiteVector(const cv::Vec3d& value) {
  return std::isfinite(value[0]) && std::isfinite(value[1]) &&
         std::isfinite(value[2]);
}

}  // namespace

BoardPoseEstimate estimateBoardPose(
    const std::vector<int>& ids,
    const std::vector<std::vector<cv::Point2f>>& image_corners,
    const cv::Mat& camera_matrix,
    const cv::Mat& distortion,
    double board_scale,
    double max_reprojection_rms_px) {
  BoardPoseEstimate result;
  if (ids.size() != image_corners.size() || camera_matrix.empty() ||
      board_scale <= 0.0 || max_reprojection_rms_px <= 0.0) {
    result.reason = "invalid estimator input";
    return result;
  }

  std::vector<cv::Point3d> object_points;
  std::vector<cv::Point2d> observed_points;
  for (std::size_t tag_index = 0; tag_index < ids.size(); ++tag_index) {
    BoardCorners board_corners;
    if (image_corners[tag_index].size() != 4 ||
        !boardObjectCorners(ids[tag_index], board_scale, &board_corners)) {
      continue;
    }
    for (std::size_t corner = 0; corner < 4; ++corner) {
      const BoardPoint& point = board_corners[corner];
      object_points.emplace_back(point.x, point.y, point.z);
      observed_points.emplace_back(image_corners[tag_index][corner]);
    }
    ++result.tag_count;
  }

  if (object_points.size() < 4) {
    result.reason = "insufficient board points";
    return result;
  }

  bool solved = false;
  try {
    solved = cv::solvePnP(object_points, observed_points, camera_matrix,
                          distortion, result.rvec, result.tvec, false,
                          cv::SOLVEPNP_ITERATIVE);
  } catch (const cv::Exception&) {
    result.reason = "solvePnP exception";
    return result;
  }
  if (!solved) {
    result.reason = "solvePnP failed";
    return result;
  }
  if (!finiteVector(result.rvec) || !finiteVector(result.tvec) ||
      result.tvec[2] <= 0.0) {
    result.reason = "invalid board pose";
    return result;
  }

  std::vector<cv::Point2d> projected_points;
  cv::projectPoints(object_points, result.rvec, result.tvec, camera_matrix,
                    distortion, projected_points);
  double squared_error_sum = 0.0;
  for (std::size_t index = 0; index < projected_points.size(); ++index) {
    const cv::Point2d error = projected_points[index] - observed_points[index];
    squared_error_sum += error.dot(error);
  }
  result.reprojection_rms_px = std::sqrt(
      squared_error_sum / static_cast<double>(projected_points.size()));
  if (!std::isfinite(result.reprojection_rms_px)) {
    result.reason = "invalid reprojection error";
    return result;
  }
  if (result.reprojection_rms_px > max_reprojection_rms_px) {
    result.reason = "reprojection error too high";
    return result;
  }

  result.valid = true;
  result.reason = "ok";
  return result;
}

}  // namespace precision_landing
