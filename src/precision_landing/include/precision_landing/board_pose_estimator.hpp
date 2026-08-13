#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace precision_landing {

struct BoardPoseEstimate {
  bool valid = false;
  cv::Vec3d rvec{0.0, 0.0, 0.0};
  cv::Vec3d tvec{0.0, 0.0, 0.0};
  double reprojection_rms_px = 0.0;
  int tag_count = 0;
  std::string reason = "not estimated";
};

BoardPoseEstimate estimateBoardPose(
    const std::vector<int>& ids,
    const std::vector<std::vector<cv::Point2f>>& image_corners,
    const cv::Mat& camera_matrix,
    const cv::Mat& distortion,
    double board_scale,
    double max_reprojection_rms_px);

}  // namespace precision_landing
