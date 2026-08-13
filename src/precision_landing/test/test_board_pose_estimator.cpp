#include <array>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>

#include "precision_landing/board_geometry.hpp"
#include "precision_landing/board_pose_estimator.hpp"

namespace {

const cv::Mat kCamera = (cv::Mat_<double>(3, 3) <<
    385.8869, 0.0, 312.5947,
    0.0, 385.3804, 244.3042,
    0.0, 0.0, 1.0);
const cv::Mat kDistortion = cv::Mat::zeros(1, 5, CV_64F);
const cv::Vec3d kRvec(0.12, -0.08, 0.15);
const cv::Vec3d kTvec(0.03, -0.02, 0.80);

std::vector<cv::Point2f> projectTag(int id) {
  precision_landing::BoardCorners board_corners;
  EXPECT_TRUE(precision_landing::boardObjectCorners(id, 1.0, &board_corners));
  std::vector<cv::Point3d> object_points;
  for (const auto& point : board_corners) {
    object_points.emplace_back(point.x, point.y, point.z);
  }
  std::vector<cv::Point2d> projected;
  cv::projectPoints(object_points, kRvec, kTvec, kCamera, kDistortion, projected);
  std::vector<cv::Point2f> result;
  for (const auto& point : projected) result.emplace_back(point);
  return result;
}

void expectKnownPose(const precision_landing::BoardPoseEstimate& estimate,
                     int expected_tags) {
  ASSERT_TRUE(estimate.valid) << estimate.reason;
  EXPECT_EQ(estimate.tag_count, expected_tags);
  EXPECT_LT(cv::norm(estimate.tvec - kTvec), 1e-3);
  EXPECT_LT(estimate.reprojection_rms_px, 0.1);
}

}  // namespace

TEST(BoardPoseEstimator, OneOuterTagRecoversIdZeroOrigin) {
  const std::vector<int> ids{10};
  const std::vector<std::vector<cv::Point2f>> corners{projectTag(10)};
  expectKnownPose(precision_landing::estimateBoardPose(
                      ids, corners, kCamera, kDistortion, 1.0, 3.0),
                  1);
}

TEST(BoardPoseEstimator, MultipleTagsShareOneTiltedBoardPose) {
  const std::vector<int> ids{10, 21, 22, 13};
  std::vector<std::vector<cv::Point2f>> corners;
  for (const int id : ids) corners.push_back(projectTag(id));
  expectKnownPose(precision_landing::estimateBoardPose(
                      ids, corners, kCamera, kDistortion, 1.0, 3.0),
                  4);
}

TEST(BoardPoseEstimator, RejectsExcessiveReprojectionError) {
  const std::vector<int> ids{10, 21};
  std::vector<std::vector<cv::Point2f>> corners{projectTag(10), projectTag(21)};
  corners[1][0].x += 40.0f;
  const auto estimate = precision_landing::estimateBoardPose(
      ids, corners, kCamera, kDistortion, 1.0, 1.0);
  EXPECT_FALSE(estimate.valid);
  EXPECT_EQ(estimate.reason, "reprojection error too high");
}
