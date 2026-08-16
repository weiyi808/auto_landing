#include <cmath>

#include <gtest/gtest.h>

#include "precision_landing/heading_from_board.hpp"

TEST(HeadingFromBoard, IdentityBoardFacesBodyForward) {
  Eigen::Vector3d forward_frd = Eigen::Vector3d::Zero();
  ASSERT_TRUE(precision_landing::boardForwardBodyFrd(
      Eigen::Quaterniond::Identity(), &forward_frd));
  EXPECT_NEAR(forward_frd.x(), 1.0, 1e-9);
  EXPECT_NEAR(forward_frd.y(), 0.0, 1e-9);
  EXPECT_NEAR(precision_landing::yawEnuToFaceBoard(0.3, forward_frd), 0.3, 1e-9);
}

TEST(HeadingFromBoard, BoardFrontToTheRightNeedsClockwiseEnuYaw) {
  const Eigen::AngleAxisd camera_from_board(0.5 * M_PI, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d forward_frd = Eigen::Vector3d::Zero();
  ASSERT_TRUE(precision_landing::boardForwardBodyFrd(
      Eigen::Quaterniond(camera_from_board), &forward_frd));
  EXPECT_NEAR(forward_frd.x(), 0.0, 1e-9);
  EXPECT_NEAR(forward_frd.y(), 1.0, 1e-9);
  EXPECT_NEAR(precision_landing::yawEnuToFaceBoard(0.0, forward_frd),
              -0.5 * M_PI, 1e-9);
}

TEST(HeadingFromBoard, OffsetRotatesLockedYaw) {
  Eigen::Vector3d forward_frd = Eigen::Vector3d::Zero();
  ASSERT_TRUE(precision_landing::boardForwardBodyFrd(
      Eigen::Quaterniond::Identity(), &forward_frd));
  EXPECT_NEAR(precision_landing::yawEnuToFaceBoard(0.0, forward_frd, M_PI),
              M_PI, 1e-9);
}

TEST(HeadingFromBoard, BodyFrdVelocityToEnuFacingEast) {
  const Eigen::Quaterniond facing_east = Eigen::Quaterniond::Identity();
  const Eigen::Vector3d enu = precision_landing::bodyFrdVelocityToEnu(
      Eigen::Vector3d(0.10, 0.0, 0.20), facing_east);
  EXPECT_NEAR(enu.x(), 0.10, 1e-9);
  EXPECT_NEAR(enu.y(), 0.0, 1e-9);
  EXPECT_NEAR(enu.z(), -0.20, 1e-9);
}

TEST(HeadingFromBoard, EnuEastIsNedNorth) {
  EXPECT_NEAR(precision_landing::yawEnuToNed(0.0), 0.5 * M_PI, 1e-9);
  EXPECT_NEAR(precision_landing::yawEnuToNed(0.5 * M_PI), 0.0, 1e-9);
  EXPECT_NEAR(std::abs(precision_landing::yawEnuToNed(-0.5 * M_PI)), M_PI,
              1e-9);
}

TEST(HeadingFromBoard, BodyFrdVelocityToEnuFacingNorth) {
  const Eigen::Quaterniond facing_north(
      Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d enu = precision_landing::bodyFrdVelocityToEnu(
      Eigen::Vector3d(0.10, 0.0, 0.0), facing_north);
  EXPECT_NEAR(enu.x(), 0.0, 1e-9);
  EXPECT_NEAR(enu.y(), 0.10, 1e-9);
  EXPECT_NEAR(enu.z(), 0.0, 1e-9);
}
