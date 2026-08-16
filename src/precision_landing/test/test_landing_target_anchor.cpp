#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "precision_landing/landing_target_anchor.hpp"

namespace {

void expectNear(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected,
                double tolerance = 1e-9) {
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

}  // namespace

TEST(LandingTargetAnchor, FirstObservationBecomesWorldTarget) {
  precision_landing::LandingTargetAnchor anchor;
  const Eigen::Vector3d tag_frd(0.10, -0.20, 1.00);
  const Eigen::Vector3d vehicle(2.0, 3.0, 4.0);
  ASSERT_TRUE(anchor.observe(tag_frd, vehicle, Eigen::Quaterniond::Identity()));
  expectNear(anchor.targetEnu(), Eigen::Vector3d(2.10, 3.20, 3.00));
  Eigen::Vector3d error = Eigen::Vector3d::Zero();
  ASSERT_TRUE(anchor.errorBodyFrd(vehicle, Eigen::Quaterniond::Identity(), &error));
  expectNear(error, tag_frd);
}

TEST(LandingTargetAnchor, VehicleMotionKeepsFixedTargetError) {
  precision_landing::LandingTargetAnchor anchor;
  const Eigen::Vector3d vehicle(1.0, 2.0, 3.0);
  ASSERT_TRUE(anchor.observe(Eigen::Vector3d(0.0, 0.0, 1.0), vehicle,
                             Eigen::Quaterniond::Identity()));
  const Eigen::Vector3d moved = vehicle + Eigen::Vector3d(0.12, -0.05, 0.0);
  Eigen::Vector3d error = Eigen::Vector3d::Zero();
  ASSERT_TRUE(anchor.errorBodyFrd(moved, Eigen::Quaterniond::Identity(), &error));
  EXPECT_NEAR(error.x(), -0.12, 1e-9);
  EXPECT_NEAR(error.y(), -0.05, 1e-9);
}

TEST(LandingTargetAnchor, OutlierDoesNotMoveSettledTarget) {
  precision_landing::AnchorConfig config;
  config.min_init_height_m = 0.10;
  config.innov_gate_sigma = 3.0;
  precision_landing::LandingTargetAnchor anchor(config);
  const Eigen::Vector3d vehicle = Eigen::Vector3d::Zero();
  const Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  for (int n = 0; n < 8; ++n) {
    ASSERT_TRUE(anchor.observe(Eigen::Vector3d(0.02, 0.0, 1.0), vehicle, q, 0.05));
  }
  const Eigen::Vector3d before = anchor.targetEnu();
  EXPECT_FALSE(anchor.observe(Eigen::Vector3d(0.40, 0.0, 1.0), vehicle, q, 0.05));
  expectNear(anchor.targetEnu(), before, 1e-9);
}

TEST(LandingTargetAnchor, LowHeightTracksStableMeasurement) {
  precision_landing::AnchorConfig config;
  config.min_init_height_m = 0.40;
  precision_landing::LandingTargetAnchor anchor(config);
  const Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  ASSERT_TRUE(anchor.observe(Eigen::Vector3d(0.0, 0.0, 1.00),
                             Eigen::Vector3d::Zero(), q, 0.05));
  int accepted = 0;
  for (int n = 0; n < 20; ++n) {
    if (anchor.observe(Eigen::Vector3d(0.05, 0.0, 0.20),
                       Eigen::Vector3d::Zero(), q, 0.05)) {
      ++accepted;
    }
  }
  EXPECT_GE(accepted, 8);
  EXPECT_NEAR(anchor.targetEnu().x(), 0.05, 0.015);
  EXPECT_NEAR(anchor.targetEnu().y(), 0.0, 1e-6);
}

TEST(LandingTargetAnchor, HoldsLastTargetWhenDetectionsStop) {
  precision_landing::LandingTargetAnchor anchor;
  const Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  ASSERT_TRUE(anchor.observe(Eigen::Vector3d(0.05, 0.0, 1.0),
                             Eigen::Vector3d::Zero(), q));
  const Eigen::Vector3d held = anchor.targetEnu();
  Eigen::Vector3d error = Eigen::Vector3d::Zero();
  ASSERT_TRUE(
      anchor.errorBodyFrd(Eigen::Vector3d(0.20, 0.0, 0.0), q, &error));
  expectNear(held, Eigen::Vector3d(0.05, 0.0, -1.0));
  EXPECT_NEAR(error.x(), -0.15, 1e-9);
}

TEST(LandingTargetAnchor, RejectsLowFirstObservation) {
  precision_landing::LandingTargetAnchor anchor;
  EXPECT_FALSE(anchor.observe(Eigen::Vector3d(0.20, 0.0, 0.24),
                              Eigen::Vector3d::Zero(),
                              Eigen::Quaterniond::Identity()));
  EXPECT_FALSE(anchor.valid());
}

TEST(LandingTargetAnchor, RelocatesWhenHighObservationJumpsFar) {
  precision_landing::LandingTargetAnchor anchor;
  const Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  ASSERT_TRUE(anchor.observe(Eigen::Vector3d(0.0, 0.0, 1.0),
                             Eigen::Vector3d::Zero(), q));
  ASSERT_TRUE(anchor.observe(Eigen::Vector3d(0.80, 0.0, 1.0),
                             Eigen::Vector3d::Zero(), q));
  Eigen::Vector3d error = Eigen::Vector3d::Zero();
  ASSERT_TRUE(anchor.errorBodyFrd(Eigen::Vector3d::Zero(), q, &error));
  EXPECT_NEAR(error.x(), 0.80, 1e-6);
}
