#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <limits>

#include "precision_landing/px4_landing_target_core.hpp"

namespace precision_landing {
namespace {

NativeTargetInput nominalInput() {
  NativeTargetInput input;
  input.now_s = 10.0;
  input.vision_stamp_s = 10.0;
  input.local_pose_age_s = 0.01;
  input.vision_valid = true;
  input.fcu_connected = true;
  input.enable_target_publish = true;
  input.mavros_local_ned = true;
  input.controller_conflict = false;
  input.camera_center = Eigen::Vector3d(0.20, 0.10, 1.00);
  input.vehicle_position_enu = Eigen::Vector3d(2.0, 3.0, 4.0);
  input.local_from_body = Eigen::Quaterniond::Identity();
  return input;
}

void expectVectorNear(const Eigen::Vector3d& actual,
                      const Eigen::Vector3d& expected,
                      double tolerance = 1e-9) {
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

TEST(Px4LandingTargetCoreTest, ConvertsCameraOpticalToBodyFluAndLocalEnu) {
  Px4LandingTargetCore core(0.10, 0.20, 0.20, Eigen::Vector3d::Zero());

  const NativeTargetOutput output = core.step(nominalInput());

  expectVectorNear(output.body_frd, Eigen::Vector3d(-0.10, 0.20, 1.00));
  expectVectorNear(output.body_flu, Eigen::Vector3d(-0.10, -0.20, -1.00));
  expectVectorNear(output.target_enu, Eigen::Vector3d(1.90, 2.80, 3.00));
  EXPECT_EQ(output.state, NativeTargetState::TRACKING);
  EXPECT_TRUE(output.target_valid);
  EXPECT_TRUE(output.should_publish);
}

TEST(Px4LandingTargetCoreTest, RotatesBodyVectorIntoLocalFrame) {
  Px4LandingTargetCore core(0.10, 0.20, 0.20, Eigen::Vector3d(0.05, -0.02, 0.01));
  NativeTargetInput input = nominalInput();
  input.local_from_body =
      Eigen::AngleAxisd(0.47, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-0.21, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.13, Eigen::Vector3d::UnitX());
  const Eigen::Vector3d expected_body_flu(-0.10, -0.20, -1.00);
  const Eigen::Vector3d expected_target =
      input.vehicle_position_enu +
      input.local_from_body.normalized() *
          (expected_body_flu + Eigen::Vector3d(0.05, -0.02, 0.01));

  const NativeTargetOutput output = core.step(input);

  expectVectorNear(output.target_enu, expected_target);
}

TEST(Px4LandingTargetCoreTest, HoldsOneAbsoluteTargetThenStopsAfterLossTimeout) {
  Px4LandingTargetCore core(0.10, 0.20, 0.20, Eigen::Vector3d::Zero());
  NativeTargetInput input = nominalInput();
  const NativeTargetOutput tracking = core.step(input);

  input.now_s = 10.15;
  input.vision_valid = false;
  input.vehicle_position_enu = Eigen::Vector3d(9.0, 8.0, 7.0);
  const NativeTargetOutput held = core.step(input);
  EXPECT_EQ(held.state, NativeTargetState::SHORT_HOLD);
  EXPECT_TRUE(held.should_publish);
  expectVectorNear(held.target_enu, tracking.target_enu);

  input.now_s = 10.201;
  const NativeTargetOutput lost = core.step(input);
  EXPECT_EQ(lost.state, NativeTargetState::LOST);
  EXPECT_FALSE(lost.should_publish);
  EXPECT_FALSE(lost.target_valid);
}

TEST(Px4LandingTargetCoreTest, IgnoresRepeatedVisionStampWhenHoldingTarget) {
  Px4LandingTargetCore core(0.10, 0.20, 0.20, Eigen::Vector3d::Zero());
  NativeTargetInput input = nominalInput();
  const NativeTargetOutput first = core.step(input);

  input.now_s = 10.05;
  input.camera_center = Eigen::Vector3d(-5.0, -4.0, 2.0);
  input.vehicle_position_enu = Eigen::Vector3d(20.0, 30.0, 40.0);
  const NativeTargetOutput repeated = core.step(input);

  expectVectorNear(repeated.target_enu, first.target_enu);
}

TEST(Px4LandingTargetCoreTest, BlocksPublicationForEverySafetyGate) {
  using Mutator = void (*)(NativeTargetInput*);
  const Mutator gates[] = {
      [](NativeTargetInput* input) { input->enable_target_publish = false; },
      [](NativeTargetInput* input) { input->fcu_connected = false; },
      [](NativeTargetInput* input) {
        input->local_pose_age_s = std::numeric_limits<double>::infinity();
      },
      [](NativeTargetInput* input) { input->mavros_local_ned = false; },
      [](NativeTargetInput* input) { input->controller_conflict = true; },
  };

  for (const Mutator gate : gates) {
    Px4LandingTargetCore core(0.10, 0.20, 0.20, Eigen::Vector3d::Zero());
    NativeTargetInput input = nominalInput();
    gate(&input);
    const NativeTargetOutput output = core.step(input);
    EXPECT_EQ(output.state, NativeTargetState::BLOCKED);
    EXPECT_TRUE(output.observation_fresh);
    EXPECT_FALSE(output.should_publish);
  }
}

TEST(Px4LandingTargetCoreTest,
     KeepsFreshCameraErrorAvailableWhenLocalPoseIsMissing) {
  Px4LandingTargetCore core(0.10, 0.20, 0.20, Eigen::Vector3d::Zero());
  NativeTargetInput input = nominalInput();
  input.local_pose_age_s = std::numeric_limits<double>::infinity();

  const NativeTargetOutput output = core.step(input);

  EXPECT_TRUE(output.observation_fresh);
  EXPECT_FALSE(output.target_valid);
  expectVectorNear(output.body_frd, Eigen::Vector3d(-0.10, 0.20, 1.00));
}

TEST(Px4LandingTargetCoreTest, RejectsInvalidVisionWithoutCreatingTarget) {
  Px4LandingTargetCore core(0.10, 0.20, 0.20, Eigen::Vector3d::Zero());
  NativeTargetInput input = nominalInput();
  input.camera_center.z() = 0.0;

  const NativeTargetOutput output = core.step(input);

  EXPECT_EQ(output.state, NativeTargetState::WAITING);
  EXPECT_FALSE(output.target_valid);
  EXPECT_FALSE(output.should_publish);
}

}  // namespace
}  // namespace precision_landing
