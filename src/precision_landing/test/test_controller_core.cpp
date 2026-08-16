#include <gtest/gtest.h>

#include "precision_landing/controller_core.hpp"

TEST(ControllerCore, DisabledGateAlwaysProducesZeroVelocity) {
  precision_landing::ControllerCore core(precision_landing::Limits{});
  precision_landing::Input input;
  input.enable_flight = false;
  EXPECT_DOUBLE_EQ(core.step(input).velocity_body_frd.norm(), 0.0);
}

static precision_landing::Input freshInput() {
  precision_landing::Input input;
  input.enable_flight = true;
  input.fcu_connected = true;
  input.fcu_offboard = true;
  input.tag_valid = true;
  input.transform_valid = true;
  input.tag_age_s = 0.01;
  input.vehicle_age_s = 0.01;
  input.tag_position_body_frd = Eigen::Vector3d(0.01, -0.01, 0.18);
  return input;
}

TEST(ControllerCore, StaleTagHolds) {
  auto input = freshInput();
  input.tag_age_s = 0.21;
  const auto output = precision_landing::ControllerCore({}).step(input);
  EXPECT_EQ(output.state, precision_landing::State::STALE_HOLD);
  EXPECT_TRUE(output.velocity_body_frd.isZero());
}

TEST(ControllerCore, StableLowTagBecomesReady) {
  precision_landing::ControllerCore core({});
  auto input = freshInput();
  input.tag_position_body_frd = Eigen::Vector3d(0.01, -0.01, 0.12);
  for (int n = 0; n < 20; ++n) core.step(input);
  EXPECT_EQ(core.step(input).state, precision_landing::State::READY_TO_LAND);
}

TEST(ControllerCore, HighAltitudeLateralErrorBlocksDescent) {
  auto input = freshInput();
  input.tag_position_body_frd = Eigen::Vector3d(0.25, 0.0, 1.2);
  const auto output = precision_landing::ControllerCore({}).step(input);
  EXPECT_EQ(output.state, precision_landing::State::ALIGN);
  EXPECT_DOUBLE_EQ(output.velocity_body_frd.z(), 0.0);
}

TEST(ControllerCore, DescentSpeedsUpAsLateralErrorShrinks) {
  precision_landing::Limits limits;
  limits.final_height_m = 0.12;
  limits.final_radius_m = 0.03;
  limits.descent_xy_stop_m = 0.10;
  limits.descend_start_height_m = 0.50;
  limits.commit_height_m = 0.10;
  limits.commit_down_vel_mps = 0.0;
  limits.max_down_vel_mps = 0.12;
  limits.max_z_accel_mps2 = 10.0;
  limits.max_accel_mps2 = 10.0;
  limits.kd_forward = 0.0;
  limits.kd_right = 0.0;
  limits.min_forward_vel_mps = 0.0;
  limits.min_right_vel_mps = 0.0;
  auto far = freshInput();
  far.tag_position_body_frd = Eigen::Vector3d(0.08, 0.0, 0.30);
  far.dt_s = 1.0;
  auto near = far;
  near.tag_position_body_frd.x() = 0.02;
  const double vz_far =
      precision_landing::ControllerCore(limits).step(far).velocity_body_frd.z();
  const double vz_near =
      precision_landing::ControllerCore(limits).step(near).velocity_body_frd.z();
  EXPECT_GT(vz_near, vz_far);
  EXPECT_GT(vz_near, 0.0);
}

TEST(ControllerCore, ClosingErrorAddsDampingNotOvershootBoost) {
  precision_landing::Limits limits;
  limits.kp_forward = 0.4;
  limits.final_kp_forward = 0.4;
  limits.kd_forward = 0.3;
  limits.kd_right = 0.0;
  limits.max_forward_vel_mps = 1.0;
  limits.max_accel_mps2 = 10.0;
  limits.max_z_accel_mps2 = 10.0;
  limits.max_down_vel_mps = 0.0;
  limits.min_forward_vel_mps = 0.0;
  limits.min_right_vel_mps = 0.0;
  precision_landing::ControllerCore core(limits);
  auto input = freshInput();
  input.dt_s = 0.05;
  input.tag_position_body_frd = Eigen::Vector3d(0.20, 0.0, 1.0);
  core.step(input);
  input.tag_position_body_frd.x() = 0.10;
  const auto closing = core.step(input);
  EXPECT_GE(closing.velocity_body_frd.x(), 0.0);
  EXPECT_LT(closing.velocity_body_frd.x(), 0.4 * 0.10 + 1e-9);
}

TEST(ControllerCore, ReadyLatchesThroughLateralOvershoot) {
  precision_landing::Limits limits;
  limits.final_height_m = 0.20;
  limits.final_radius_m = 0.035;
  limits.final_stable_frames = 10;
  precision_landing::ControllerCore core(limits);
  auto input = freshInput();
  input.tag_position_body_frd = Eigen::Vector3d(0.01, -0.01, 0.12);
  for (int n = 0; n < 12; ++n) core.step(input);
  EXPECT_EQ(core.step(input).state, precision_landing::State::READY_TO_LAND);
  input.tag_position_body_frd.x() = 0.20;
  const auto latched = core.step(input);
  EXPECT_EQ(latched.state, precision_landing::State::READY_TO_LAND);
  EXPECT_TRUE(latched.ready_to_land);
  EXPECT_GT(latched.velocity_body_frd.x(), 0.0);
  core.reset();
  input.tag_position_body_frd.x() = 0.25;
  input.tag_position_body_frd.z() = 1.2;
  EXPECT_NE(core.step(input).state, precision_landing::State::READY_TO_LAND);
}

TEST(ControllerCore, StaleTagContinuesWithWorldTarget) {
  auto input = freshInput();
  input.tag_age_s = 0.80;
  input.world_target_valid = true;
  input.tag_position_body_frd = Eigen::Vector3d(0.04, 0.0, 0.30);
  input.dt_s = 1.0;
  const auto output = precision_landing::ControllerCore({}).step(input);
  EXPECT_NE(output.state, precision_landing::State::STALE_HOLD);
  EXPECT_GT(output.velocity_body_frd.x(), 0.0);
}

TEST(ControllerCore, CommitHeightKeepsDescendingDespiteLateralError) {
  precision_landing::Limits limits;
  limits.commit_height_m = 0.30;
  limits.commit_down_vel_mps = 0.18;
  limits.max_down_vel_mps = 0.22;
  limits.descent_xy_stop_m = 0.10;
  limits.descend_start_height_m = 0.50;
  limits.final_height_m = 0.13;
  limits.max_z_accel_mps2 = 10.0;
  limits.max_accel_mps2 = 10.0;
  auto input = freshInput();
  input.tag_position_body_frd = Eigen::Vector3d(0.12, 0.0, 0.22);
  input.world_target_locked = true;
  input.dt_s = 1.0;
  const auto output = precision_landing::ControllerCore(limits).step(input);
  EXPECT_EQ(output.state, precision_landing::State::DESCEND);
  EXPECT_GE(output.velocity_body_frd.z(), 0.18);
}

TEST(ControllerCore, BelowFinalHeightStillDescendsUntilReady) {
  precision_landing::Limits limits;
  limits.final_height_m = 0.13;
  limits.final_radius_m = 0.03;
  limits.final_stable_frames = 20;
  limits.commit_height_m = 0.30;
  limits.commit_down_vel_mps = 0.18;
  limits.max_z_accel_mps2 = 10.0;
  auto input = freshInput();
  input.tag_position_body_frd = Eigen::Vector3d(0.08, 0.0, 0.11);
  input.world_target_locked = true;
  input.dt_s = 1.0;
  const auto output = precision_landing::ControllerCore(limits).step(input);
  EXPECT_GT(output.velocity_body_frd.z(), 0.0);
}

TEST(ControllerCore, LargeErrorGetsMinimumHorizontalCommand) {
  precision_landing::Limits limits;
  limits.final_kp_forward = 0.12;
  limits.kp_forward = 0.12;
  limits.min_forward_vel_mps = 0.08;
  limits.xy_deadzone_m = 0.012;
  limits.max_forward_vel_mps = 0.20;
  limits.max_accel_mps2 = 10.0;
  limits.max_down_vel_mps = 0.0;
  limits.commit_down_vel_mps = 0.0;
  limits.kd_forward = 0.0;
  auto input = freshInput();
  input.tag_position_body_frd = Eigen::Vector3d(0.10, 0.0, 0.40);
  input.dt_s = 1.0;
  const auto output = precision_landing::ControllerCore(limits).step(input);
  EXPECT_NEAR(output.velocity_body_frd.x(), 0.08, 1e-9);
}

TEST(ControllerCore, SmallErrorDoesNotGetMinimumCommand) {
  precision_landing::Limits limits;
  limits.final_kp_forward = 0.30;
  limits.kp_forward = 0.30;
  limits.min_forward_vel_mps = 0.10;
  limits.min_vel_error_m = 0.04;
  limits.xy_deadzone_m = 0.012;
  limits.max_accel_mps2 = 10.0;
  limits.max_down_vel_mps = 0.0;
  limits.commit_down_vel_mps = 0.0;
  limits.kd_forward = 0.0;
  auto input = freshInput();
  input.tag_position_body_frd = Eigen::Vector3d(0.02, 0.0, 0.40);
  input.dt_s = 1.0;
  const auto output = precision_landing::ControllerCore(limits).step(input);
  EXPECT_NEAR(output.velocity_body_frd.x(), 0.006, 1e-9);
}

TEST(ControllerCore, LowStartWithoutLockDoesNotCommitDescent) {
  precision_landing::Limits limits;
  limits.commit_height_m = 0.30;
  limits.commit_down_vel_mps = 0.25;
  limits.align_radius_m = 0.10;
  limits.max_z_accel_mps2 = 10.0;
  auto input = freshInput();
  input.world_target_locked = false;
  input.tag_position_body_frd = Eigen::Vector3d(0.20, 0.0, 0.24);
  input.dt_s = 1.0;
  const auto output = precision_landing::ControllerCore(limits).step(input);
  EXPECT_EQ(output.state, precision_landing::State::ALIGN);
  EXPECT_DOUBLE_EQ(output.velocity_body_frd.z(), 0.0);
}

TEST(ControllerCore, CommandsTowardTagInBodyFrame) {
  auto input = freshInput();
  input.tag_position_body_frd = Eigen::Vector3d(0.25, -0.10, 1.0);
  input.dt_s = 1.0;
  const auto output = precision_landing::ControllerCore({}).step(input);
  EXPECT_GT(output.velocity_body_frd.x(), 0.0);
  EXPECT_LT(output.velocity_body_frd.y(), 0.0);
}

TEST(ControllerCore, AlignBeforeLandHoversUntilXyWithin2cm) {
  precision_landing::Limits limits;
  limits.require_xy_align_before_land = true;
  limits.land_align_radius_m = 0.020;
  limits.final_radius_m = 0.055;
  limits.final_height_m = 0.14;
  limits.final_stable_frames = 2;
  limits.commit_height_m = 0.30;
  limits.commit_down_vel_mps = 0.25;
  limits.max_z_accel_mps2 = 10.0;
  limits.max_accel_mps2 = 10.0;
  limits.kd_forward = 0.0;
  limits.min_forward_vel_mps = 0.0;
  auto input = freshInput();
  input.world_target_locked = true;
  input.tag_position_body_frd = Eigen::Vector3d(0.04, 0.0, 0.12);
  input.dt_s = 1.0;
  precision_landing::ControllerCore core(limits);
  const auto hovering = core.step(input);
  EXPECT_EQ(hovering.state, precision_landing::State::ALIGN);
  EXPECT_DOUBLE_EQ(hovering.velocity_body_frd.z(), 0.0);
  EXPECT_GT(hovering.velocity_body_frd.x(), 0.0);
  EXPECT_FALSE(hovering.ready_to_land);
  for (int n = 0; n < 8; ++n) core.step(input);
  EXPECT_NE(core.step(input).state, precision_landing::State::READY_TO_LAND);
}

TEST(ControllerCore, AlignBeforeLandReadyWhenXyWithin2cm) {
  precision_landing::Limits limits;
  limits.require_xy_align_before_land = true;
  limits.land_align_radius_m = 0.020;
  limits.final_height_m = 0.14;
  limits.final_stable_frames = 2;
  limits.commit_down_vel_mps = 0.25;
  limits.max_z_accel_mps2 = 10.0;
  precision_landing::ControllerCore core(limits);
  auto input = freshInput();
  input.world_target_locked = true;
  input.tag_position_body_frd = Eigen::Vector3d(0.012, 0.0, 0.12);
  for (int n = 0; n < 3; ++n) core.step(input);
  const auto ready = core.step(input);
  EXPECT_EQ(ready.state, precision_landing::State::READY_TO_LAND);
  EXPECT_GT(ready.velocity_body_frd.z(), 0.0);
}

TEST(ControllerCore, BelowPadCommitKeepsDescendingDespiteXy) {
  precision_landing::Limits limits;
  limits.require_xy_align_before_land = true;
  limits.land_align_radius_m = 0.020;
  limits.final_height_m = 0.15;
  limits.pad_commit_height_m = 0.105;
  limits.commit_height_m = 0.30;
  limits.commit_down_vel_mps = 0.25;
  limits.max_z_accel_mps2 = 10.0;
  limits.max_accel_mps2 = 10.0;
  auto input = freshInput();
  input.world_target_locked = true;
  input.tag_position_body_frd = Eigen::Vector3d(0.04, 0.0, 0.09);
  input.dt_s = 1.0;
  const auto output = precision_landing::ControllerCore(limits).step(input);
  EXPECT_NE(output.state, precision_landing::State::ALIGN);
  EXPECT_GT(output.velocity_body_frd.z(), 0.0);
}

TEST(ControllerCore, FloorAlignTimeoutResumesDescentWithoutReady) {
  precision_landing::Limits limits;
  limits.require_xy_align_before_land = true;
  limits.land_align_radius_m = 0.020;
  limits.final_height_m = 0.15;
  limits.pad_commit_height_m = 0.105;
  limits.floor_align_timeout_frames = 3;
  limits.final_stable_frames = 20;
  limits.commit_down_vel_mps = 0.25;
  limits.max_z_accel_mps2 = 10.0;
  limits.max_accel_mps2 = 10.0;
  precision_landing::ControllerCore core(limits);
  auto input = freshInput();
  input.world_target_locked = true;
  input.tag_position_body_frd = Eigen::Vector3d(0.04, 0.0, 0.13);
  input.dt_s = 1.0;
  EXPECT_EQ(core.step(input).state, precision_landing::State::ALIGN);
  EXPECT_DOUBLE_EQ(core.step(input).velocity_body_frd.z(), 0.0);
  const auto resumed = core.step(input);
  EXPECT_NE(resumed.state, precision_landing::State::READY_TO_LAND);
  EXPECT_FALSE(resumed.ready_to_land);
  EXPECT_GT(resumed.velocity_body_frd.z(), 0.0);
}

TEST(ControllerCore, ReadyStaysReadyNearContact) {
  precision_landing::Limits limits;
  limits.require_xy_align_before_land = true;
  limits.land_align_radius_m = 0.020;
  limits.final_height_m = 0.15;
  limits.final_stable_frames = 2;
  limits.commit_down_vel_mps = 0.25;
  limits.max_z_accel_mps2 = 10.0;
  precision_landing::ControllerCore core(limits);
  auto input = freshInput();
  input.world_target_locked = true;
  input.tag_position_body_frd = Eigen::Vector3d(0.01, 0.0, 0.12);
  for (int n = 0; n < 3; ++n) core.step(input);
  EXPECT_EQ(core.step(input).state, precision_landing::State::READY_TO_LAND);
  input.tag_position_body_frd.z() = 0.02;
  const auto contact = core.step(input);
  EXPECT_EQ(contact.state, precision_landing::State::READY_TO_LAND);
  EXPECT_TRUE(contact.ready_to_land);
}

TEST(ControllerCore, LooseLandGateStillReadyAt4cmWhenAlignOff) {
  precision_landing::Limits limits;
  limits.require_xy_align_before_land = false;
  limits.final_radius_m = 0.055;
  limits.final_height_m = 0.14;
  limits.final_stable_frames = 2;
  precision_landing::ControllerCore core(limits);
  auto input = freshInput();
  input.tag_position_body_frd = Eigen::Vector3d(0.04, 0.0, 0.12);
  for (int n = 0; n < 3; ++n) core.step(input);
  EXPECT_EQ(core.step(input).state, precision_landing::State::READY_TO_LAND);
}
