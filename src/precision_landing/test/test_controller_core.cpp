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
  for (int n = 0; n < 20; ++n) core.step(input);
  EXPECT_EQ(core.step(input).state, precision_landing::State::READY_TO_LAND);
}

TEST(ControllerCore, LateralErrorBlocksDescent) {
  auto input = freshInput();
  input.tag_position_body_frd.x() = 0.25;
  EXPECT_EQ(precision_landing::ControllerCore({}).step(input).state,
            precision_landing::State::ALIGN);
}

TEST(ControllerCore, CommandsTowardTagInBodyFrame) {
  auto input = freshInput();
  input.tag_position_body_frd = Eigen::Vector3d(0.25, -0.10, 1.0);
  input.dt_s = 1.0;
  const auto output = precision_landing::ControllerCore({}).step(input);
  EXPECT_GT(output.velocity_body_frd.x(), 0.0);
  EXPECT_LT(output.velocity_body_frd.y(), 0.0);
}
