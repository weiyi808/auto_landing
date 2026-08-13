#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "precision_landing/landing_accuracy.hpp"

namespace precision_landing {
namespace {

TEST(LandingAccuracyTest, SelectsNewestSampleWithinTouchdownWindow) {
  const std::vector<LandingAccuracySample> samples = {
      {8.8, 1.0, 1.0, 1.0},
      {9.2, 0.30, 0.40, 0.20},
      {9.9, 0.03, -0.04, 0.10},
  };

  const LandingAccuracyResult result =
      selectFinalAccuracy(samples, 10.0, 1.0);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.sample.stamp_s, 9.9);
  EXPECT_DOUBLE_EQ(result.sample.forward_m, 0.03);
  EXPECT_DOUBLE_EQ(result.sample.right_m, -0.04);
  EXPECT_NEAR(result.horizontal_error_m, 0.05, 1e-12);
  EXPECT_EQ(result.in_window_count, 2U);
}

TEST(LandingAccuracyTest, RejectsAllSamplesOlderThanWindow) {
  const std::vector<LandingAccuracySample> samples = {
      {8.0, 0.01, 0.02, 0.03},
      {8.9, 0.04, 0.05, 0.06},
  };

  const LandingAccuracyResult result =
      selectFinalAccuracy(samples, 10.0, 1.0);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.in_window_count, 0U);
}

TEST(LandingAccuracyTest, IgnoresSamplesAfterTouchdownAndNonFiniteSamples) {
  const std::vector<LandingAccuracySample> samples = {
      {9.8, 0.06, 0.08, 0.02},
      {10.1, 0.0, 0.0, 0.0},
      {9.9, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
  };

  const LandingAccuracyResult result =
      selectFinalAccuracy(samples, 10.0, 1.0);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.sample.stamp_s, 9.8);
  EXPECT_NEAR(result.horizontal_error_m, 0.10, 1e-12);
  EXPECT_EQ(result.in_window_count, 1U);
}

}  // namespace
}  // namespace precision_landing
