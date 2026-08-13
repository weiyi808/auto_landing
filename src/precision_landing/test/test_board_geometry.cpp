#include <array>

#include <gtest/gtest.h>

#include "precision_landing/board_geometry.hpp"

using precision_landing::BoardCorners;

TEST(BoardGeometry, OuterTagUsesBlackSquareAndCanonicalCornerOrder) {
  BoardCorners points;
  ASSERT_TRUE(precision_landing::boardObjectCorners(10, 1.0, &points));
  EXPECT_NEAR(precision_landing::blackMarkerSizeMeters(10, 1.0), 0.068, 1e-12);
  EXPECT_NEAR(points[0].x, -0.101 + 0.034, 1e-12);
  EXPECT_NEAR(points[0].y, -0.0575 + 0.034, 1e-12);
  EXPECT_NEAR(points[1].x, -0.101 - 0.034, 1e-12);
  EXPECT_NEAR(points[1].y, -0.0575 + 0.034, 1e-12);
  EXPECT_NEAR(points[2].x, -0.101 - 0.034, 1e-12);
  EXPECT_NEAR(points[2].y, -0.0575 - 0.034, 1e-12);
  EXPECT_NEAR(points[3].x, -0.101 + 0.034, 1e-12);
  EXPECT_NEAR(points[3].y, -0.0575 - 0.034, 1e-12);
}

TEST(BoardGeometry, MiddleAndCenterUseDetectedBlackSquareSizes) {
  EXPECT_NEAR(precision_landing::blackMarkerSizeMeters(20, 1.0), 0.0288, 1e-12);
  EXPECT_NEAR(precision_landing::blackMarkerSizeMeters(0, 1.0), 0.0144, 1e-12);
  EXPECT_DOUBLE_EQ(precision_landing::blackMarkerSizeMeters(99, 1.0), 0.0);
}

TEST(BoardGeometry, ScaleAppliesToSizeAndOffsetsTogether) {
  BoardCorners points;
  ASSERT_TRUE(precision_landing::boardObjectCorners(20, 0.5, &points));
  const double center_x = (points[0].x + points[1].x) * 0.5;
  const double center_y = (points[0].y + points[2].y) * 0.5;
  EXPECT_NEAR(center_x, -0.0365 * 0.5, 1e-12);
  EXPECT_NEAR(center_y, -0.039 * 0.5, 1e-12);
  EXPECT_FALSE(precision_landing::boardObjectCorners(99, 1.0, &points));
}
