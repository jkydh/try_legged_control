#include <gtest/gtest.h>
#include <legged_common/hardware_interface/HybridJointTorque.h>
#include <cmath>
#include <limits>

using legged::computeHybridJointTorque;

TEST(HybridTorqueLimit, LimitsSumOfFeedbackAndFeedforward) {
  const auto result = computeHybridJointTorque(1, 1, 8, 8, 8, 0, 0, 20);
  EXPECT_DOUBLE_EQ(result.requested, 24);
  EXPECT_DOUBLE_EQ(result.applied, 20);
  const auto negative = computeHybridJointTorque(-1, -1, 8, 8, -8, 0, 0, 20);
  EXPECT_DOUBLE_EQ(negative.requested, -24);
  EXPECT_DOUBLE_EQ(negative.applied, -20);
}

TEST(HybridTorqueLimit, PreservesInRangeCommandAndItsSign) {
  const auto result = computeHybridJointTorque(0.1, -0.5, 40, 2, -3, 0, 0, 20);
  EXPECT_DOUBLE_EQ(result.position, 4);
  EXPECT_DOUBLE_EQ(result.velocity, -1);
  EXPECT_DOUBLE_EQ(result.applied, 0);
  EXPECT_DOUBLE_EQ(computeHybridJointTorque(0, 0, 0, 0, -7.5, 0, 0, 20).applied, -7.5);
}

TEST(HybridTorqueLimit, IncludesEvenSmallOvershoots) {
  EXPECT_DOUBLE_EQ(computeHybridJointTorque(0, 0, 0, 0, std::nextafter(20.0, 21.0), 0, 0, 20).applied, 20);
  EXPECT_DOUBLE_EQ(computeHybridJointTorque(0, 0, 0, 0, std::nextafter(-20.0, -21.0), 0, 0, 20).applied, -20);
}

TEST(HybridTorqueLimit, NonfiniteCommandsAndInvalidLimitsFailToZero) {
  for (double invalid : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                         -std::numeric_limits<double>::infinity()}) {
    EXPECT_DOUBLE_EQ(computeHybridJointTorque(0, 0, 0, 0, invalid, 0, 0, 20).applied, 0);
    EXPECT_DOUBLE_EQ(computeHybridJointTorque(0, 0, 0, 0, 10, 0, 0, invalid).applied, 0);
  }
  EXPECT_DOUBLE_EQ(computeHybridJointTorque(0, 0, 0, 0, 10, 0, 0, 0).applied, 0);
  EXPECT_DOUBLE_EQ(computeHybridJointTorque(0, 0, 0, 0, 10, 0, 0, -20).applied, 0);
}
