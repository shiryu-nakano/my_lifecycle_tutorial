#include <gtest/gtest.h>
#include "lifecycle_supervisor/supervisor_component.hpp"

using namespace lifecycle_supervisor;

TEST(SupervisorComponentTest, DistanceCalculation) {
    Pose2D a{0, 0, 0};
    Pose2D b{3, 4, 0};
    EXPECT_DOUBLE_EQ(SupervisorComponent::CalcDistance(a, b), 5.0);
}

TEST(SupervisorComponentTest, ThresholdCheck) {
    Pose2D a{0, 0, 0};
    Pose2D b{1, 0, 0};
    EXPECT_TRUE(SupervisorComponent::IsWithinThreshold(a, b, 1.5));
    EXPECT_FALSE(SupervisorComponent::IsWithinThreshold(a, b, 0.5));
}
