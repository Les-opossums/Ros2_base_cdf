#include <gtest/gtest.h>
#include <stdexcept>
#include "localization/math_lidar.hpp"

TEST(UnitVectorTest, HandlesNonZeroVector) {
    Eigen::Vector2d vec(3.0, 4.0);
    Eigen::Vector2d result = unit_vector(vec);
    
    EXPECT_NEAR(result.norm(), 1.0, 1e-6);  // Check if the resulting vector is of unit length
    EXPECT_NEAR(result.x(), 3.0 / 5.0, 1e-6);  // Check x-component
    EXPECT_NEAR(result.y(), 4.0 / 5.0, 1e-6);  // Check y-component
}

TEST(UnitVectorTest, HandlesZeroVector) {
    Eigen::Vector2d vec(0.0, 0.0);
    EXPECT_ANY_THROW(unit_vector(vec));  // Expecting an exception if dividing by zero
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}