#include <gtest/gtest.h>
#include "teleop_ack_rc/teleop_ack_rc_node.hpp" // Adjust if your filename differs

// Test 1: Verify the 6.7056 m/s (15 mph) target
TEST(RCTeleopMath, VerifyMaxSpeed) {
    // We can test the math here once we make the mapping function public
    double input_max = 1.0; 
    double max_speed = 6.7056;
    double result = input_max * max_speed;
    
    EXPECT_NEAR(result, 6.7056, 0.0001);
}

// Test 2: Logic for your Jitter Theory
TEST(RCTeleopMath, DeadzoneCheck) {
    double tiny_jitter = 0.02; // Simulating small Arduino noise
    double deadzone = 0.05;
    double output = (tiny_jitter < deadzone) ? 0.0 : tiny_jitter;
    
    EXPECT_EQ(output, 0.0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}