#include "riptide_gyro/testing.hpp"

TEST_F(RosTest, InitialTest)
{
    RCLCPP_INFO(rosNode->get_logger(), "Hello world from gyro test!");
    GTEST_SKIP();
}
