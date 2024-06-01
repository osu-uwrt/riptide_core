#include "riptide_gyro/testing.hpp"


int RosTest::argc = 0;
char **RosTest::argv = nullptr;

void RosTest::initTest(int argc, char **argv)
{
    RosTest::argc = argc;
    RosTest::argv = argv;
}


void RosTest::SetUp()
{
    rclcpp::init(RosTest::argc, RosTest::argv);
    rosNode = std::make_shared<rclcpp::Node>("serial_library_test_node");
}


void RosTest::TearDown()
{
    rclcpp::shutdown();
}
