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

    char *homeVar = getenv("HOME");
    if(!homeVar) {
        std::cerr << "HOME could not be determined! Reverting to current directory" << std::endl;
        home = "";
        return;
    }

    home = std::string(homeVar);
    if(*home.end() != '/')
    {
        home += "/";
    }
}


void RosTest::TearDown()
{
    rclcpp::shutdown();
}


std::string RosTest::homeDir()
{
    return home;
}
