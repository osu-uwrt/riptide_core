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


bool RosTest::compareSerialData(const SerialData& data1, const SerialData& data2)
{
    if(data1.numData != data2.numData)
    {
        std::cout << "data1 with length " << data1.numData << " does not match data2 " << data2.numData << "\n";
        std::cout << "data1: \"" << data1.data << "\", data2: \"" << data2.data << "\"" << std::endl;
        return false;
    }

    if(memcmp(data1.data, data2.data, data1.numData) != 0)
    {
        std::cout << "data1 with length " << data1.numData << " does not match data2 " << data2.numData << "\n";
        std::cout << "data1: \"" << data1.data << "\", data2: \"" << data2.data << "\"" << std::endl;
        return false;
    }

    return true;
}


std::string RosTest::homeDir()
{
    return home;
}
