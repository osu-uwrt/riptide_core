#pragma once

#include "riptide_gyro/gyro_base.hpp"

#if defined(USE_LINUX)

#include <gtest/gtest.h>

class RosTest : public ::testing::Test
{
    public:
    static void initTest(int argc, char **argv);

    protected:
    void SetUp() override;
    void TearDown() override;

    std::string homeDir();
    rclcpp::Node::SharedPtr rosNode;
    
    private:
    static char **argv;
    static int argc;
    std::string home;
};

#endif
