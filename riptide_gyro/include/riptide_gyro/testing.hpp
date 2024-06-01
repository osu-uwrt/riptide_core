#pragma once

#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

class RosTest : public ::testing::Test
{
    public:
    static void initTest(int argc, char **argv);

    protected:
    void SetUp() override;
    void TearDown() override;

    rclcpp::Node::SharedPtr rosNode;
    
    private:
    static char **argv;
    static int argc;
};
