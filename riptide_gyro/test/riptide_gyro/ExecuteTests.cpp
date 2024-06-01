#include "riptide_gyro/testing.hpp"

int main(int argc, char **argv)
{
    RosTest::initTest(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}