#include "riptide_gyro/serial_library.hpp"
#include "riptide_gyro/testing.hpp"

using namespace std::chrono_literals;

TEST_F(Type1SerialProcessorTest, TestRecvWithManualSend)
{
    uwrt_gyro::LinuxSerialTransceiver client(
        rosNode,
        homeDir() + "virtualsp2",
        9600,
        1,
        0);
    
    client.init();
    
    const char *msg = "Aqwerty";

    client.send(msg);
    
    Time startTime = rosNode->get_clock()->now();
    while(!processor->hasDataForField(FIELD_SYNC) && rosNode->get_clock()->now() - startTime < 1s)
    {
        processor->update(rosNode->get_clock()->now());
    }

    ASSERT_TRUE(processor->hasDataForField(FIELD_SYNC));
    ASSERT_EQ(std::string(processor->getField(TYPE_1_FRAME_1_FIELD_1).data.data), "q");
    ASSERT_EQ(std::string(processor->getField(TYPE_1_FRAME_1_FIELD_2).data.data), "w");
    ASSERT_EQ(std::string(processor->getField(TYPE_1_FRAME_1_FIELD_3).data.data), "e");
}

TEST_F(Type1SerialProcessorTest, TestSendWithManualRecv)
{
    uwrt_gyro::LinuxSerialTransceiver client(
        rosNode,
        homeDir() + "virtualsp2",
        9600,
        1,
        0);
    
    client.init();

    //pack msg and send
    processor->setField(TYPE_1_FRAME_1_FIELD_1, uwrt_gyro::serialDataFromString("a", 1), rosNode->get_clock()->now());
    processor->setField(TYPE_1_FRAME_1_FIELD_2, uwrt_gyro::serialDataFromString("b", 1), rosNode->get_clock()->now());
    processor->setField(TYPE_1_FRAME_1_FIELD_3, uwrt_gyro::serialDataFromString("c", 1), rosNode->get_clock()->now());
}
