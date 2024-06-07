#include "riptide_gyro/testing.hpp"

void Type1SerialProcessorTest::SetUp() 
{
    LinuxTransceiverTest::SetUp();
    transceiver = uwrt_gyro::LinuxSerialTransceiver(
        rosNode,
        homeDir() + "virtualsp1",
        9600,
        1,
        0);
    
    frameMap = {
        {Type1SerialFrames1::TYPE_1_FRAME_1, 
            {
                FIELD_SYNC,
                Type1SerialFrame1Fields::TYPE_1_FRAME_1_FIELD_1,
                Type1SerialFrame1Fields::TYPE_1_FRAME_1_FIELD_2,
                Type1SerialFrame1Fields::TYPE_1_FRAME_1_FIELD_3,
            }
        }
    };

    const char syncValue[2] = "A";
    processor = std::make_shared<uwrt_gyro::SerialProcessor>(transceiver, frameMap, Type1SerialFrames1::TYPE_1_FRAME_1, syncValue);
}

void Type1SerialProcessorTest::TearDown()
{
    LinuxTransceiverTest::TearDown();
}
