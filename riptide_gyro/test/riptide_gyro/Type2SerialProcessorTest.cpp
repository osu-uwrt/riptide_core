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
        {Type2SerialFrames1::TYPE_2_FRAME_1, 
            {
                FIELD_SYNC,
                TYPE_2_FIELD_1,
                FIELD_FRAME,
                TYPE_2_FIELD_2,
                TYPE_2_FIELD_2,
                TYPE_2_FIELD_2,
                TYPE_2_FIELD_3
            }
        },
        {Type2SerialFrames1::TYPE_2_FRAME_2, 
            {
                TYPE_2_FIELD_2,
                TYPE_2_FIELD_2,
                FIELD_FRAME,
                TYPE_2_FIELD_4,
                TYPE_2_FIELD_3,
                TYPE_2_FIELD_2,
                FIELD_SYNC
            }
        },
        {Type2SerialFrames1::TYPE_2_FRAME_3, 
            {
                TYPE_2_FIELD_5,
                FIELD_SYNC,
                TYPE_2_FIELD_1,
                FIELD_FRAME,
                TYPE_2_FIELD_6,
                TYPE_2_FIELD_5,
                TYPE_2_FIELD_6
            }
        }
    };

    const char syncValue[2] = "A";
    processor = std::make_shared<uwrt_gyro::SerialProcessor>(
        transceiver,
        frameMap,
        Type1SerialFrames1::TYPE_1_FRAME_1,
        syncValue);
}

void Type1SerialProcessorTest::TearDown()
{
    LinuxTransceiverTest::TearDown();
}
