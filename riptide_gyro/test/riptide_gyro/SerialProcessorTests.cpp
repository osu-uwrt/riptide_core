#include "riptide_gyro/testing.hpp"

using namespace std::chrono_literals;

bool LinuxSerialProcessorTest::waitForFrame(SerialFrameId id, Time startTime)
{
    Time now = rosNode->get_clock()->now();
    while(now - startTime < 500ms)
    {
        processor->update(now);
        if(processor->getField(id).timestamp >= startTime)
        {
            return true;
        }

        now = rosNode->get_clock()->now();
    }

    std::cout << "Timed out waiting for frame with id " << std::to_string(id) << std::endl;
    return false;
}

void Type1SerialProcessorTest::SetUp() 
{
    LinuxTransceiverTest::SetUp();
    transceiver = uwrt_gyro::LinuxSerialTransceiver(
        rosNode,
        homeDir() + "virtualsp1",
        9600,
        0, 
        1);
    
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

    const char syncValue[1] = {'A'};
    processor = std::make_shared<uwrt_gyro::SerialProcessor>(
        transceiver, 
        frameMap,
        Type1SerialFrames1::TYPE_1_FRAME_1, 
        syncValue, 
        sizeof(syncValue));
}


void Type1SerialProcessorTest::TearDown()
{
    LinuxTransceiverTest::TearDown();
}


void Type2SerialProcessorTest::SetUp() 
{
    LinuxTransceiverTest::SetUp();
    transceiver = uwrt_gyro::LinuxSerialTransceiver(
        rosNode,
        homeDir() + "virtualsp1",
        9600,
        0,
        1);
    
    frameMap = {
        {Type2SerialFrames1::TYPE_2_FRAME_1, 
            {
                TYPE_2_FIELD_1,
                FIELD_FRAME,
                TYPE_2_FIELD_2,
                TYPE_2_FIELD_2,
                FIELD_SYNC,
                TYPE_2_FIELD_2,
                TYPE_2_FIELD_3
            }
        },
        {Type2SerialFrames1::TYPE_2_FRAME_2, 
            {
                TYPE_2_FIELD_2,
                FIELD_FRAME,
                TYPE_2_FIELD_2,
                TYPE_2_FIELD_4,
                FIELD_SYNC,
                TYPE_2_FIELD_3,
                TYPE_2_FIELD_2,
            }
        },
        {Type2SerialFrames1::TYPE_2_FRAME_3, 
            {
                TYPE_2_FIELD_5,
                FIELD_FRAME,
                TYPE_2_FIELD_1,
                TYPE_2_FIELD_6,
                FIELD_SYNC,
                TYPE_2_FIELD_5,
                TYPE_2_FIELD_6
            }
        }
    };

    const char syncValue[1] = {'A'};
    processor = std::make_shared<uwrt_gyro::SerialProcessor>(
        transceiver,
        frameMap,
        Type1SerialFrames1::TYPE_1_FRAME_1,
        syncValue,
        sizeof(syncValue));
}

void Type2SerialProcessorTest::TearDown()
{
    LinuxTransceiverTest::TearDown();
}
