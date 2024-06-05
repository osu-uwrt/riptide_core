#include "riptide_gyro/serial_library.hpp"
#include "riptide_gyro/testing.hpp"

enum Type1SerialFrames1
{
    TYPE_1_FRAME_1
};

enum Type1SerialFrame1Fields
{
    TYPE_1_FRAME_1_FIELD_1,
    TYPE_1_FRAME_1_FIELD_2,
    TYPE_1_FRAME_1_FIELD_3
};

class Type1SerialProcessorTest : public LinuxTransceiverTest
{
    protected:
    void SetUp() override
    {
        LinuxTransceiverTest::SetUp();
        transceiver = uwrt_gyro::LinuxSerialTransceiver(
            rosNode,
            homeDir() + "virtualsp1",
            9600,
            1,
            0);
        
        SerialFramesMap map = {
            {Type1SerialFrames1::TYPE_1_FRAME_1, 
                {
                    FIELD_SYNC,
                    Type1SerialFrame1Fields::TYPE_1_FRAME_1_FIELD_1,
                    Type1SerialFrame1Fields::TYPE_1_FRAME_1_FIELD_2,
                    Type1SerialFrame1Fields::TYPE_1_FRAME_1_FIELD_3,
                }
            }
        };

        const char syncValue[2] = "\xdd";
        processor = std::make_shared<uwrt_gyro::SerialProcessor>(transceiver, map, Type1SerialFrames1::TYPE_1_FRAME_1, syncValue);
    }

    void TearDown() override
    {
        LinuxTransceiverTest::TearDown();
    }

    uwrt_gyro::LinuxSerialTransceiver transceiver;
    uwrt_gyro::SerialProcessor::SharedPtr processor;
};


TEST_F(Type1SerialProcessorTest, TestStuff)
{
    GTEST_SKIP();
}
