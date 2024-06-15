#pragma once

#include "riptide_gyro/serial_library.hpp"

#if defined(USE_LINUX)

#include <gtest/gtest.h>

class RosTest : public ::testing::Test
{
    public:
    static void initTest(int argc, char **argv);

    protected:
    void SetUp() override;
    void TearDown() override;
    bool compareSerialData(const SerialData& data1, const SerialData& data2);
    std::string homeDir();
    rclcpp::Node::SharedPtr rosNode;
    
    private:
    static char **argv;
    static int argc;
    std::string home;
};

class LinuxTransceiverTest : public RosTest
{
    protected:
    void SetUp() override;
    void TearDown() override;

    private:
    pid_t socatProc;
};

//
// type1 processor test stuff
//
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
    void SetUp() override;
    void TearDown() override;

    SerialFramesMap frameMap;
    uwrt_gyro::LinuxSerialTransceiver transceiver;
    uwrt_gyro::SerialProcessor::SharedPtr processor;
};

enum Type2SerialFrames1
{
    TYPE_2_FRAME_1,
    TYPE_2_FRAME_2,
    TYPE_2_FRAME_3
};

enum Type2SerialFields
{
    TYPE_2_FIELD_1,
    TYPE_2_FIELD_2,
    TYPE_2_FIELD_3,
    TYPE_2_FIELD_4,
    TYPE_2_FIELD_5,
    TYPE_2_FIELD_6
};

class Type2SerialProcessorTest : public LinuxTransceiverTest
{
    protected:
    void SetUp() override;
    void TearDown() override;

    SerialFramesMap frameMap;
    uwrt_gyro::LinuxSerialTransceiver transceiver;
    uwrt_gyro::SerialProcessor::SharedPtr processor;
};

#endif
