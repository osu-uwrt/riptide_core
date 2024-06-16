#include "riptide_gyro/serial_library.hpp"
#include "riptide_gyro/testing.hpp"

using namespace std::chrono_literals;

TEST_F(Type1SerialProcessorTest, TestBasicRecvWithManualSendType1)
{
    uwrt_gyro::LinuxSerialTransceiver client(
        rosNode,
        homeDir() + "virtualsp2",
        9600,
        1,
        0);
    
    client.init();
    
    const char msg[] = "AqweA";

    client.send(msg, sizeof(msg));
    
    Time startTime = rosNode->get_clock()->now();
    processor->update(rosNode->get_clock()->now());

    ASSERT_TRUE(processor->hasDataForField(FIELD_SYNC));

    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_1_FRAME_1_FIELD_1).data, uwrt_gyro::serialDataFromString("q", 1)));
    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_1_FRAME_1_FIELD_2).data, uwrt_gyro::serialDataFromString("w", 1)));
    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_1_FRAME_1_FIELD_3).data, uwrt_gyro::serialDataFromString("e", 1)));
}

TEST_F(Type1SerialProcessorTest, TestBasicSendWithManualRecvType1)
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

    processor->send(TYPE_1_FRAME_1);

    char buf[4];
    client.recv(buf, 4);
    
    ASSERT_TRUE(memcmp(buf, "Aabc", 4) == 0);
}

TEST_F(Type2SerialProcessorTest, TestBasicRecvWithManualSendType2)
{
    uwrt_gyro::LinuxSerialTransceiver client(
        rosNode,
        homeDir() + "virtualsp2",
        9600,
        1,
        0);
    
    client.init();

    //define messages (to be sent in increasing order)
    const char
        msg1[] = {'A', 'a', 0, 'b', 'c', 'A', 'd', 'e'},
        msg2[] = {'a', 1, 'b', 'c', 'A', 'd', 'e'},
        msg3[] = {'q', 'A', 'b', 2, 'c', 'd', 'A', 'e', 'z'};
    
    Time now = rosNode->get_clock()->now();

    client.send(msg1, sizeof(msg1));
    processor->update(now);

    ASSERT_TRUE(processor->hasDataForField(FIELD_SYNC));
    // ASSERT_TRUE(strcmp(processor->getField(FIELD_SYNC).data.data, "A") == 0);
    SerialDataStamped data = processor->getField(FIELD_FRAME);
    int frameId = uwrt_gyro::convertFromCString<int>(data.data.data, data.data.numData);
    ASSERT_EQ(frameId, 0);

    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_2_FIELD_1).data, uwrt_gyro::serialDataFromString("a", 1)));
    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_2_FIELD_2).data, uwrt_gyro::serialDataFromString("bcd", 3)));
    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_2_FIELD_3).data, uwrt_gyro::serialDataFromString("e", 1)));

    client.send(msg2, sizeof(msg2));
    processor->update(now);

    ASSERT_TRUE(processor->hasDataForField(FIELD_SYNC));
    data = processor->getField(FIELD_FRAME);
    frameId = uwrt_gyro::convertFromCString<int>(data.data.data, data.data.numData);
    ASSERT_EQ(frameId, 1);
    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_2_FIELD_2).data, uwrt_gyro::serialDataFromString("abe", 3)));
    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_2_FIELD_3).data, uwrt_gyro::serialDataFromString("d", 1)));
    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_2_FIELD_4).data, uwrt_gyro::serialDataFromString("c", 1)));
    
    client.send(msg3, sizeof(msg3));
    processor->update(now);

    ASSERT_TRUE(processor->hasDataForField(FIELD_SYNC));
    data = processor->getField(FIELD_FRAME);
    frameId = uwrt_gyro::convertFromCString<int>(data.data.data, data.data.numData);
    ASSERT_EQ(frameId, 2);
    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_2_FIELD_1).data, uwrt_gyro::serialDataFromString("c", 1)));
    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_2_FIELD_5).data, uwrt_gyro::serialDataFromString("be", 2)));
    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_2_FIELD_6).data, uwrt_gyro::serialDataFromString("dz", 2)));
}

TEST_F(Type2SerialProcessorTest, TestBasicRecvAndSendType2)
{
    //create another processor to recv
    uwrt_gyro::LinuxSerialTransceiver sender(
        rosNode, 
        homeDir() + "/virtualsp2",
        9600,
        1,
        0);
    
    const char syncValue[1] = {'A'};
    uwrt_gyro::SerialProcessor senderProcessor(
        sender,
        frameMap,
        Type1SerialFrames1::TYPE_1_FRAME_1,
        syncValue,
        sizeof(syncValue));

    //test sending frame 1
    Time now = rosNode->get_clock()->now();
    senderProcessor.setField(TYPE_2_FIELD_1, uwrt_gyro::serialDataFromString("1", 1), now);
    senderProcessor.setField(TYPE_2_FIELD_2, uwrt_gyro::serialDataFromString("234", 3), now);
    senderProcessor.setField(TYPE_2_FIELD_3, uwrt_gyro::serialDataFromString("5", 1), now);

    senderProcessor.send(TYPE_2_FRAME_1);
    senderProcessor.send(TYPE_2_FRAME_1);
    
    ASSERT_TRUE(waitForFrame(TYPE_2_FIELD_1, now));
    ASSERT_TRUE(waitForFrame(TYPE_2_FIELD_2, now));
    ASSERT_TRUE(waitForFrame(TYPE_2_FIELD_3, now));

    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_2_FIELD_1).data, uwrt_gyro::serialDataFromString("1", 1)));
    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_2_FIELD_2).data, uwrt_gyro::serialDataFromString("234", 3)));
    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_2_FIELD_3).data, uwrt_gyro::serialDataFromString("5", 1)));

    //test sending frame 2
    now = rosNode->get_clock()->now();
    senderProcessor.setField(TYPE_2_FIELD_2, uwrt_gyro::serialDataFromString("abc", 3), now);
    senderProcessor.setField(TYPE_2_FIELD_3, uwrt_gyro::serialDataFromString("d", 1), now);
    senderProcessor.setField(TYPE_2_FIELD_4, uwrt_gyro::serialDataFromString("E", 1), now);

    senderProcessor.send(TYPE_2_FRAME_2);
    senderProcessor.send(TYPE_2_FRAME_2);
    
    ASSERT_TRUE(waitForFrame(TYPE_2_FIELD_2, now));
    ASSERT_TRUE(waitForFrame(TYPE_2_FIELD_3, now));
    ASSERT_TRUE(waitForFrame(TYPE_2_FIELD_4, now));

    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_2_FIELD_2).data, uwrt_gyro::serialDataFromString("abc", 3)));
    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_2_FIELD_3).data, uwrt_gyro::serialDataFromString("d", 1)));
    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_2_FIELD_4).data, uwrt_gyro::serialDataFromString("E", 1)));

    //test sending frame 3
    now = rosNode->get_clock()->now();
    senderProcessor.setField(TYPE_2_FIELD_5, uwrt_gyro::serialDataFromString("zy", 2), now);
    senderProcessor.setField(TYPE_2_FIELD_6, uwrt_gyro::serialDataFromString("xw", 2), now);
    senderProcessor.setField(TYPE_2_FIELD_1, uwrt_gyro::serialDataFromString("s", 1), now);

    senderProcessor.send(TYPE_2_FRAME_3);
    senderProcessor.send(TYPE_2_FRAME_3);
    
    ASSERT_TRUE(waitForFrame(TYPE_2_FIELD_5, now));
    ASSERT_TRUE(waitForFrame(TYPE_2_FIELD_6, now));
    ASSERT_TRUE(waitForFrame(TYPE_2_FIELD_1, now));

    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_2_FIELD_5).data, uwrt_gyro::serialDataFromString("zy", 2)));
    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_2_FIELD_6).data, uwrt_gyro::serialDataFromString("xw", 2)));
    ASSERT_TRUE(compareSerialData(processor->getField(TYPE_2_FIELD_1).data, uwrt_gyro::serialDataFromString("s", 1)));
}

TEST(GenericType2SerialProcessorTest, TestConstructorNonContinuousSync)
{
    GTEST_SKIP();
}

TEST(GenericType2SerialProcessorTest, TestConstructorDefaultFrameUndefined)
{
    GTEST_SKIP();
}

TEST(GenericType2SerialProcessorTest, TestConstructorSyncValueNotInFrameList)
{
    GTEST_SKIP();
}

TEST(GenericType2SerialProcessorTest, TestConstructorFrameIdValueNotCommonBetweenFrames)
{
    GTEST_SKIP();
}
