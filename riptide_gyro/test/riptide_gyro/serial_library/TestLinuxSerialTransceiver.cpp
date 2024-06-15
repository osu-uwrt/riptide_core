#include "riptide_gyro/serial_library.hpp"
#include "riptide_gyro/testing.hpp"

#if defined(TESTING_ENABLED)


TEST_F(LinuxTransceiverTest, TestTransceiverRdWr)
{
    uwrt_gyro::LinuxSerialTransceiver transceiver1(
        rosNode,
        homeDir() + "virtualsp1",
        9600,
        1,
        0);
    
    uwrt_gyro::LinuxSerialTransceiver transceiver2(
        rosNode,
        homeDir() + "virtualsp2",
        9600,
        1,
        0
    );

    const std::string
        expectedMsg1 = "Hello world!",
        expectedMsg2 = "Hello world 2!!";

    char buf[256];
    
    transceiver1.init();
    transceiver2.init();
    transceiver1.send(expectedMsg1.c_str(), expectedMsg1.length());
    size_t recvd1 = transceiver2.recv(buf, sizeof(buf));
    std::string msg1((char*) buf);
    transceiver2.send(expectedMsg2.c_str(), expectedMsg2.length());
    size_t recvd2 = transceiver1.recv(buf, sizeof(buf));
    std::string msg2((char*) buf);
    transceiver1.deinit();
    transceiver2.deinit();

    ASSERT_EQ(expectedMsg1, msg1);
    ASSERT_EQ(msg1.length(), recvd1);
    ASSERT_EQ(expectedMsg2, msg2);
    ASSERT_EQ(msg2.length(), recvd2);
}


TEST_F(LinuxTransceiverTest, TestTransceiverRoWo)
{
    uwrt_gyro::LinuxSerialTransceiver transceiver1(
        rosNode,
        homeDir() + "virtualsp1",
        9600,
        1,
        0,
        O_WRONLY);
    
    uwrt_gyro::LinuxSerialTransceiver transceiver2(
        rosNode,
        homeDir() + "virtualsp2",
        9600,
        1,
        0,
        O_RDONLY);

    const std::string expectedMsg = "Hello world again!";

    char buf[256];
    
    transceiver1.init();
    transceiver2.init();
    transceiver1.send(expectedMsg.c_str(), expectedMsg.length());
    size_t recvd1 = transceiver2.recv(buf, sizeof(buf));
    std::string msg1(buf);
    transceiver2.send("stuff", sizeof("stuff"));
    transceiver1.recv(buf, sizeof(buf));
    std::string msg2(buf);
    transceiver1.deinit();
    transceiver2.deinit();

    ASSERT_EQ(expectedMsg, msg1);
    ASSERT_EQ(msg1.length(), recvd1);
    ASSERT_EQ("", msg2);
}

#endif
