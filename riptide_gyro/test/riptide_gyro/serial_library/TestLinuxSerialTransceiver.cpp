#include "riptide_gyro/serial_library.hpp"
#include "riptide_gyro/testing.hpp"
#include <filesystem>

#if defined(TESTING_ENABLED)

class LinuxTransceiverTest : public RosTest
{
    protected:
    void SetUp() override
    {
        RosTest::SetUp();

        char
            virtualFile1[24],
            virtualFile2[24],
            virtualSerialArg1[64],
            virtualSerialArg2[64];

        sprintf(virtualFile1, "%s/virtualsp1", homeDir().c_str());
        sprintf(virtualFile2, "%s/virtualsp2", homeDir().c_str());
        sprintf(virtualSerialArg1, "PTY,link=%s,raw,echo=0", virtualFile1);
        sprintf(virtualSerialArg2, "PTY,link=%s,raw,echo=0", virtualFile2);
        
        // create a virtual terminal using socat
        pid_t proc = fork();
        if(proc < 0)
        {
            RCLCPP_FATAL(rosNode->get_logger(), "fork() failed: %s", strerror(errno));
            exit(1);
        } else if(proc == 0)
        {
            // child process
            const char *socatCommand[] = {
                "/usr/bin/socat",
                virtualSerialArg1,
                virtualSerialArg2,
                NULL
            };

            if(execve(socatCommand[0], (char *const*) socatCommand, NULL) < 0)
            {
                RCLCPP_ERROR(rosNode->get_logger(), "execve() failed: %s", strerror(errno));
            }
            exit(0);
        } else
        {
            //parent process
            socatProc = proc;

            //wait for file created by socat to appear
            while(!(std::filesystem::exists(virtualFile1) || std::filesystem::exists(virtualFile2)))
            {
                usleep(1000);
            }
        }
    }

    void TearDown() override
    {
        int sig = SIGINT;
        if(kill(socatProc, sig) < 0)
        {
            RCLCPP_ERROR(rosNode->get_logger(), "kill() returned error: %s", strerror(errno));
        }

        
        if(waitpid(socatProc, NULL, 0) < 0)
        {
            RCLCPP_ERROR(rosNode->get_logger(), "waitpid() failed: %s", strerror(errno));
        }

        RosTest::TearDown();
    }

    private:
    pid_t socatProc;
};


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
    transceiver1.send(expectedMsg1);
    size_t recvd1 = transceiver2.recv(buf, sizeof(buf));
    std::string msg1((char*) buf);
    transceiver2.send(expectedMsg2);
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
    transceiver1.send(expectedMsg);
    size_t recvd1 = transceiver2.recv(buf, sizeof(buf));
    std::string msg1(buf);
    transceiver2.send("stuff");
    transceiver1.recv(buf, sizeof(buf));
    std::string msg2(buf);
    transceiver1.deinit();
    transceiver2.deinit();

    ASSERT_EQ(expectedMsg, msg1);
    ASSERT_EQ(msg1.length(), recvd1);
    ASSERT_EQ("", msg2);
}

#endif
