#include "riptide_gyro/testing.hpp"
#include <filesystem>

void LinuxTransceiverTest::SetUp()
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

    //delete files if they already exist
    if(std::filesystem::exists(virtualFile1))
    {
        std::filesystem::remove(virtualFile1);
    }

    if(std::filesystem::exists(virtualFile2))
    {
        std::filesystem::remove(virtualFile2);
    }
    
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
        while(!(std::filesystem::exists(virtualFile1) && std::filesystem::exists(virtualFile2)))
        {
            usleep(1000);
        }
    }
}


void LinuxTransceiverTest::TearDown()
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
