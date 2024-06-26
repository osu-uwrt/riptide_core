#include "riptide_gyro/serial_library.hpp"

#if defined(USE_LINUX)

// linux serial port implementation: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

namespace uwrt_gyro
{

    LinuxSerialTransceiver::LinuxSerialTransceiver(
        const rclcpp::Node::SharedPtr node, 
        const std::string& fileName, 
        int baud,
        int minimumBytes,
        int maximumTimeout,
        int mode,
        int bitsPerByte,
        bool twoStopBits,
        bool parityBit)
          : rosNode(node),
            fileName(fileName),
            baud(baud),
            mode(mode),
            bitsPerByte(bitsPerByte),
            minimumBytes(minimumBytes),
            maximumTimeout(maximumTimeout),
            initialized(false),
            twoStopBits(twoStopBits),
            parityBit(parityBit) { }
    

    bool LinuxSerialTransceiver::init(void)
    {
        //
        // SERIAL PORT OPEN
        //
        file = open(fileName.c_str(), mode);
        if(file < 0)
        {
            RCLCPP_FATAL(rosNode->get_logger(), "Could not open file %s: %s", fileName.c_str(), strerror(errno));
            initialized = false;
            return false;
        }

        //
        // SERIAL PORT CONFIGURATION
        //
        struct termios config;
        
        //init config with current settings
        if(tcgetattr(file, &config) < 0)
        {
            RCLCPP_FATAL(rosNode->get_logger(), "tcgetattr() failed: %s", strerror(errno));
            initialized = false;
            return false;
        }

        //configure bits per byte
        config.c_cflag &= ~CSIZE;
        config.c_cflag |= bitsPerByte;

        //configure parity
        config.c_cflag = (parityBit ? config.c_cflag | PARENB : config.c_cflag & ~PARENB);

        //configure stop bit
        config.c_cflag = (twoStopBits ? config.c_cflag | CSTOPB : config.c_cflag & ~CSTOPB);

        //configure baud
        cfsetspeed(&config, baud);

        //configure system call settings vmin and vtime
        config.c_cc[VMIN] = minimumBytes;
        config.c_cc[VTIME] = maximumTimeout;
        
        //other, assumed settings
        config.c_cflag &= ~CRTSCTS;                 // disable hardware flow control
        config.c_cflag |= CREAD | CLOCAL;
        config.c_lflag &= ~ICANON;                  // disable canonical mode (line by line reading)
        config.c_lflag &= ~(ECHO | ECHOE | ECHONL); // disable echoing of sent characters
        config.c_lflag &= ~ISIG;                    // disable signal characters
        config.c_iflag &= ~(IXON | IXOFF | IXANY);  // disable software flow control
        config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); //disable special handling of input
        config.c_oflag &= ~(OPOST | ONLCR); //disable special handling of output bytes

        if(tcsetattr(file, TCSANOW, &config))
        {
            RCLCPP_FATAL(rosNode->get_logger(), "tcsetattr() failed: %s", strerror(errno));
            initialized = false;
            return false;
        }

        initialized = true;
        return true;
    }


    void LinuxSerialTransceiver::send(const char *data, size_t numData) const
    {
        if(initialized)
        {
            write(file, data, numData);
        }
    }


    size_t LinuxSerialTransceiver::recv(char *data, size_t numData) const
    {
        memset(data, 0, numData);
        if(initialized)
        {
            return read(file, data, numData);
        }

        return 0;
    }


    void LinuxSerialTransceiver::deinit(void)
    {
        if(initialized)
        {
            close(file);
            initialized = false;
        }
    }
}

#endif
