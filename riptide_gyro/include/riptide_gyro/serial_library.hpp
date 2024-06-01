#pragma once

#include <string>

namespace uwrt_gyro
{
    class SerialTransceiver
    {
        virtual void init(void) = 0;
        virtual void send(const std::string& msg) const = 0;
        virtual std::string recv(void) const = 0;
        virtual void deinit(void) = 0;
    };


    class LinuxSerialTransceiver : public SerialTransceiver
    {
        void init(void) override;
        void send(const std::string& msg) const override;
        std::string recv(void) const override;
        void deinit(void) override;
    };
}
