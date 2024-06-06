#pragma once

#include "riptide_gyro/gyro_base.hpp"

#if defined(USE_LINUX)
#include <termios.h>
#include <fcntl.h>
#endif

namespace uwrt_gyro
{
    class SerialTransceiver
    {
        public:
        virtual bool init(void) = 0;
        virtual void send(const std::string& msg) const = 0;
        virtual size_t recv(char *data, size_t numData) const = 0;
        virtual void deinit(void) = 0;
    };


    #if defined(USE_LINUX)

    class LinuxSerialTransceiver : public SerialTransceiver
    {
        public:
        LinuxSerialTransceiver() = default;
        LinuxSerialTransceiver(
            const rclcpp::Node::SharedPtr node,
            const std::string& fileName,
            int baud,
            int minimumBytes,
            int maximumTimeout,
            int mode = O_RDWR,
            int bitsPerByte = CS8,
            bool twoStopBits = false,
            bool parityBit = false);

        bool init(void) override;
        void send(const std::string& msg) const override;
        size_t recv(char *data, size_t numData) const override;
        void deinit(void) override;

        private:
        rclcpp::Node::SharedPtr rosNode;
        std::string fileName;
        int
            file,
            baud,
            mode,
            bitsPerByte,
            minimumBytes,
            maximumTimeout;
        
        bool 
            initialized,
            twoStopBits,
            parityBit;
    };

    #endif

    size_t extractFieldFromBuffer(char *src, size_t srcLen, SerialFrame frame, SerialFieldId field, char *dst, size_t dstLen);
    void insertFieldToBuffer(char *dst, size_t dstLen, SerialFrame frame, SerialFieldId field, const char *src, size_t srcLen);
    SerialData serialDataFromString(const char *str, size_t numData);

    template<typename T>
    T convertCString(const char *str)
    {
        T val;
        for(size_t i = 0; i < sizeof(T) / sizeof(*str); i++)
        {
            val |= str[i];
            val = val << sizeof(*str);
        }

        return val;
    }
    
    template<typename T>
    class ProtectedResource
    {
        public:
        ProtectedResource(T resource)
        : resource(resource) { }

        T& lockResource()
        {
            lock.lock();
            return resource;
        }

        void unlockResource()
        {
            lock.unlock();
        }

        private:
        mutex lock;
        T resource;
    };


    class SerialProcessor
    {
        public:
        #if defined(USE_LINUX)
        typedef std::shared_ptr<SerialProcessor> SharedPtr;
        #endif

        SerialProcessor() = default;
        SerialProcessor(SerialTransceiver& transceiver, SerialFramesMap frames, SerialFrameId defaultFrame, const char *syncValue);
        ~SerialProcessor();
        void update(const Time& now);
        bool hasDataForField(SerialFieldId field);
        SerialDataStamped getField(SerialFieldId field);
        void setField(SerialFieldId field, SerialData data, const Time& now);
        void send(SerialFrame frame);

        private:
        // regular member vars
        SerialTransceiver& transceiver;
        char 
            msgBuffer[PROCESSOR_BUFFER_SIZE],
            transmissionBuffer[PROCESSOR_BUFFER_SIZE];
        
        size_t msgBufferCursorPos;
        const char *syncValue;
        const SerialFramesMap frameMap;
        const SerialFrameId defaultFrame;
        
        // "thread-safe" resources 
        ProtectedResource<SerialValuesMap*> valueMap;
    };
}
