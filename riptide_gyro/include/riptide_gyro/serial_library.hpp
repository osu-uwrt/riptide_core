#pragma once

#if __linux__
#include "riptide_gyro/gyro_base.hpp"
#include <termios.h>
#include <fcntl.h>
#else
#include "gyro_base.hpp"
#endif

namespace uwrt_gyro
{
    class SerialTransceiver
    {
        public:
        virtual SerialLibErrorCode init(void) = 0;
        virtual void send(const char *data, size_t numData) const = 0;
        virtual size_t recv(char *data, size_t numData) const = 0;
        virtual void deinit(void) = 0;
    };


    #if defined(BUILD_LINUX)

    class LinuxSerialTransceiver : public SerialTransceiver
    {
        public:
        LinuxSerialTransceiver() = default;
        LinuxSerialTransceiver(
            const slstring& fileName,
            int baud,
            int minimumBytes,
            int maximumTimeout,
            int mode = O_RDWR,
            int bitsPerByte = CS8,
            bool twoStopBits = false,
            bool parityBit = false);

        SerialLibErrorCode init(void) override;
        void send(const char *data, size_t numData) const override;
        size_t recv(char *data, size_t numData) const override;
        void deinit(void) override;

        private:
        slstring fileName;
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

    #elif defined(BUILD_ARDUINO)

    class ArduinoSerialTransceiver : public SerialTransceiver
    {
        public:
        ArduinoSerialTransceiver(int baud)
         : baud(baud) { }

        SerialLibErrorCode init(void) override
        {
            Serial.begin(baud);
            Serial.setTimeout(1);
        }
        void send(const char *data, size_t numData) const override
        {
            Serial.write(data, numData);
        }
        size_t recv(char *data, size_t numData) const override
        {
            Serial.readBytes(data, numData);
        }
        void deinit(void) override { }

        private:
        int baud;
    };

    #endif

    char *memstr(const char *haystack, size_t numHaystack, const char *needle, size_t numNeedle);
    size_t extractFieldFromBuffer(const char *src, size_t srcLen, SerialFrame frame, SerialFieldId field, char *dst, size_t dstLen);
    void insertFieldToBuffer(char *dst, size_t dstLen, SerialFrame frame, SerialFieldId field, const char *src, size_t srcLen);
    SerialData serialDataFromString(const char *str, size_t numData);
    
    // "normalized" in this case means that the frame starts with a sync, makes processing easier
    // SerialFrame normalizeSerialFrame(const SerialFrame& frame);
    // SerialFramesMap normalizeSerialFramesMap(const SerialFramesMap& map);

    // packs c string into primitive type. 0 is most significant
    template<typename T>
    T convertFromCString(const char *str, size_t strLen)
    {
        T val = 0;

        if(strLen <= 0)
        {
            return 0;
        }

        size_t tSz = sizeof(T) / sizeof(*str);

        //shift the smaller number of bytes
        size_t placesToShift = (tSz < strLen ? tSz : strLen);

        val |= str[0];
        for(size_t i = 1; i < placesToShift; i++)
        {
            val = val << sizeof(*str) * 8;
            val |= str[i] & 0xFF;
        }

        return val;
    }
    
    template<typename T>
    size_t convertToCString(T val, char *str, size_t strLen)
    {
        size_t valLen = sizeof(val);
        size_t numData = 0;
        for(int i = valLen / sizeof(*str) - 1; i >= 0; i--)
        {
            char newC = (char) val & 0xFF;
            if(i < strLen)
            {
                str[i] = newC;
                numData++;
            }

            val = val >> sizeof(*str) * 8;
        }

        return numData;
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
        slmutex lock;
        T resource;
    };


    static bool defaultCheckFunc(const char*, const SerialFrame&)
    {
        return true;
    }


    class SerialProcessor
    {
        public:
        #if defined(USE_LINUX)
        typedef std::shared_ptr<SerialProcessor> SharedPtr;
        #endif

        SerialProcessor() = default;
        SerialProcessor(SerialTransceiver& transceiver, SerialFramesMap frames, SerialFrameId defaultFrame, const char syncValue[], size_t syncValueLen, CheckFunc checker = &defaultCheckFunc);
        ~SerialProcessor();
        SerialLibErrorCode update(const Time& now);
        bool hasDataForField(SerialFieldId field);
        SerialDataStamped getField(SerialFieldId field);
        void setField(SerialFieldId field, SerialData data, const Time& now);
        SerialLibErrorCode send(SerialFrameId frameId);

        private:
        // regular member vars
        SerialTransceiver& transceiver;
        char 
            msgBuffer[PROCESSOR_BUFFER_SIZE],
            transmissionBuffer[PROCESSOR_BUFFER_SIZE];
        
        size_t msgBufferCursorPos;
        char syncValue[MAX_DATA_BYTES];
        size_t syncValueLen;

        const SerialFramesMap frameMap;
        const SerialFrameId defaultFrame;
        const CheckFunc checker;
        
        // "thread-safe" resources 
        ProtectedResource<SerialValuesMap*> valueMap;
    };
}
