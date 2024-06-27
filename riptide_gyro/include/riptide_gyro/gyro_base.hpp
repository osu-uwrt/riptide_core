#pragma once

#if (__linux__ && !defined(FORCE_ARDUINO)) || defined(ENABLE_TESTING)
#define USE_LINUX
#endif

#if defined(FORCE_ARDUINO)
#define USE_ARDUINO
#endif

#include <limits.h>
#include <stdint.h>
#include <stdbool.h>

//
// settings
//
#define MAX_DATA_BYTES 8
#define PROCESSOR_BUFFER_SIZE 4096

//
// platform-dependent includes and typedefs
//

#if defined(USE_LINUX)
    #include <string>
    #include <rclcpp/rclcpp.hpp>

    typedef rclcpp::Time Time;
    typedef std::string string;
    typedef std::mutex mutex;
    
    template<typename K, typename V>
    using map = std::map<K, V>;

    template<typename T>
    using list = std::list<T>;

    template<typename T>
    using vector = std::vector<T>;

    template<typename T>
    inline string to_string(const T& arg)
    {
        return std::to_string(arg);
    }

#elif defined(USE_ARDUINO)
    #include "riptide_gyro/arduino_library.hpp"

    typedef long Time;
    typedef arduino_lib::string string;
    typedef arduino_lib::mutex mutex;
    
    template<typename K, typename V>
    using map = arduino_lib::map<K, V>;

    template<typename T>
    using list = arduino_lib::vector<T>;

    template<typename T>
    using vector = arduino_lib::vector<T>;

    template<typename T>
    inline string to_string(const T& arg)
    {
        return arduino_lib::to_string(arg);
    }
#endif


//
// exception
//
class SerialLibraryException
{
    public:
    SerialLibraryException(const string& error)
     : error(error)
    { }

    string what()
    {
        return error;
    }

    private:
    string error;
};

class NonFatalSerialLibraryException : public SerialLibraryException
{
    public:
    NonFatalSerialLibraryException(const string& error)
     : SerialLibraryException(error) { }
};

class FatalSerialLibraryException : public SerialLibraryException
{
    public:
    FatalSerialLibraryException(const string& error)
     : SerialLibraryException(error) { }
};

#define THROW_FATAL_SERIAL_LIB_EXCEPTION(errmsg) throw FatalSerialLibraryException(string(__FILE__) + string("@") + to_string(__LINE__) + string(": ") + errmsg);
#define THROW_NON_FATAL_SERIAL_LIB_EXCEPTION(errmsg) throw NonFatalSerialLibraryException(string(__FILE__) + string("@") + to_string(__LINE__) + string(": ") + errmsg);
#define SERIAL_LIB_ASSERT(cond, msg) \
    do \
    { \
        if(!(cond)) \
        { \
            THROW_FATAL_SERIAL_LIB_EXCEPTION(#cond ": " msg); \
        } \
    } while(0)


//
// library typedefs
//

#define FIELD_SYNC INT_MAX
#define FIELD_FRAME INT_MAX - 1
typedef uint8_t SerialFrameId;
typedef int SerialFieldId;

//describes the fields held by a serial frame. Each frame represents 8 bits.
typedef vector<SerialFieldId> SerialFrame;
typedef map<SerialFrameId, SerialFrame> SerialFramesMap;

typedef bool(*CheckFunc)(const char *msgStart, const SerialFrame& frame);

struct SerialData
{
    size_t numData;
    char data[MAX_DATA_BYTES];

    void operator=(const SerialData& rhs)
    {
        if(rhs.numData >= MAX_DATA_BYTES)
        {
            THROW_FATAL_SERIAL_LIB_EXCEPTION(string("SerialData being assigned must have less than ") + to_string(MAX_DATA_BYTES) + string(" data, but has ") + to_string(rhs.numData));
        }
        numData = rhs.numData;
        memcpy(data, rhs.data, numData);
    }
};

struct SerialDataStamped
{
    Time timestamp;
    SerialData data;

    void operator=(const SerialDataStamped& rhs)
    {
        timestamp = rhs.timestamp;
        data = rhs.data;
    }
};

typedef map<SerialFieldId, SerialDataStamped> SerialValuesMap;
