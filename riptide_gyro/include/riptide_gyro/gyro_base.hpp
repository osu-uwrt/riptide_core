#pragma once

#if __linux__ && !defined(FORCE_ARDUINO)
#define USE_LINUX
#endif

#if defined(FORCE_ARDUINO)
#define USE_ARDUINO
#endif

#include <limits.h>

//
// settings
//
#define MAX_DATA_BYTES 4 
#define PROCESSOR_BUFFER_SIZE 256

//
// platform-dependent includes and typedefs
//
#if defined(USE_LINUX)
    #include <string>
    #include <rclcpp/rclcpp.hpp>

    #define TESTING_ENABLED

    typedef rclcpp::Time Time;
    typedef std::string string;
    typedef std::mutex mutex;
    
    template<typename K, typename V>
    using map = std::map<K, V>;

    template<typename T>
    using list = std::list<T>;

    template<typename T>
    using vector = std::vector<T>;
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
#endif


//
// exception
//
class SerialLibraryException
#if defined(USE_LINUX)
: public std::exception
#endif
{
    public:
    SerialLibraryException(const string& error)
     : error(error) { }
    
    std::string what()
    {
        return error;
    }

    private:
    string error;
};

#define THROW_SERIAL_LIB_EXCEPTION_WTIH_LINE(errmsg, line) throw SerialLibraryException(__FILE__ "@" #line ": " errmsg)
#define THROW_SERIAL_LIB_EXCEPTION(errmsg) THROW_SERIAL_LIB_EXCEPTION_WTIH_LINE(errmsg, __LINE__)
#define SERIAL_LIB_ASSERT(cond, msg) \
    do \
    { \
        if(!(cond)) \
        { \
            THROW_SERIAL_LIB_EXCEPTION(#cond ": " msg); \
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

struct SerialData
{
    size_t numData;
    char data[MAX_DATA_BYTES];

    void operator=(const SerialData& rhs)
    {
        if(rhs.numData >= MAX_DATA_BYTES)
        {
            THROW_SERIAL_LIB_EXCEPTION("SerialData being assigned must have less than " + std::to_string(numData) + " data");
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
