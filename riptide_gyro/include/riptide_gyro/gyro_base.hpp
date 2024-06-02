#pragma once

#if __linux__
#define USE_LINUX
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
#endif


//
// library typedefs
//

#define FIELD_SYNC INT_MAX
#define FIELD_FRAME INT_MAX - 1
typedef int SerialFrameId;
typedef int SerialFieldId;

//describes the fields held by a serial frame. Each frame represents 8 bits.
typedef vector<SerialFieldId> SerialFrame;
typedef map<SerialFrameId, SerialFrame> SerialFramesMap;

struct SerialData
{
    size_t numData;
    char data[MAX_DATA_BYTES];
};

struct SerialDataStamped
{
    Time timestamp;
    SerialData data;
};

typedef map<SerialFieldId, SerialDataStamped> SerialValuesMap;


//
// other types
//

class SerialLibraryException
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
