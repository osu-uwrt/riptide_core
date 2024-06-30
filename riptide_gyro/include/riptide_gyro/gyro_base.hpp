#pragma once

#if __linux__
#define BUILD_LINUX
#elif defined(ARDUINO)
#define BUILD_ARDUINO
#endif

#if (defined(BUILD_LINUX) && !defined(FORCE_ARDUINO)) || defined(ENABLE_TESTING)
#define USE_LINUX
#endif

#if defined(FORCE_ARDUINO) || defined(ARDUINO)
#undef USE_LINUX
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
    typedef std::string slstring;
    typedef std::mutex slmutex;
    
    template<typename K, typename V>
    using slmap = std::map<K, V>;

    template<typename T>
    using sllist = std::list<T>;

    template<typename T>
    using slvector = std::vector<T>;

    template<typename T>
    inline slstring to_string(const T& arg)
    {
        return std::to_string(arg);
    }

#elif defined(USE_ARDUINO)
    #include "arduino_library.hpp"

    typedef long Time;
    typedef arduino_lib::string slstring;
    typedef arduino_lib::mutex slmutex;
    
    template<typename K, typename V>
    using slmap = arduino_lib::map<K, V>;

    template<typename T>
    using sllist = arduino_lib::vector<T>;

    template<typename T>
    using slvector = arduino_lib::vector<T>;

    template<typename T>
    inline slstring to_string(const T& arg)
    {
        return arduino_lib::to_string(arg);
    }
#endif

template<typename T>
inline size_t find(const slvector<T>& v, const T& t, size_t start = 0)
{
    for(auto it = v.begin() + start; it != v.end(); it++)
    {
        if(*it == t)
        {
            return it - v.begin();
        }
    }

    return v.size();
}

template<typename T>
slvector<T> getMinimalSet(const slvector<T>& vec)
{
    slvector<T> ret;
    for(size_t i = 0; i < vec.size(); i++)
    {
        T toAdd = vec.at(i);
        if(find(ret, toAdd) == ret.size())
        {
            ret.push_back(toAdd);
        }
    }

    return ret;
}


//
// exception
//
// class SerialLibraryException
// {
//     public:
//     SerialLibraryException(const slstring& error)
//      : error(error)
//     { }

//     slstring what()
//     {
//         return error;
//     }

//     private:
//     slstring error;
// };

// class NonFatalSerialLibraryException : public SerialLibraryException
// {
//     public:
//     NonFatalSerialLibraryException(const slstring& error)
//      : SerialLibraryException(error) { }
// };

// class FatalSerialLibraryException : public SerialLibraryException
// {
//     public:
//     FatalSerialLibraryException(const slstring& error)
//      : SerialLibraryException(error) { }
// };

//
// error handling
//
typedef int SerialLibErrorCode;

static slstring serialliberror;

// #if defined(USE_LINUX)
// #define THROW_FATAL_SERIAL_LIB_EXCEPTION(errmsg) throw FatalSerialLibraryException(slstring(__FILE__) + slstring("@") + to_string(__LINE__) + slstring(": ") + errmsg);
// #define THROW_NON_FATAL_SERIAL_LIB_EXCEPTION(errmsg) throw NonFatalSerialLibraryException(slstring(__FILE__) + slstring("@") + to_string(__LINE__) + slstring(": ") + errmsg);
#define SERIAL_LIB_NO_ERROR 0
#define SERIAL_LIB_FATAL_ERROR 1
#define SERIAL_LIB_NONFATAL_ERROR 2

static SerialLibErrorCode slReportFatalError(const slstring& msg)
{
    serialliberror = slstring(msg);
    return SERIAL_LIB_FATAL_ERROR;
}

static SerialLibErrorCode slReportNonFatalError(const slstring& msg)
{
    serialliberror = slstring(msg);
    return SERIAL_LIB_NONFATAL_ERROR;
}

// #elif defined(USE_ARDUINO)
// void fatalException(const slstring& errmsg)
// {
//     while(true)
//     {
//         Serial.println(msg);
//         for(int i = 0; i < 4; i++)
//         {
//             digitalWrite(LED_BUILTIN, HIGH);
//             delay(100);
//             digitalWrite(LED_BUILTIN, LOW);
//             delay(100);
//         }

//         delay(2000);
//     }
// }

// #define THROW_FATAL_SERIAL_LIB_EXCEPTION(errmsg) fatalException(slstring(__FILE__) + slstring("@") + to_string(__LINE__) + slstring(": ") + errmsg)
// #define THROW_NON_FATAL_SERIAL_LIB_EXCEPTION(errmsg) \
// do \
// { \
//     Serial.println(slstring(__FILE__) + slstring("@") + to_string(__LINE__) + slstring(": ") + errmsg); \
//     return; \
// } while(0)

// #endif

static void slAssert(bool cond, const char *msg)
{
    if(!(cond))
    { 
        #if defined(USE_LINUX)
            exit(1);
        #elif defined(USE_ARDUINO)
            while(true);
        #endif
    }
}

#define SERIAL_LIB_ASSERT(cond, msg) slAssert(cond, #cond " : " msg)

//
// library typedefs
//

#define FIELD_SYNC INT_MAX
#define FIELD_FRAME INT_MAX - 1
typedef uint8_t SerialFrameId;
typedef int SerialFieldId;

//describes the fields held by a serial frame. Each frame represents 8 bits.
typedef slvector<SerialFieldId> SerialFrame;
typedef slmap<SerialFrameId, SerialFrame> SerialFramesMap;

typedef bool(*CheckFunc)(const char *msgStart, const SerialFrame& frame);

struct SerialData
{
    size_t numData;
    char data[MAX_DATA_BYTES];

    void operator=(const SerialData& rhs)
    {
        if(rhs.numData >= MAX_DATA_BYTES)
        {
            slReportFatalError(slstring("SerialData being assigned must have less than ") + to_string(MAX_DATA_BYTES) + slstring(" data, but has ") + to_string(rhs.numData));
            return;
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

typedef slmap<SerialFieldId, SerialDataStamped> SerialValuesMap;
