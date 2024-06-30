#pragma once

//
// arduino_library.hpp: A miniture std library to make 
// the serial lib capable of running on microcontrollers 
// like Arduino
//

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#define ARDUSTRING_MAX_LEN 24
#define ARDUVECTOR_MAX_LEN 8

#if __linux__
#include <unistd.h>
#define SLEEP(usec) usleep(usec)
#define BUILD_LINUX
#elif defined(ARDUINO)
#include <Arduino.h>
#define SLEEP(usec) delay(usec / 1000)
#define BUILD_ARDUINO
#endif

namespace arduino_lib
{
    #define NUM_ARGS(_type, ...) (sizeof((_type []){__VA_ARGS__}) / sizeof( _type ))

    static size_t countInString(char *str, char c)
    {
        char *next = strchr(str, c);
        size_t i = 0;
        while(next)
        {
            i++;
            next = strchr(next + 1, c);
        }

        return i;
    }

    class string
    {
        public:
        string()
         : cursor(0)
        {
            strcpy(impl, "");
        }

        string(const string& s)
         : cursor(s.length())
        {
            strcpy(impl, s.c_str());
        }

        string(const char *src)
         : cursor(strlen(src))
        {
            strcpy(impl, src);
        }

        size_t length() const
        {
            return cursor;
        }

        void append(const string& other)
        {
            strcat(impl, other.c_str());
            cursor += other.length();
        }

        string operator+(const string& other)
        {
            string c(*this);
            c.append(other);
            return c;
        }

        string operator+(const char* other)
        {
            string c(*this);
            c.append(other);
            return c;
        }

        bool operator==(const string& other) const
        {
            return strcmp(impl, other.c_str()) == 0;
        }

        bool operator==(const char *other) const
        {
            return strcmp(impl, other) == 0;
        }

        bool operator!=(const string& other) const 
        {
            return !(*this == other);
        }

        bool operator!=(const char *other) const
        {
            return !(*this == other);
        }

        const char *c_str() const
        {
            return (const char*) impl;
        }

        private:
        size_t cursor;
        char impl[ARDUSTRING_MAX_LEN];
    };

    static string to_string(size_t i)
    {
        return "";
    }

    inline string operator+(const string& a, const string& b)
    {
        string c = a;
        c.append(b);
        return c;
    }

    class exception
    {
        public:
        exception(const string& msg)
         : msg(msg) { }
        
        string what() const 
        {
            return msg;
        }
        
        private:
        const string msg;
    };

    inline void alFatalError(const string& msg)
    {
        #if defined(BUILD_LINUX)
            throw exception(msg);
        #else
            while(true)
            {
                Serial.println(msg.c_str());
                SLEEP(1000);
            }
        #endif
    }

    class mutex
    {
        public:
        mutex()
        : impl(false) { }

        bool try_lock()
        {
            if(!impl)
            {
                impl = true;
                return true;
            }

            return false;
        }

        void lock()
        {
            while(!try_lock())
            {
                SLEEP(1000); //sleep 1 ms
            }
        }

        void unlock()
        {
            impl = false;
        }

        private:
        bool impl;
    };


    template<typename Container, typename T>
    class Iterator
    {
        public:
        Iterator(const Container *c, size_t i)
         : c(c),
           i(i) { }
        
        Iterator(const Iterator<Container, T>& other)
         : c(other.c),
           i(other.i) { }
        
        size_t idx() const
        {
            return i;
        }
        
        T operator*()
        {
            return c->at(i);
        }

        T* operator->()
        {
            return c->ptrto(i);
        }

        bool operator==(const Iterator<Container, T>& other) const
        {
            return c == other.c && i == other.i;
        }

        bool operator!=(const Iterator<Container, T>& other) const
        {
            return !(*this == other);
        }

        int operator-(const Iterator<Container, T>& other) const
        {
            return i - other.i;
        }

        Iterator<Container, T> operator-(int x)
        {
            Iterator<Container, T> other(*this);
            other.i -= x;
            return other;
        }

        Iterator<Container, T> operator+(int x)
        {
            Iterator<Container, T> other(*this);
            other.i += x;
            return other;
        }

        Iterator<Container, T> operator++(int)
        {
            i += 1;
            return *this;
        }
        
        private:
        const Container *c;
        size_t i;
    };


    #define ARDUINOLIB_VECTOR(type, ...) \
        arduino_lib::vector<type>(NUM_ARGS(type, __VA_ARGS__), __VA_ARGS__)

    template<typename T>
    class vector
    {
        public:
        typedef arduino_lib::Iterator<arduino_lib::vector<T>, T> Iterator;

        vector() 
         : impl(new T[ARDUVECTOR_MAX_LEN]),
           sz(0) { }

        vector(const vector& v)
         : impl(new T[ARDUVECTOR_MAX_LEN])
        {
            sz = 0;
            append(v);
        }

        vector(const T *init, size_t numInit)
         : impl(new T[ARDUVECTOR_MAX_LEN]),
           sz(numInit)
        {
            memcpy(impl, init, numInit * sizeof(T));
        }

        vector(size_t numInit, ...)
         : impl(new T[ARDUVECTOR_MAX_LEN]),
           sz(numInit)
        {
            va_list args;
            va_start(args, numInit);
            
            for(int i = 0; i < numInit; i++)
            {
                impl[i] = va_arg(args, T);
            }

            va_end(args);
        }

        ~vector()
        {
            delete[] impl;
        }

        size_t size() const
        {
            return sz;
        }

        Iterator begin() const 
        {
            return Iterator(this, 0);
        }

        Iterator end() const 
        {
            return Iterator(this, sz);
        }

        T *ptrto(size_t idx) const
        {
            if(idx >= sz)
            {
                alFatalError(string("Index out of bounds: ") + to_string(idx));
            }

            return &impl[idx];
        }

        T at(size_t idx) const
        {
            return *ptrto(idx);
        }

        void append(const vector& other)
        {
            for(int i = 0; i < other.size(); i++)
            {
                impl[sz] = other[i];
                sz++;

                if(sz >= ARDUSTRING_MAX_LEN)
                {
                    break;
                }
            }
        }

        void push_back(const T& t)
        {
            if(sz < ARDUSTRING_MAX_LEN)
            {
                impl[sz] = t;
                sz++;
            }
        }

        vector operator+(const vector& other)
        {
            vector v(*this);
            v.append(other);
            return v;
        }

        bool operator==(const vector& other) const 
        {
            if(size() != other.size())
            {
                return false;
            }

            for(int i = 0; i < size(); i++)
            {
                if(at(i) != other[i])
                {
                    return false;
                }
            }

            return true;
        }

        bool operator!=(const vector& other) const
        {
            return !(*this == other);
        }

        T operator[](size_t i) const
        {
            return at(i);
        }

        private:
        size_t sz;
        T *impl;
    };

    //definitino macro is needed to avoid comma splicing in map definition
    #define ARDUINOLIB_PAIR_TYPE(keyType, valueType) arduino_lib::pair<keyType, valueType>

    template<typename K, typename V>
    class pair
    {
        public:
        pair() { };
        pair(K first, V second)
         : first(first),
           second(second) { }

        K first;
        V second;
    };


    #define ARDUINOLIB_MAP(keyType, valueType, ...) \
        arduino_lib::map<keyType, valueType>( \
            arduino_lib::vector<arduino_lib::pair<keyType, valueType>>( \
                sizeof((arduino_lib::pair<keyType, valueType>[]) {__VA_ARGS__}) / sizeof(arduino_lib::pair<keyType, valueType>), \
                __VA_ARGS__ \
            ) \
        ) 

    template<typename K, typename V>
    class map
    {
        public:
        typedef arduino_lib::pair<K, V> PairT;
        typedef arduino_lib::vector<PairT> VectorT;
        typedef arduino_lib::Iterator<VectorT, PairT> Iterator; //TODO custom iterator type, pointers increment by a byte, we need more than that

        map()
         : impl(vector<pair<K, V>>()) { }

        map(const vector<pair<K, V>>& pairs)
         : impl(pairs) { }
        
        size_t size() const
        {
            return impl.size();
        }

        Iterator begin() const
        {
            return Iterator(&impl, 0);
        }

        Iterator end() const
        {
            return Iterator(&impl, size());
        }

        Iterator find(const K& key) const
        {
            for(Iterator it = begin(); it != end(); it++)
            {
                if(it->first == key)
                {
                    return it;
                }
            }

            return end();
        }

        V *ptrto(const K& key) const 
        {
            Iterator it = find(key);
            if(it == end())
            {
                alFatalError("Key not found!");
            }

            return &(it->second);
        }

        V at(const K& key) const
        {
            return *ptrto(key);
        }

        V operator[](const K& key)
        {
            return at(key);
        }

        void insert(const pair<K, V> p)
        {
            impl.push_back(p);
        }

        private:
        vector<pair<K, V>> impl;
    };
}
