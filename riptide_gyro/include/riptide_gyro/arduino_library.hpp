#pragma once

#include <string.h>

#define ARDUSTRING_MAX_LEN 64

#if __linux__
#include <unistd.h>
#define SLEEP(usec) usleep(usec)
#else
#define SLEEP(usec) delay(usec / 1000)
#endif

namespace arduino_lib
{
    class string
    {
        public:
        string(const char *src)
         : cursor(strlen(src))
        {
            strcpy(impl, src);
        }

        size_t length() const
        {
            return cursor;
        }

        private:
        size_t cursor;
        char impl[ARDUSTRING_MAX_LEN];
    };

    string to_string(size_t i)
    {
        return "";
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

    class mutex
    {
        public:
        mutex()
        : impl(false) { }

        void lock()
        {
            while(!impl)
            {
                SLEEP(1000); //sleep 1 ms
            }

            impl = true;
        }

        void unlock()
        {
            impl = false;
        }

        private:
        bool impl;
    };

    template<typename T>
    class vector
    {
        public:
        typedef T* Iterator;

        vector(const T *init, size_t numInit)
         : impl(new T[numInit]),
           sz(numInit)
        {
            memcpy(impl, init, numInit * sizeof(T));
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
            return impl;
        }

        Iterator end() const 
        {
            return &impl[sz];
        }

        T at(size_t idx)
        {
            if(idx >= sz)
            {
                throw exception("Index out of bounds: " + to_string(idx));
            }

            return impl[sz];
        }

        private:
        size_t sz;
        T *impl;
    };

    template<typename K, typename V>
    class pair
    {
        public:
        pair(K first, V second)
         : first(first),
           second(second) { }

        K first;
        V second;
    };

    template<typename K, typename V>
    class map
    {
        public:
        typedef pair<K, V>* Iterator; //TODO custom iterator type, pointers increment by a byte, we need more than that

        map(const vector<pair<K, V>>& pairs)
         : impl(pairs) { }
        
        size_t size() const
        {
            return impl.size();
        }

        Iterator begin() const
        {
            return &impl.at(0);
        }

        Iterator end() const
        {
            return &impl.at(impl.size());
        }

        Iterator find(const K& key) const
        {
            for(Iterator it = begin(); it != end(); it++)
            {
                if(*it.first == key)
                {
                    return it;
                }
            }

            return end();
        }

        V at(const K& key)
        {
            Iterator it = find(key);
            if(it == end())
            {
                throw exception("at()");
            }

            return *it.second;
        }

        private:
        vector<pair<K, V> impl;
    };
}
