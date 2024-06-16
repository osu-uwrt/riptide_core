#include "riptide_gyro/serial_library.hpp"

namespace uwrt_gyro
{
    char *memstr(const char *haystack, size_t numHaystack, const char *needle, size_t numNeedle)
    {
        char *search = (char*) haystack;
        for(size_t i = 0; i <= numHaystack - numNeedle; i++)
        {
            if(memcmp(search, needle, numNeedle) == 0)
            {
                return search;
            }
            search = &search[1];
        }

        return nullptr;
    }


    size_t extractFieldFromBuffer(const char *src, size_t srcLen, SerialFrame frame, SerialFieldId field, char *dst, size_t dstLen)
    {
        auto it = frame.begin();
        size_t nextUnusedCharacter = 0; //for dst

        while(it != frame.end() && nextUnusedCharacter < dstLen)
        {
            //find next location of the field in the frame
            it = std::find(it, frame.end(), field);
            if(it == frame.end())
            {
                break;
            }

            int idx = it - frame.begin();
            if(idx < srcLen)
            {
                dst[nextUnusedCharacter] = src[idx];
            }

            nextUnusedCharacter++;
            it++;
        }

        return nextUnusedCharacter;
    }

    void insertFieldToBuffer(char *dst, size_t dstLen, SerialFrame frame, SerialFieldId field, const char *src, size_t srcLen)
    {
        auto it = frame.begin();
        size_t nextUnusedCharacter = 0; //for src

        while(it != frame.end() && nextUnusedCharacter < dstLen)
        {
            //find next location of the field in the frame
            it = std::find(it, frame.end(), field);
            if(it == frame.end())
            {
                break;
            }

            int idx = it - frame.begin();

            //we know where to put the next character from src; now put it there
            if(idx < dstLen)
            {
                dst[idx] = src[nextUnusedCharacter];
                nextUnusedCharacter++;
            }

            it++;
        }
    }

    SerialData serialDataFromString(const char* str, size_t numData)
    {
        SerialData data;
        strcpy(data.data, str);
        data.numData = numData;
        return data;
    }


    SerialFrame normalizeSerialFrame(const SerialFrame& frame)
    {
        auto syncIt = std::find(frame.begin(), frame.end(), FIELD_SYNC);

        //start with frame from sync field to end
        SerialFrame normalizedFrame(syncIt, frame.end());

        //add frame begin to sync field
        normalizedFrame.insert(normalizedFrame.end(),frame.begin(), syncIt);

        return normalizedFrame;
    }


    SerialFramesMap normalizeSerialFramesMap(const SerialFramesMap& map)
    {
        SerialFramesMap normalizedFrameMap;
        
        //compute normalized frames and add them to map
        for(auto pair : map)
        {
            SerialFrame normalizedFrame = normalizeSerialFrame(pair.second);
            normalizedFrameMap.insert({ pair.first, normalizedFrame });
        }

        return normalizedFrameMap;
    }
}


