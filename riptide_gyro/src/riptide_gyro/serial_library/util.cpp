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
        size_t
            loc = 0,
            nextUnusedCharacter = 0; //for dst

        while(loc < frame.size() && nextUnusedCharacter < dstLen)
        {
            //find next location of the field in the frame
            loc = find<SerialFieldId>(frame, field, loc);
            if(loc >= frame.size())
            {
                break;
            }

            if(loc < srcLen)
            {
                dst[nextUnusedCharacter] = src[loc];
            }

            nextUnusedCharacter++;
            loc++;
        }

        return nextUnusedCharacter;
    }

    void insertFieldToBuffer(char *dst, size_t dstLen, SerialFrame frame, SerialFieldId field, const char *src, size_t srcLen)
    {
        size_t
            loc = 0,
            nextUnusedCharacter = 0; //for src

        while(loc < frame.size() && nextUnusedCharacter < dstLen)
        {
            //find next location of the field in the frame
            loc = find<SerialFieldId>(frame, field, loc);
            if(loc >= frame.size())
            {
                break;
            }

            //we know where to put the next character from src; now put it there
            if(loc < dstLen)
            {
                dst[loc] = src[nextUnusedCharacter];
                nextUnusedCharacter++;
            }

            loc++;
        }
    }

    SerialData serialDataFromString(const char* str, size_t numData)
    {
        SerialData data;
        strcpy(data.data, str);
        data.numData = numData;
        return data;
    }


    // SerialFrame normalizeSerialFrame(const SerialFrame& frame)
    // {
    //     size_t syncLoc = find<SerialFieldId>(frame, FIELD_SYNC);

    //     //start with frame from sync field to end
    //     SerialFrame normalizedFrame(syncLoc, frame.end());

    //     //add frame begin to sync field
    //     normalizedFrame.insert(normalizedFrame.end(),frame.begin(), syncIt);

    //     return normalizedFrame;
    // }


    // SerialFramesMap normalizeSerialFramesMap(const SerialFramesMap& map)
    // {
    //     SerialFramesMap normalizedFrameMap;
        
    //     //compute normalized frames and add them to map
    //     for(auto pair : map)
    //     {
    //         SerialFrame normalizedFrame = normalizeSerialFrame(pair.second);
    //         normalizedFrameMap.insert({ pair.first, normalizedFrame });
    //     }

    //     return normalizedFrameMap;
    // }
}


