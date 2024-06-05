#include "riptide_gyro/serial_library.hpp"

namespace uwrt_gyro
{
    size_t extractFieldFromBuffer(char *msgStart, SerialFrame frame, SerialFieldId field, char *dst)
    {
        auto it = frame.begin();
        size_t nextUnusedCharacter = 0;

        while(it != frame.end())
        {
            it = std::find(it, frame.end(), field);
            if(it == frame.end())
            {
                break;
            }

            int idx = it - frame.begin();
            dst[nextUnusedCharacter] = msgStart[idx];

            nextUnusedCharacter++;
            it++;
        }

        return nextUnusedCharacter;
    }
}


