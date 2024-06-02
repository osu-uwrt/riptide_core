#include "riptide_gyro/serial_library.hpp"

namespace uwrt_gyro
{
    SerialProcessor::SerialProcessor(SerialTransceiver& transceiver, SerialFramesMap frames, SerialFrameId defaultFrame, char *syncValue)
     : transceiver(transceiver),
       frameMap(frames),
       msgBufferCursorPos(0),
       syncValue(syncValue),
       defaultFrame(defaultFrame),
       valueMap(new SerialValuesMap())
    {
        transceiver.init();

        //check that the frames include a sync
        if(frames.find(FIELD_SYNC) == frames.end())
        {
            THROW_SERIAL_LIB_EXCEPTION("No sync field provided in the serial frames map.");
        }
    }


    SerialProcessor::~SerialProcessor()
    {
        transceiver.deinit();
    }


    void SerialProcessor::update(const Time& now)
    {
        size_t recvd = transceiver.recv(transmissionBuffer, PROCESSOR_BUFFER_SIZE);
        if(recvd == 0)
        {
            return;
        }

        // append new contents to buffer
        size_t bytesToCopy = recvd;
        if(msgBufferCursorPos + recvd > PROCESSOR_BUFFER_SIZE)
        {
            bytesToCopy = -msgBufferCursorPos + PROCESSOR_BUFFER_SIZE;
        }

        memcpy(&msgBuffer[msgBufferCursorPos], transmissionBuffer, bytesToCopy);
        msgBufferCursorPos += bytesToCopy;

        // find sync values. having two means we have one potential messages
        char
            *firstSyncLocation = strstr(msgBuffer, syncValue),
            *secondSyncLocation = strstr(msgBuffer, syncValue);

        // if one of the sync values is NULL, dont process message
        if(!(firstSyncLocation || secondSyncLocation))
        {
            return;
        }

        // can process message here. first need to figure out the frame to use.
        // if there was only one frame provided, this is easy. otherwise, need to look for indication in the message
        SerialFrame frameToUse = frameMap.at(defaultFrame);
        if(frameMap.size() > 1)
        {
            char frameIdBuf[frameMap.size()] = {0};
            size_t bytes = extractFieldFromBuffer(firstSyncLocation, frameToUse, FIELD_FRAME, frameIdBuf);
            if(bytes == 0)
            {
                throw SerialLibraryException("No frame id found in message");
            }

            SerialFrameId frameId = convertCString<SerialFrameId>(frameIdBuf);
            frameToUse = frameMap.at(frameId);
        }

        //iterate through frame and find all unknown fields
        for(auto it = frameToUse.begin(); it != frameToUse.end(); it++)
        {
            SerialValuesMap *values = valueMap.lockResource();

            if(values->find(*it) == values->end())
            {
                values->at(*it) = SerialDataStamped();
            }

            valueMap.unlockResource();
        }

        //iterate through known fields and update their values
        SerialValuesMap *values = valueMap.lockResource();
        char fieldBuf[frameMap.size()];
        for(auto it = values->begin(); it != values->end(); it++)
        {
            memset(fieldBuf, 0, frameMap.size());
            SerialFieldId field = it->first;
            size_t extracted = extractFieldFromBuffer(firstSyncLocation, frameToUse, field, fieldBuf);
            if(extracted > 0)
            {
                SerialDataStamped serialData;
                serialData.timestamp = now;
                memcpy(serialData.data.data, fieldBuf, extracted);

                it->second = serialData;
            }
        }
    }
    
    
    bool SerialProcessor::hasDataForField(SerialFieldId field)
    {
        SerialValuesMap *values = valueMap.lockResource();
        bool hasData = values->find(field) != values->end();
        valueMap.unlockResource();
        return hasData;
    }
    
    
    SerialDataStamped SerialProcessor::getField(SerialFieldId field)
    {
        SerialValuesMap *values = valueMap.lockResource();
        SerialDataStamped data = values->at(field);
        valueMap.unlockResource();
        return data;
    }
    
    
    void SerialProcessor::setField(SerialFieldId field, SerialData data, const Time& now)
    {
        SerialValuesMap *values = valueMap.lockResource();
        values->at(field).timestamp = now;
        values->at(field).data = data;
        valueMap.unlockResource();
    }
    
    
    void SerialProcessor::send(SerialFramesMap map)
    {

    }
}
