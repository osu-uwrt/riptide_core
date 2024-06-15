#include "riptide_gyro/serial_library.hpp"

namespace uwrt_gyro
{
    SerialProcessor::SerialProcessor(SerialTransceiver& transceiver, SerialFramesMap frames, SerialFrameId defaultFrame, const char syncValue[])
     : transceiver(transceiver),
       msgBufferCursorPos(0),
       defaultFrame(defaultFrame),
       valueMap(new SerialValuesMap())
    {
        transceiver.init();

        memcpy(this->syncValue, syncValue, sizeof(syncValue));
        syncValueLen = sizeof(syncValue);
        
        //check that the frames include a sync
        //TODO: must check all individual frames for a sync, not the frame ids
        for(int i = 0; i < frames.size(); i++)
        {
            SerialFrame frame = frames.at(i);
            if(std::find(frame.begin(), frame.end(), FIELD_SYNC) == frame.end())
            {
                THROW_SERIAL_LIB_EXCEPTION("No sync field provided in frame " + std::to_string(i) + " of the map.");
            }
        }

        //add sync value
        SerialData syncData = serialDataFromString(syncValue, strlen(syncValue));
        SerialDataStamped syncDataStamped;
        syncDataStamped.data = syncData;
        SerialValuesMap *values = valueMap.lockResource();
        values->insert( {FIELD_SYNC, syncDataStamped} );
        valueMap.unlockResource();

        //compute normalized frames and add them to map
        for(auto pair : frames)
        {
            SerialFrame normalizedFrame = normalizeSerialFrame(pair.second);
            normalizedFrameMap.insert({ pair.first, normalizedFrame });
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
        char *firstSyncLocation = memstr(msgBuffer, msgBufferCursorPos, syncValue, syncValueLen);
        size_t currentDataLen = msgBuffer + msgBufferCursorPos - firstSyncLocation;
        char *secondSyncLocation = memstr(firstSyncLocation + 1, currentDataLen, syncValue, syncValueLen);

        // if one of the sync values is NULL, dont process message
        if(!firstSyncLocation || !secondSyncLocation)
        {
            return;
        }

        size_t msgSz = (size_t) secondSyncLocation - (size_t) firstSyncLocation;

        // can process message here. first need to figure out the frame to use.
        // if there was only one frame provided, this is easy. otherwise, need to look for indication in the message
        SerialFrame frameToUse = normalizedFrameMap.at(defaultFrame);
        size_t frameMapSz = normalizedFrameMap.size();
        if(frameMapSz > 1)
        {
            char frameIdBuf[MAX_DATA_BYTES] = {0};
            size_t bytes = extractFieldFromBuffer(firstSyncLocation, msgSz, frameToUse, FIELD_FRAME, frameIdBuf, frameMapSz);
            if(bytes == 0)
            {
                throw SerialLibraryException("No frame id found in message");
            }

            SerialFrameId frameId = convertFromCString<SerialFrameId>(frameIdBuf, bytes);

            if(normalizedFrameMap.find(frameId) == normalizedFrameMap.end())
            {
                THROW_SERIAL_LIB_EXCEPTION("Cannot parse message because frame " + std::to_string(frameId) + " does not exist");
            }

            frameToUse = normalizedFrameMap.at(frameId);
        }

        //iterate through frame and find all unknown fields
        for(auto it = frameToUse.begin(); it != frameToUse.end(); it++)
        {
            SerialValuesMap *values = valueMap.lockResource();

            if(values->find(*it) == values->end())
            {
                values->insert({ *it, SerialDataStamped() });
            }

            valueMap.unlockResource();
        }

        //iterate through known fields and update their values
        SerialValuesMap *values = valueMap.lockResource();
        const int fieldBufSz = normalizedFrameMap.size();
        char fieldBuf[fieldBufSz];
        for(auto it = values->begin(); it != values->end(); it++)
        {
            memset(fieldBuf, 0, normalizedFrameMap.size());
            SerialFieldId field = it->first;
            size_t extracted = extractFieldFromBuffer(firstSyncLocation, msgSz, frameToUse, field, fieldBuf, fieldBufSz);
            if(extracted > 0)
            {
                SerialDataStamped serialData;
                serialData.timestamp = now;
                memcpy(serialData.data.data, fieldBuf, extracted);
                serialData.data.numData = extracted;

                it->second = serialData;
            }
        }

        valueMap.unlockResource();

        //remove message from the buffer
        memmove(msgBuffer, secondSyncLocation, PROCESSOR_BUFFER_SIZE - msgBufferCursorPos);
        size_t amountRemoved = secondSyncLocation - msgBuffer;
        msgBufferCursorPos -= amountRemoved;
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
        SerialDataStamped data;
        if(values->find(field) != values->end())
        {
            data = values->at(field);
        }

        valueMap.unlockResource();
        return data;
    }
    
    
    void SerialProcessor::setField(SerialFieldId field, SerialData data, const Time& now)
    {
        SerialValuesMap *values = valueMap.lockResource();
        if(values->find(field) == values->end())
        {
            //no field currently set, so add one
            values->insert({ field, SerialDataStamped() });
        }

        SerialDataStamped stampedData;
        stampedData.data = data;
        stampedData.timestamp = now;
        values->at(field) = stampedData;
        valueMap.unlockResource();
    }
    
    
    void SerialProcessor::send(SerialFrame frame)
    {
        //loop through minimal set of frames and pack each frame into the transmission buffer
        std::set<SerialFieldId> frameSet(frame.begin(), frame.end());
        for(auto fieldIt = frameSet.begin(); fieldIt != frameSet.end(); fieldIt++)
        {
            SerialValuesMap *values = valueMap.lockResource();
            if(values->find(*fieldIt) == values->end() && *fieldIt != FIELD_SYNC)
            {
                THROW_SERIAL_LIB_EXCEPTION("Cannot send serial frame because it is missing field " + std::to_string(*fieldIt));
            }

            insertFieldToBuffer(
                transmissionBuffer, 
                sizeof(transmissionBuffer), 
                frame, 
                *fieldIt, 
                values->at(*fieldIt).data.data, 
                values->at(*fieldIt).data.numData);
            
            valueMap.unlockResource();
        }

        transceiver.send(transmissionBuffer, frame.size());
    }
}
