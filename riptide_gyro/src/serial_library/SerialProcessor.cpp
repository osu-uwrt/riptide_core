#include "riptide_gyro/serial_library.hpp"

namespace uwrt_gyro
{
    SerialProcessor::SerialProcessor(SerialTransceiver& transceiver, SerialFramesMap frames, SerialFrameId defaultFrame, const char syncValue[], size_t syncValueLen)
     : transceiver(transceiver),
       msgBufferCursorPos(0),
       syncValueLen(syncValueLen),
       frameMap(frames),
       defaultFrame(defaultFrame),
       valueMap(new SerialValuesMap())
    {
        transceiver.init();

        memcpy(this->syncValue, syncValue, sizeof(syncValue));
        
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

        char *secondSyncLocation = nullptr;

        do
        {
            // find sync values. having two means we have one potential messages
            char *firstSyncLocation = memstr(msgBuffer, msgBufferCursorPos, syncValue, syncValueLen);
            if(!firstSyncLocation)
            {
                return;
            }

            size_t currentDataLen = msgBuffer + msgBufferCursorPos - firstSyncLocation;
            secondSyncLocation = memstr(firstSyncLocation + 1, currentDataLen, syncValue, syncValueLen);

            // if one of the sync values is NULL, dont process message
            if(!secondSyncLocation)
            {
                return;
            }

            size_t msgSz = (size_t) secondSyncLocation - (size_t) firstSyncLocation;

            // can process message here. first need to figure out the frame to use.
            // if there was only one frame provided, this is easy. otherwise, need to look for indication in the message
            SerialFrame frameToUse = frameMap.at(defaultFrame);
            size_t frameMapSz = frameMap.size();
            if(frameMapSz > 1)
            {
                char frameIdBuf[MAX_DATA_BYTES] = {0};
                size_t bytes = extractFieldFromBuffer(firstSyncLocation, msgSz, frameToUse, FIELD_FRAME, frameIdBuf, MAX_DATA_BYTES);
                if(bytes == 0)
                {
                    throw SerialLibraryException("No frame id found in message");
                }

                SerialFrameId frameId = convertFromCString<SerialFrameId>(frameIdBuf, bytes);

                if(frameMap.find(frameId) == frameMap.end())
                {
                    THROW_SERIAL_LIB_EXCEPTION("Cannot parse message because frame " + std::to_string(frameId) + " does not exist");
                }

                frameToUse = frameMap.at(frameId);
            }

            //find beginning of message based on frame being used
            int 
                msgStartOffsetFromSyncLoc = std::find(frameToUse.begin(), frameToUse.end(), FIELD_SYNC) - frameToUse.begin(),
                msgSyncOffsetFromBufferStart = firstSyncLocation - msgBuffer;

            if(msgStartOffsetFromSyncLoc < msgSyncOffsetFromBufferStart)
            {
                //message is cut off by the start of the buffer. Move on
                continue;
            }

            char *msgStart = firstSyncLocation - msgStartOffsetFromSyncLoc;

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
            const int fieldBufSz = frameMap.size();
            char fieldBuf[fieldBufSz];
            for(auto it = values->begin(); it != values->end(); it++)
            {
                memset(fieldBuf, 0, frameMap.size());
                SerialFieldId field = it->first;
                size_t extracted = extractFieldFromBuffer(msgStart, msgSz, frameToUse, field, fieldBuf, fieldBufSz);
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
        } while(secondSyncLocation);
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
    
    
    void SerialProcessor::send(SerialFrameId frameId)
    {
        if(frameMap.find(frameId) == frameMap.end())
        {
            THROW_SERIAL_LIB_EXCEPTION("Cannot send message with unknown frame id " + std::to_string(frameId));
        }

        SerialFrame frame = frameMap.at(frameId);

        //loop through minimal set of frames and pack each frame into the transmission buffer
        std::set<SerialFieldId> frameSet(frame.begin(), frame.end());
        for(auto fieldIt = frameSet.begin(); fieldIt != frameSet.end(); fieldIt++)
        {
            SerialData dataToInsert;
            SerialValuesMap *values = valueMap.lockResource();
            if(values->find(*fieldIt) != values->end())
            {
                dataToInsert = values->at(*fieldIt).data;
            } else
            {
                //couldnt find the field included in the frame. Check if the frame is a builtin type
                if(*fieldIt == FIELD_SYNC)
                {
                    memcpy(dataToInsert.data, syncValue, syncValueLen);
                    dataToInsert.numData = syncValueLen;
                } else if(*fieldIt == FIELD_FRAME)
                {
                    dataToInsert.numData = convertToCString<SerialFrameId>(frameId, dataToInsert.data, MAX_DATA_BYTES);
                } else
                {
                    //if it is a custom type, throw exception because it is undefined
                    THROW_SERIAL_LIB_EXCEPTION("Cannot send serial frame because it is missing field " + std::to_string(*fieldIt));
                }
            }

            insertFieldToBuffer(
                transmissionBuffer, 
                sizeof(transmissionBuffer), 
                frame, 
                *fieldIt, 
                dataToInsert.data,
                dataToInsert.numData);
            
            valueMap.unlockResource();
        }

        transceiver.send(transmissionBuffer, frame.size());
    }
}