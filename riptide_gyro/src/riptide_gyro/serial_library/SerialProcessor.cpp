#include "riptide_gyro/serial_library.hpp"

namespace uwrt_gyro
{
    SerialProcessor::SerialProcessor(SerialTransceiver& transceiver, SerialFramesMap frames, SerialFrameId defaultFrame, const char syncValue[], size_t syncValueLen, CheckFunc checker)
     : transceiver(transceiver),
       msgBufferCursorPos(0),
       syncValueLen(syncValueLen),
       checker(checker),
       frameMap(frames),
       defaultFrame(defaultFrame),
       valueMap(new SerialValuesMap())
    {
        SERIAL_LIB_ASSERT(transceiver.init(), "Transceiver initialization failed!");

        memcpy(this->syncValue, syncValue, sizeof(syncValue));
        
        //check that the frames include a sync
        //TODO: must check all individual frames for a sync, not the frame ids
        for(int i = 0; i < frames.size(); i++)
        {
            SerialFrame frame = frames.at(i);

            SERIAL_LIB_ASSERT(
                find(frame, FIELD_SYNC) != frame.size(), 
                "Frame was provided with no sync field");
        }

        //add sync value
        SerialData syncData = serialDataFromString(syncValue, syncValueLen);
        SerialDataStamped syncDataStamped;
        syncDataStamped.data = syncData;
        SerialValuesMap *values = valueMap.lockResource();
        values->insert( {FIELD_SYNC, syncDataStamped} );
        valueMap.unlockResource();

        //do assertions
        bool
            frameIdLocsMatch = false,
            syncLocsMatch = false;
        
        SERIAL_LIB_ASSERT(frames.size() > 0, "Must have at least one frame");
        SERIAL_LIB_ASSERT(frames.find(defaultFrame) != frames.end(), "Default frame must be contained within frames");

        size_t syncFieldLoc = find<SerialFieldId>(frames.at(0), FIELD_SYNC);
        SERIAL_LIB_ASSERT(syncFieldLoc != frames.at(0).size(), "Frame 0 does not contain a sync!");

        size_t frameFieldLoc = find<SerialFieldId>(frames.at(0), FIELD_FRAME);
        bool containsFrameField = frameFieldLoc != frames.at(0).size();
        SERIAL_LIB_ASSERT(containsFrameField || frames.size() == 1 , "Field 0 does not contain a frame field, but multiple frames are used!");

        for(size_t i = 0; i < frames.size(); i++)
        {
            size_t iSyncLoc = find<SerialFieldId>(frames.at(i), FIELD_SYNC);
            size_t iFrameLoc = find<SerialFieldId>(frames.at(i), FIELD_FRAME);

            SERIAL_LIB_ASSERT(iSyncLoc == syncFieldLoc, "Sync fields not aligned!");
            SERIAL_LIB_ASSERT(iFrameLoc == frameFieldLoc, "Frame fields not aligned!");
            SERIAL_LIB_ASSERT(find<SerialFieldId>(frames.at(i), FIELD_FRAME, iFrameLoc + 1) == frames.at(i).size(), "Large frame fields are not supported yet.");

            //check that the sync is continuous
            int syncFrameLen = 1;
            while(iSyncLoc < frames.at(i).size())
            {
                size_t nextSyncFieldLoc = find<SerialFieldId>(frames.at(i), FIELD_SYNC, iSyncLoc + 1);
                if(nextSyncFieldLoc != frames.at(i).size())
                {
                    SERIAL_LIB_ASSERT(nextSyncFieldLoc - iSyncLoc == 1, "Sync frame is not continuous!");
                    syncFrameLen++;
                }

                iSyncLoc = nextSyncFieldLoc;
            }

            SERIAL_LIB_ASSERT(syncFrameLen == syncValueLen, "Sync field length is not equal to the sync value length!");
        }
    }


    SerialProcessor::~SerialProcessor()
    {
        transceiver.deinit();
    }


    SerialLibErrorCode SerialProcessor::update(const Time& now)
    {
        //TODO can probably rewrite method and use SERIAL_LIB_ASSERT
        size_t recvd = transceiver.recv(transmissionBuffer, PROCESSOR_BUFFER_SIZE);
        if(recvd == 0)
        {
            return SERIAL_LIB_NO_ERROR;
        }

        // append new contents to buffer
        size_t bytesToCopy = recvd;
        if(msgBufferCursorPos + recvd > PROCESSOR_BUFFER_SIZE)
        {
            bytesToCopy = -msgBufferCursorPos + PROCESSOR_BUFFER_SIZE;
        }

        memcpy(&msgBuffer[msgBufferCursorPos], transmissionBuffer, bytesToCopy);
        msgBufferCursorPos += bytesToCopy;

        char *syncLocation = nullptr;

        do
        {
            // find sync values. having two means we have one potential messages
            syncLocation = memstr(msgBuffer, msgBufferCursorPos, syncValue, syncValueLen);
            if(!syncLocation)
            {
                return SERIAL_LIB_NO_ERROR;
            }

            size_t msgSz = msgBufferCursorPos + (size_t) msgBuffer - (size_t) syncLocation;

            // can process message here. first need to figure out the frame to use.
            // if there was only one frame provided, this is easy. otherwise, need to look for indication in the message
            SerialFrame frameToUse = frameMap.at(defaultFrame);
            size_t
                frameMapSz = frameMap.size(),
                frameSz = frameToUse.size();

            //determine the message string based on the sync location. then process it
            int
                msgStartOffsetFromSync = find<SerialFieldId>(frameToUse, FIELD_SYNC),
                syncOffsetFromBuffer = syncLocation - msgBuffer;

            char 
                *msgStart = syncLocation,
                *msgEnd = msgBuffer + msgBufferCursorPos;

            bool msgPassesUserTest = checker(msgStart, frameToUse);
            if(msgStartOffsetFromSync <= syncOffsetFromBuffer && msgPassesUserTest) //todo also add msg checker function to this condition
            {
                //message good and parsable
                msgStart = syncLocation - msgStartOffsetFromSync;
                msgEnd = msgStart + frameSz;

                //check that we can parse for a frame id
                if(msgBufferCursorPos < frameSz)
                {
                    //we dont have enough information to parse the default frame for a frame id.
                    break;
                }

                //parse for a frame id
                if(frameMapSz > 1)
                {
                    char frameIdBuf[MAX_DATA_BYTES] = {0};
                    size_t bytes = extractFieldFromBuffer(msgStart, msgSz, frameToUse, FIELD_FRAME, frameIdBuf, MAX_DATA_BYTES);
                    if(bytes == 0)
                    {
                        return slReportNonFatalError("No frame id found in message");
                    }

                    SerialFrameId frameId = convertFromCString<SerialFrameId>(frameIdBuf, bytes);

                    if(frameMap.find(frameId) == frameMap.end())
                    {
                        return slReportNonFatalError("Cannot parse message because frame " + to_string(frameId) + " does not exist");
                    }

                    frameToUse = frameMap.at(frameId);
                    
                    //check if the frame is parsable
                    frameSz = frameToUse.size();
                    if(msgBufferCursorPos < frameSz)
                    {
                        //we dont have enough information to parse this frame
                        continue;
                    }
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

                //iterate through known fields and update their values from message
                SerialValuesMap *values = valueMap.lockResource();
                const int fieldBufSz = frameMap.size();
                char fieldBuf[fieldBufSz];
                for(auto it = values->begin(); it != values->end(); it++)
                {
                    memset(fieldBuf, 0, frameMap.size());
                    SerialFieldId field = it->first;
                    size_t extracted = extractFieldFromBuffer(msgStart, frameSz, frameToUse, field, fieldBuf, fieldBufSz);
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
            } else
            {
                //message bad. dont remove like normal, just delete through the sync character
                msgEnd = syncLocation + 1;
            }

            //remove message from the buffer
            memmove(msgBuffer, msgEnd, PROCESSOR_BUFFER_SIZE - msgBufferCursorPos);
            size_t amountRemoved = msgEnd - msgStart;
            msgBufferCursorPos -= amountRemoved;
        } while(syncLocation);

        return SERIAL_LIB_NO_ERROR;
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
    
    
    SerialLibErrorCode SerialProcessor::send(SerialFrameId frameId)
    {
        if(frameMap.find(frameId) == frameMap.end())
        {
            slReportNonFatalError("Cannot send message with unknown frame id " + to_string(frameId));
        }

        SerialFrame frame = frameMap.at(frameId);

        //loop through minimal set of frames and pack each frame into the transmission buffer
        slvector<SerialFieldId> frameSet = getMinimalSet<SerialFieldId>(frame);
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
                    return slReportNonFatalError("Cannot send serial frame because it is missing field " + to_string(*fieldIt));
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
