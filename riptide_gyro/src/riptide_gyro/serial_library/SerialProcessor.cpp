#include "riptide_gyro/serial_library.hpp"

namespace uwrt_gyro
{
    SerialProcessor::SerialProcessor(SerialTransceiver& transceiver, SerialFramesMap frames, SerialFrameId defaultFrame, const char syncValue[], size_t syncValueLen, CheckFunc checker)
     : transceiver(transceiver),
       msgBufferCursorPos(0),
       syncValueLen(syncValueLen),
       frameMap(frames),
       defaultFrame(defaultFrame),
       checker(checker),
       newMsgFunc(nullptr),
       valueMap(new SerialValuesMap())
    {
        SERIAL_LIB_ASSERT(transceiver.init(), "Transceiver initialization failed!");

        memcpy(this->syncValue, syncValue, syncValueLen);
        
        //check that the frames include a sync
        //TODO: must check all individual frames for a sync, not the frame ids
        for(size_t i = 0; i < frames.size(); i++)
        {
            SerialFrame frame = frames.at(i);
            if(std::find(frame.begin(), frame.end(), FIELD_SYNC) == frame.end())
            {
                THROW_NON_FATAL_SERIAL_LIB_EXCEPTION("No sync field provided in frame " + std::to_string(i) + " of the map.");
            }
        }

        //add sync value
        SerialData syncData = serialDataFromString(syncValue, syncValueLen);
        SerialDataStamped syncDataStamped;
        syncDataStamped.data = syncData;
        SerialValuesMap *values = valueMap.lockResource();
        values->insert( {FIELD_SYNC, syncDataStamped} );
        valueMap.unlockResource();
        
        SERIAL_LIB_ASSERT(frames.size() > 0, "Must have at least one frame");
        SERIAL_LIB_ASSERT(frames.find(defaultFrame) != frames.end(), "Default frame must be contained within frames");

        auto syncFieldIt = std::find(frames.at(0).begin(), frames.at(0).end(), FIELD_SYNC);
        SERIAL_LIB_ASSERT(syncFieldIt != frames.at(0).end(), "Frame 0 does not contain a sync!");
        size_t syncFieldLoc = syncFieldIt - frames.at(0).begin();

        auto frameFieldIt = std::find(frames.at(0).begin(), frames.at(0).end(), FIELD_FRAME);
        // bool containsFrameField = frameFieldIt != frames.at(0).end();
        // SERIAL_LIB_ASSERT(containsFrameField || frames.size() == 1 , "Field 0 does not contain a frame field, but multiple frames are used!");
        size_t frameFieldLoc = frameFieldIt - frames.at(0).begin();

        for(size_t i = 0; i < frames.size(); i++)
        {
            auto iSyncIt = std::find(frames.at(i).begin(), frames.at(i).end(), FIELD_SYNC);
            auto iFrameIt = std::find(frames.at(i).begin(), frames.at(i).end(), FIELD_FRAME);

            SERIAL_LIB_ASSERT((size_t) (iSyncIt - frames.at(i).begin()) == syncFieldLoc, "Sync fields not aligned!");
            SERIAL_LIB_ASSERT((size_t) (iFrameIt - frames.at(i).begin()) == frameFieldLoc, "Frame fields not aligned!");
            SERIAL_LIB_ASSERT(std::find(iFrameIt + 1, frames.at(i).end(), FIELD_FRAME) == frames.at(i).end(), "Large frame fields are not supported yet.");

            //check that the sync is continuous
            size_t syncFrameLen = 1;
            while(iSyncIt != frames.at(i).end())
            {
                auto nextSyncFieldIt = std::find(iSyncIt + 1, frames.at(i).end(), FIELD_SYNC);
                if(nextSyncFieldIt != frames.at(i).end())
                {
                    SERIAL_LIB_ASSERT(nextSyncFieldIt - iSyncIt == 1, "Sync frame is not continuous!");
                    syncFrameLen++;
                }

                iSyncIt = nextSyncFieldIt;
            }

            SERIAL_LIB_ASSERT(syncFrameLen == syncValueLen, "Sync field length is not equal to the sync value length!");
        }
    }


    SerialProcessor::~SerialProcessor()
    {
        transceiver.deinit();
    }


    void SerialProcessor::setNewMsgCallback(NewMsgFunc func)
    {
        this->newMsgFunc = func;
    }


    void SerialProcessor::update(const Time& now)
    {
        //TODO can probably rewrite method and use SERIAL_LIB_ASSERT
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

        char *syncLocation = nullptr;
        do
        {
            // find sync values. having two means we have one potential messages
            syncLocation = memstr(msgBuffer, msgBufferCursorPos, syncValue, syncValueLen);
            if(!syncLocation)
            {
                return;
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
                msgStartOffsetFromSync = std::find(frameToUse.begin(), frameToUse.end(), FIELD_SYNC) - frameToUse.begin(),
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
                    if(bytes > 0)
                    {
                        SerialFrameId frameId = convertFromCString<SerialFrameId>(frameIdBuf, bytes);

                        if(frameMap.find(frameId) == frameMap.end())
                        {
                            THROW_NON_FATAL_SERIAL_LIB_EXCEPTION("Cannot parse message because frame " + std::to_string(frameId) + " does not exist");
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
                for(auto it = values->begin(); it != values->end(); it++)
                {
                    memset(fieldBuf, 0, sizeof(fieldBuf));
                    SerialFieldId field = it->first;
                    size_t extracted = extractFieldFromBuffer(msgStart, frameSz, frameToUse, field, fieldBuf, sizeof(fieldBuf));
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
                
                //call new message function
                if(newMsgFunc)
                {
                    newMsgFunc();
                }
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
            THROW_NON_FATAL_SERIAL_LIB_EXCEPTION("Cannot send message with unknown frame id " + std::to_string(frameId));
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
                    valueMap.unlockResource();
                    THROW_NON_FATAL_SERIAL_LIB_EXCEPTION("Cannot send serial frame because it is missing field " + std::to_string(*fieldIt));
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
