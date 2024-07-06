#include <rclcpp/rclcpp.hpp>
#include <riptide_msgs2/msg/int32_stamped.hpp>
#include <riptide_msgs2/msg/gyro_status.hpp>
#include "riptide_gyro/serial_library.hpp"

using namespace std::chrono_literals;

//
// Rigid settings 
//
#define GYRO_BAUD 921600
#define DEFAULT_PORT "/dev/ttyUSB0"
#define DEFAULT_FRAME "fog_link"

namespace uwrt_gyro {

    //
    // Field definitions
    //
    enum UwrtGyroField
    {
        VRATE,
        TEMP_HIGH,
        TEMP_LOW,
        VSUPPLY_HIGH,
        VSUPPLY_LOW,
        SLD_CURRENT_HIGH,
        SLD_CURRENT_LOW,
        DIAG_SIGNAL_HIGH,
        DIAG_SIGNAL_LOW,
        CHECKSUM,
        RESERVED
    };

    //
    // Frame definitions
    //

    enum UwrtGyroFrame
    {
        TEMP_HIGH_FRAME         = 0x00,
        TEMP_LOW_FRAME          = 0x01,
        VSUPPLY_HIGH_FRAME      = 0x02,
        VSUPPLY_LOW_FRAME       = 0x03,
        SLD_CURRENT_HIGH_FRAME  = 0x04,
        SLD_CURRENT_LOW_FRAME   = 0x05,
        DIAG_SIGNAL_HIGH_FRAME  = 0x06,
        DIAG_SIGNAL_LOW_FRAME   = 0x07,
        RESERVED_1_FRAME        = 0x08,
        RESERVED_2_FRAME        = 0x09,
        RESERVED_3_FRAME        = 0x0A,
        RESERVED_4_FRAME        = 0x0B,
        RESERVED_5_FRAME        = 0x0C,
        RESERVED_6_FRAME        = 0x0D,
        RESERVED_7_FRAME        = 0x0E,
        RESERVED_8_FRAME        = 0x0F
    };

    std::pair<SerialFrameId, SerialFrame> defineGyroFrameMapping(const UwrtGyroFrame& frame, const UwrtGyroField& specialField)
    {
        return {
            frame,
            SerialFrame({
                FIELD_SYNC,
                UwrtGyroField::VRATE,
                UwrtGyroField::VRATE,
                UwrtGyroField::VRATE,
                FIELD_FRAME,
                specialField,
                UwrtGyroField::CHECKSUM,
                UwrtGyroField::CHECKSUM
            })
        };
    }

    std::pair<SerialFrameId, SerialFrame> defineReservedFrameMapping(const UwrtGyroFrame& frame)
    {
        return {
            frame,
            SerialFrame({
                FIELD_SYNC,
                UwrtGyroField::RESERVED,
                UwrtGyroField::RESERVED,
                UwrtGyroField::RESERVED,
                FIELD_FRAME,
                UwrtGyroField::RESERVED,
                UwrtGyroField::CHECKSUM,
                UwrtGyroField::CHECKSUM
            })
        };
    }

    const SerialFramesMap GYRO_FRAME_MAP = {
        defineGyroFrameMapping(UwrtGyroFrame::TEMP_HIGH_FRAME, UwrtGyroField::TEMP_HIGH),
        defineGyroFrameMapping(UwrtGyroFrame::TEMP_LOW_FRAME, UwrtGyroField::TEMP_LOW),
        defineGyroFrameMapping(UwrtGyroFrame::VSUPPLY_HIGH_FRAME, UwrtGyroField::VSUPPLY_HIGH),
        defineGyroFrameMapping(UwrtGyroFrame::VSUPPLY_LOW_FRAME, UwrtGyroField::VSUPPLY_LOW),
        defineGyroFrameMapping(UwrtGyroFrame::SLD_CURRENT_HIGH_FRAME, UwrtGyroField::SLD_CURRENT_HIGH),
        defineGyroFrameMapping(UwrtGyroFrame::SLD_CURRENT_LOW_FRAME, UwrtGyroField::SLD_CURRENT_LOW),
        defineGyroFrameMapping(UwrtGyroFrame::DIAG_SIGNAL_HIGH_FRAME, UwrtGyroField::DIAG_SIGNAL_HIGH),
        defineGyroFrameMapping(UwrtGyroFrame::DIAG_SIGNAL_LOW_FRAME, UwrtGyroField::DIAG_SIGNAL_LOW),

        defineReservedFrameMapping(UwrtGyroFrame::RESERVED_1_FRAME),
        defineReservedFrameMapping(UwrtGyroFrame::RESERVED_2_FRAME),
        defineReservedFrameMapping(UwrtGyroFrame::RESERVED_3_FRAME),
        defineReservedFrameMapping(UwrtGyroFrame::RESERVED_4_FRAME),
        defineReservedFrameMapping(UwrtGyroFrame::RESERVED_5_FRAME),
        defineReservedFrameMapping(UwrtGyroFrame::RESERVED_6_FRAME),
        defineReservedFrameMapping(UwrtGyroFrame::RESERVED_7_FRAME),
        defineReservedFrameMapping(UwrtGyroFrame::RESERVED_8_FRAME)
    };


    static bool gyroChecker(const char *msgStart, const SerialFrame& frame)
    {
        uint16_t checksum = 0;

        //for fitzoptica gyro, checksum is the sum of the first 5 bytes
        for(int i = 1; i < 6; i++)
        {
            checksum += (unsigned char) msgStart[i];
        }

        //locate checksum in message
        char expectedChecksumBuf[2];
        size_t extracted = extractFieldFromBuffer(msgStart, frame.size(), frame, UwrtGyroField::CHECKSUM, expectedChecksumBuf, sizeof(expectedChecksumBuf));
        if(extracted != sizeof(expectedChecksumBuf))
        {
            return false;
        }

        uint16_t expectedChecksum = convertFromCString<uint16_t>(expectedChecksumBuf, extracted);
        return checksum == expectedChecksum;
    }


    const char GYRO_SYNC[] = { (char) 0xdd };

    class GyroDriver : public rclcpp::Node
    {
        public:
        GyroDriver()
        : rclcpp::Node("riptide_gyro")
        {
            RCLCPP_INFO(get_logger(), "Starting gyro driver");

            //declare parameters
            declare_parameter<std::string>("gyro_port", DEFAULT_PORT);
            declare_parameter<std::string>("gyro_frame", DEFAULT_FRAME);

            frame = get_parameter("gyro_frame").as_string();

            //start timer
            timer = create_wall_timer(
                2ms,
                std::bind(&GyroDriver::timerCb, this));
            
            //create publishers
            rawDataPub = create_publisher<riptide_msgs2::msg::Int32Stamped>("gyro/raw", rclcpp::SensorDataQoS());
            statusPub = create_publisher<riptide_msgs2::msg::GyroStatus>("gyro/status", 10);

            //initialize serial library
            std::string port = get_parameter("gyro_port").as_string();
            RCLCPP_INFO(get_logger(), "Gyro port: %s", port.c_str());
            transceiver = std::make_shared<LinuxSerialTransceiver>(port, GYRO_BAUD, 1, 0, O_RDONLY);
            processor = std::make_shared<SerialProcessor>(*transceiver, GYRO_FRAME_MAP, UwrtGyroFrame::TEMP_HIGH_FRAME, GYRO_SYNC, sizeof(GYRO_SYNC), &gyroChecker);
            processor->setNewMsgCallback(std::bind(&GyroDriver::gyroFrameReceived, this));
            procThread = std::make_unique<std::thread>(std::bind(&GyroDriver::threadFunc, this));
            RCLCPP_INFO(get_logger(), "Gyro driver started.");
        }

        private:
        void timerCb()
        {
            
        }

        void gyroFrameReceived()
        {
            if(!processor->hasDataForField(UwrtGyroField::VRATE))
            {
                RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gyro reading");
            }

            SerialDataStamped data = processor->getField(UwrtGyroField::VRATE);
            uint32_t 
                unsignedRawReading = 0,
                buffer = 0;
            
            //low
            unsignedRawReading |= data.data.data[0] & 0xFF;

            //high
            buffer = (uint32_t) data.data.data[1];
            buffer = buffer << 16;
            unsignedRawReading |= buffer & 0xFF0000;

            //middle
            buffer = (uint32_t) data.data.data[2];
            buffer = buffer << 8;
            unsignedRawReading |= buffer & 0x00FF00;

            int32_t signedRawReading = (unsignedRawReading > 0x800000 ? unsignedRawReading - 0xFFFFFF : unsignedRawReading);

            riptide_msgs2::msg::Int32Stamped rawMsg;
            rawMsg.header.frame_id = DEFAULT_FRAME;
            rawMsg.header.stamp = data.timestamp;
            rawMsg.data = signedRawReading;
            rawDataPub->publish(rawMsg);
        }

        void threadFunc()
        {
            while(rclcpp::ok())
            {
                try
                {
                    processor->update(get_clock()->now());
                } catch(NonFatalSerialLibraryException& ex)
                {
                    RCLCPP_WARN(get_logger(), "Caught serial lib exception: %s", ex.what().c_str());
                }
            }
        }

        std::string frame;
        std::shared_ptr<LinuxSerialTransceiver> transceiver;
        SerialProcessor::SharedPtr processor;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<riptide_msgs2::msg::Int32Stamped>::SharedPtr rawDataPub;
        rclcpp::Publisher<riptide_msgs2::msg::GyroStatus>::SharedPtr statusPub;
        std::unique_ptr<std::thread> procThread;
    };
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<uwrt_gyro::GyroDriver> driver = std::make_shared<uwrt_gyro::GyroDriver>();

    try
    {
        rclcpp::spin(driver);
    } catch(SerialLibraryException& ex)
    {
        RCLCPP_FATAL(driver->get_logger(), "Caught SerialLibraryException during execution! what(): %s", ex.what().c_str());
    }   

    rclcpp::shutdown();
}
