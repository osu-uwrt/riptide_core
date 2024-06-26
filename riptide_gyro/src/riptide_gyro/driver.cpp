#include <rclcpp/rclcpp.hpp>
#include "riptide_gyro/serial_library.hpp"

using namespace std::chrono_literals;

//
// Rigid settings 
//
#define GYRO_BAUD 921600
#define DEFAULT_PORT "/dev/ttyUSB0"

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
        CHECKSUM
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
        DIAG_SIGNAL_LOW_FRAME   = 0x07
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

    const SerialFramesMap GYRO_FRAME_MAP = {
        defineGyroFrameMapping(UwrtGyroFrame::TEMP_HIGH_FRAME, UwrtGyroField::TEMP_HIGH),
        defineGyroFrameMapping(UwrtGyroFrame::TEMP_LOW_FRAME, UwrtGyroField::TEMP_LOW),
        defineGyroFrameMapping(UwrtGyroFrame::VSUPPLY_HIGH_FRAME, UwrtGyroField::VSUPPLY_HIGH),
        defineGyroFrameMapping(UwrtGyroFrame::VSUPPLY_LOW_FRAME, UwrtGyroField::VSUPPLY_LOW),
        defineGyroFrameMapping(UwrtGyroFrame::SLD_CURRENT_HIGH_FRAME, UwrtGyroField::SLD_CURRENT_HIGH),
        defineGyroFrameMapping(UwrtGyroFrame::SLD_CURRENT_LOW_FRAME, UwrtGyroField::SLD_CURRENT_LOW),
        defineGyroFrameMapping(UwrtGyroFrame::DIAG_SIGNAL_HIGH_FRAME, UwrtGyroField::DIAG_SIGNAL_HIGH),
        defineGyroFrameMapping(UwrtGyroFrame::DIAG_SIGNAL_LOW_FRAME, UwrtGyroField::DIAG_SIGNAL_LOW)
    };

    const char GYRO_SYNC[] = { (char) 0xdd };

    class GyroDriver : public rclcpp::Node
    {
        public:
        GyroDriver()
        : rclcpp::Node("riptide_gyro")
        {
            //declare parameters
            declare_parameter<std::string>("gyro_port", DEFAULT_PORT);
        }

        void init()
        {
            RCLCPP_INFO(get_logger(), "Starting gyro driver");

            //start timer
            timer = create_wall_timer(
                2ms,
                std::bind(&GyroDriver::timerCb, this));

            //initialize serial library

            std::string port = get_parameter("gyro_port").as_string();
            RCLCPP_INFO(get_logger(), "Gyro port: %s", port.c_str());
            transceiver = std::make_shared<LinuxSerialTransceiver>(shared_from_this(), "/dev/ttyUSB0", GYRO_BAUD, 1, 0, O_RDONLY);
            processor = std::make_shared<SerialProcessor>(*transceiver, GYRO_FRAME_MAP, UwrtGyroFrame::TEMP_HIGH_FRAME, GYRO_SYNC, sizeof(GYRO_SYNC));

            RCLCPP_INFO(get_logger(), "Gyro driver started.");
        }

        private:
        void timerCb()
        {
            processor->update(get_clock()->now());
            if(!processor->hasDataForField(UwrtGyroField::VRATE))
            {
                RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gyro reading");
            }

            SerialDataStamped data = processor->getField(UwrtGyroField::VRATE);
            uint32_t 
                rawReading = 0,
                buffer = 0;
            
            //low
            rawReading |= data.data.data[0];

            //high
            buffer = (uint32_t) data.data.data[1];
            buffer = buffer << 16;
            rawReading |= buffer;

            //middle
            buffer = (uint32_t) data.data.data[2];
            buffer = buffer << 8;
            rawReading |= buffer;

            RCLCPP_INFO(get_logger(), "Reading: %d", rawReading);
        }

        std::shared_ptr<LinuxSerialTransceiver> transceiver;
        SerialProcessor::SharedPtr processor;
        rclcpp::TimerBase::SharedPtr timer;
    };
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<uwrt_gyro::GyroDriver> driver = std::make_shared<uwrt_gyro::GyroDriver>();

    try
    {
        driver->init();
    } catch(SerialLibraryException& ex)
    {
        RCLCPP_FATAL(driver->get_logger(), "Caught SerialLibraryException during execution! what(): %s", ex.what().c_str());
    }

    while(rclcpp::ok())
    {
        try
        {
            rclcpp::spin(driver);
        }   catch(SerialLibraryException& ex)
        {
            RCLCPP_FATAL(driver->get_logger(), "Caught SerialLibraryException during execution! what(): %s", ex.what().c_str());
        }
    }

    rclcpp::shutdown();
}
