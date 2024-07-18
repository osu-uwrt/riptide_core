#include <rclcpp/rclcpp.hpp>
#include <riptide_msgs2/msg/int32_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <riptide_msgs2/msg/gyro_status.hpp>
#include "riptide_gyro/serial_library.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

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


    struct NodeParameters {
        std::string frame;
        
        double
            p00,
            p10,
            p01,
            p20,
            p11;
        
        double variance;
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
        : rclcpp::Node("riptide_gyro"),
          nodeParamsResource(new NodeParameters()),
          statusResource(new riptide_msgs2::msg::GyroStatus())
        {
            RCLCPP_INFO(get_logger(), "Starting gyro driver");

            //declare parameters
            declare_parameter<std::string>("gyro_port", DEFAULT_PORT);
            declare_parameter<std::string>("gyro_frame", DEFAULT_FRAME);
            declare_parameter<double>("p00", 0);
            declare_parameter<double>("p10", 0);
            declare_parameter<double>("p01", 0);
            declare_parameter<double>("p20", 0);
            declare_parameter<double>("p11", 0);
            declare_parameter<double>("variance", 0);
            add_on_set_parameters_callback(std::bind(&GyroDriver::setParamsCb, this, _1));
            reloadParams();
            

            //start timer
            timer = create_wall_timer(
                20ms,
                std::bind(&GyroDriver::timerCb, this));
            
            //create publishers
            rawDataPub = create_publisher<riptide_msgs2::msg::Int32Stamped>("gyro/raw", rclcpp::SensorDataQoS());
            statusPub = create_publisher<riptide_msgs2::msg::GyroStatus>("gyro/status", 10);
            twistPub = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("gyro/twist", rclcpp::SensorDataQoS());

            //initialize serial library
            try
            {
                std::string port = get_parameter("gyro_port").as_string();
                RCLCPP_INFO(get_logger(), "Gyro port: %s", port.c_str());
                transceiver = std::make_shared<LinuxSerialTransceiver>(port, GYRO_BAUD, 1, 0, O_RDONLY);
                processor = std::make_shared<SerialProcessor>(*transceiver, GYRO_FRAME_MAP, UwrtGyroFrame::TEMP_HIGH_FRAME, GYRO_SYNC, sizeof(GYRO_SYNC), &gyroChecker);
                processor->setNewMsgCallback(std::bind(&GyroDriver::gyroFrameReceived, this));
                procThread = std::make_unique<std::thread>(std::bind(&GyroDriver::threadFunc, this));
                RCLCPP_INFO(get_logger(), "Gyro driver started.");
            } catch(SerialLibraryException& ex)
            {
                RCLCPP_FATAL(get_logger(), "Caught SerialLibraryException during execution! what(): %s", ex.what().c_str());
                exit(1);
            }   
        }

        ~GyroDriver()
        {
            delete nodeParamsResource.lockResource();
            delete statusResource.lockResource();
        }

        private:

        void reloadParams()
        {
            NodeParameters *params = nodeParamsResource.lockResource();
            params->frame = get_parameter("gyro_frame").as_string();
            params->p00 = get_parameter("p00").as_double();
            params->p10 = get_parameter("p10").as_double();
            params->p01 = get_parameter("p01").as_double();
            params->p20 = get_parameter("p20").as_double();
            params->p11 = get_parameter("p11").as_double();
            params->variance = get_parameter("variance").as_double();
            nodeParamsResource.unlockResource();
        }

        rcl_interfaces::msg::SetParametersResult setParamsCb(const std::vector<rclcpp::Parameter> &)
        {
            reloadParams();

            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        }

        //this timer routine publishes diagnostics and everything else that isnt rate
        void timerCb()
        {
            std::string frame = nodeParamsResource.lockResource()->frame;
            nodeParamsResource.unlockResource();

            riptide_msgs2::msg::GyroStatus *statMsg = statusResource.lockResource();
            statMsg->header.frame_id = frame;
            statMsg->header.stamp = get_clock()->now();

            //pack temperature into status
            if(processor->hasDataForField(UwrtGyroField::TEMP_HIGH) && processor->hasDataForField(UwrtGyroField::TEMP_LOW))
            {
                SerialDataStamped data = processor->getField(UwrtGyroField::TEMP_HIGH);
                statMsg->temperature = convertFromCString<uint8_t>(data.data.data, data.data.numData);
                statMsg->temperature = statMsg->temperature << 8;
                data = processor->getField(UwrtGyroField::TEMP_LOW);
                statMsg->temperature |= convertFromCString<uint8_t>(data.data.data, data.data.numData);
            }

            //publish status
            statusPub->publish(*statMsg);
            statusResource.unlockResource();

            //diagnostics
            unsigned short failedOfLastTen = processor->failedOfLastTenMessages();
            if(failedOfLastTen > 5)
            {
                RCLCPP_WARN(get_logger(), "Unusually high message drop rate (%d / last 10 messages) detected.", failedOfLastTen);
            }
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

            NodeParameters *params = nodeParamsResource.lockResource();

            riptide_msgs2::msg::Int32Stamped rawMsg;
            rawMsg.header.frame_id = params->frame;
            rawMsg.header.stamp = data.timestamp;
            rawMsg.data = signedRawReading;
            rawDataPub->publish(rawMsg);

            //calculate and publish twist
            geometry_msgs::msg::TwistWithCovarianceStamped twistMsg;
            twistMsg.header = rawMsg.header;
            twistMsg.twist.covariance[35] = params->variance;

            //get temperature value from status
            int gyroTemperature = statusResource.lockResource()->temperature;
            statusResource.unlockResource();

            double 
                normalizedRawReading = (signedRawReading - 1192.0) / 1.517e6,
                normalizedGyroTemperature = (gyroTemperature - 12140.0) / 558.6;

            //fit = p00 + p10*x + p01*y + p20*x^2 + p11*xy, where x is rate and y is temp
            twistMsg.twist.twist.angular.z = 
                params->p00 +
                params->p10 * normalizedRawReading + 
                params->p01 * normalizedGyroTemperature + 
                params->p20 * normalizedRawReading * normalizedRawReading +
                params->p11 * normalizedRawReading * normalizedGyroTemperature;
            
            nodeParamsResource.unlockResource();

            twistPub->publish(twistMsg);
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

        //params
        ProtectedResource<NodeParameters*> nodeParamsResource;        

        //status
        ProtectedResource<riptide_msgs2::msg::GyroStatus*> statusResource;

        //tools
        std::shared_ptr<LinuxSerialTransceiver> transceiver;
        SerialProcessor::SharedPtr processor;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<riptide_msgs2::msg::Int32Stamped>::SharedPtr rawDataPub;
        rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twistPub;
        rclcpp::Publisher<riptide_msgs2::msg::GyroStatus>::SharedPtr statusPub;
        std::unique_ptr<std::thread> procThread;
    };
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    std::shared_ptr<uwrt_gyro::GyroDriver> driver = std::make_shared<uwrt_gyro::GyroDriver>();
    rclcpp::spin(driver);
    rclcpp::shutdown();
}
