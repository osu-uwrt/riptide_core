#include <fstream>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <riptide_msgs2/msg/int32_stamped.hpp>
#include <riptide_msgs2/action/tare_gyro.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <riptide_msgs2/msg/gyro_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "riptide_gyro/serial_library.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

//
// Rigid settings 
//
#define GYRO_BAUD 921600
#define DEFAULT_PORT "/dev/ttyUSB0"
#define DEFAULT_FRAME "fog_link"

#define TEMP_UPPER 70.0
#define VSUPPLY_UPPER 5.25
#define VSUPPLY_LOWER 4.75
#define SLDCURRENT_UPPER 0.15
#define SLDCURRENT_LOWER 0.07
#define DIAG_SIGNAL_LOWER 0.95
#define DIAG_SIGNAL_UPPER 1.05

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
                UwrtGyroField::VRATE,
                UwrtGyroField::VRATE,
                UwrtGyroField::VRATE,
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

        int
            rateFreq,
            statusFreq;
        
        double
            a,
            b,
            c,
            d,
            f,
            g,
            rateNormMean,
            rateNormStd,
            tempNormMean,
            tempNormStd;
        
        std::vector<double> tempLimits; //format: lower, upper
        
        double variance;
    };

    struct TareStatus {
        TareStatus()
         : offset(0),
           active(false),
           acquiredSamples(0),
           desiredSamples(0) { }

        double offset;
        bool active;
        int
            acquiredSamples,
            desiredSamples;
    };

    struct RateStatus {
        RateStatus()
         : accum(0),
           samples(0) { }

        double accum;
        int samples;
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
        using TareGyro = riptide_msgs2::action::TareGyro;
        using TareGyroGH = rclcpp_action::ServerGoalHandle<TareGyro>;

        GyroDriver()
        : rclcpp::Node("riptide_gyro"),
          tareFileName(ament_index_cpp::get_package_share_directory("riptide_gyro") + "/tare"),
          nodeParamsResource(new NodeParameters()),
          statusResource(new riptide_msgs2::msg::GyroStatus()),
          tareStatusResource(new TareStatus())
        {
            RCLCPP_INFO(get_logger(), "Starting gyro driver");

            //declare parameters
            declare_parameter<std::string>("gyro_port", DEFAULT_PORT);
            declare_parameter<std::string>("gyro_frame", DEFAULT_FRAME);
            declare_parameter<int>("rate_frequency", 50);
            declare_parameter<int>("status_frequency", 1);
            declare_parameter<double>("a", 0);
            declare_parameter<double>("b", 0);
            declare_parameter<double>("c", 0);
            declare_parameter<double>("d", 0);
            declare_parameter<double>("f", 0);
            declare_parameter<double>("g", 0);
            declare_parameter<double>("rate_norm_mean", 0);
            declare_parameter<double>("rate_norm_std", 0);
            declare_parameter<double>("temp_norm_mean", 0);
            declare_parameter<double>("temp_norm_std", 0);
            declare_parameter<double>("variance", 0);
            declare_parameter<std::vector<double>>("cal_temp_limits", std::vector<double>());
            add_on_set_parameters_callback(std::bind(&GyroDriver::setParamsCb, this, _1));
            reloadParams();

            //load tare offset
            if(std::filesystem::exists(tareFileName))
            {
                std::ifstream tareFile(tareFileName);
                std::string tareStr;
                tareFile >> tareStr;
                double tare = 0;

                try
                {
                    tare = std::stod(tareStr);
                    
                    TareStatus *status = tareStatusResource.lockResource();
                    status->offset = tare;
                    tareStatusResource.unlockResource();

                    RCLCPP_INFO(get_logger(), "Read tare from %s as %f", tareFileName.c_str(), tare);
                } catch(std::invalid_argument& ex)
                {
                    RCLCPP_ERROR(get_logger(), "Could not read tare from file %s: %s", tareFileName.c_str(), ex.what());
                }
            } else
            {
                RCLCPP_WARN(get_logger(), "Not loading tare because file %s does not exist.", tareFileName.c_str());
            }
            
            //start timer
            timer = create_wall_timer(
                20ms,
                std::bind(&GyroDriver::statusTimerCb, this));
            
            //create publishers
            rawDataPub = create_publisher<riptide_msgs2::msg::Int32Stamped>("gyro/raw", rclcpp::SensorDataQoS());
            statusPub = create_publisher<riptide_msgs2::msg::GyroStatus>("gyro/status", 10);
            twistPub = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("gyro/twist", rclcpp::SensorDataQoS());

            //create tare server
            tareServer = rclcpp_action::create_server<TareGyro>(
                this,
                "gyro/tare",
                std::bind(&GyroDriver::handleTareGoal, this, _1, _2),
                std::bind(&GyroDriver::handleTareCancel, this, _1),
                std::bind(&GyroDriver::handleTareAccepted, this, _1));

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
            delete tareStatusResource.lockResource();
        }

        private:

        void reloadParams()
        {
            //regular parameters
            RCLCPP_INFO(get_logger(), "Reloading gyro parameters");
            NodeParameters *params = nodeParamsResource.lockResource();
            params->frame = get_parameter("gyro_frame").as_string();
            params->rateFreq = get_parameter("rate_frequency").as_int();
            params->statusFreq = get_parameter("status_frequency").as_int();
            params->a = get_parameter("a").as_double();
            params->b = get_parameter("b").as_double();
            params->c = get_parameter("c").as_double();
            params->d = get_parameter("d").as_double();
            params->f = get_parameter("f").as_double();
            params->g = get_parameter("g").as_double();
            params->rateNormMean = get_parameter("rate_norm_mean").as_double();
            params->rateNormStd = get_parameter("rate_norm_std").as_double();
            params->tempNormMean = get_parameter("temp_norm_mean").as_double();
            params->tempNormStd = get_parameter("temp_norm_std").as_double();
            params->variance = get_parameter("variance").as_double();
            params->tempLimits = get_parameter("cal_temp_limits").as_double_array();
            nodeParamsResource.unlockResource();
        }


        rcl_interfaces::msg::SetParametersResult setParamsCb(const std::vector<rclcpp::Parameter> &)
        {
            reloadParams();

            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        }


        int16_t get16BitDataFromFields(UwrtGyroField field1, UwrtGyroField field2)
        {
            if(processor->hasDataForField(field1) && processor->hasDataForField(field2))
            {
                SerialDataStamped serialData = processor->getField(field1);
                int16_t intData = convertFromCString<int8_t>(serialData.data.data, serialData.data.numData);
                intData = intData << 8;
                serialData = processor->getField(field2);
                intData |= convertFromCString<int8_t>(serialData.data.data, serialData.data.numData) & 0xFF;
                return intData;
            }

            return 0;
        }

        //this timer routine publishes diagnostics and everything else that isnt rate
        void statusTimerCb()
        {
            NodeParameters *params = nodeParamsResource.lockResource();
            std::string frame = params->frame;
            std::vector<double> tempLimits = params->tempLimits;
            nodeParamsResource.unlockResource();

            riptide_msgs2::msg::GyroStatus *statMsg = statusResource.lockResource();
            statMsg->header.frame_id = frame;
            statMsg->header.stamp = get_clock()->now();

            //pack status with raw gyro data
            statMsg->raw_temperature = get16BitDataFromFields(UwrtGyroField::TEMP_HIGH, UwrtGyroField::TEMP_LOW);
            statMsg->raw_vsupply = get16BitDataFromFields(UwrtGyroField::VSUPPLY_HIGH, UwrtGyroField::VSUPPLY_LOW);
            statMsg->raw_sldcurrent = get16BitDataFromFields(UwrtGyroField::SLD_CURRENT_HIGH, UwrtGyroField::SLD_CURRENT_LOW);
            statMsg->raw_diagsignal = get16BitDataFromFields(UwrtGyroField::DIAG_SIGNAL_HIGH, UwrtGyroField::DIAG_SIGNAL_LOW);

            //pack status with processed gyro data
            statMsg->temperature = statMsg->raw_temperature * (250 / (float) 0x8000) - 50.0;
            statMsg->vsupply = statMsg->raw_vsupply * (10 / (float) 0x8000);
            statMsg->sldcurrent = statMsg->raw_sldcurrent * (0.25 / (float) 0x8000);
            statMsg->diagsignal = statMsg->raw_diagsignal * (2.5 / (float) 0x8000);

            //diagnostic checks
            if(tempLimits.size() >= 2)
            {
                statMsg->temp_within_cal = statMsg->temperature > tempLimits[0] && statMsg->temperature < tempLimits[1];
            } else
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Could not determine temperature limits. Parameters may be missing.");
                statMsg->temp_good = false;
            }

            statMsg->temp_good = statMsg->temperature < TEMP_UPPER;
            statMsg->vsupply_good = statMsg->vsupply > VSUPPLY_LOWER && statMsg->vsupply < VSUPPLY_UPPER;
            statMsg->sldcurrent_good = statMsg->sldcurrent > SLDCURRENT_LOWER && statMsg->sldcurrent < SLDCURRENT_UPPER;
            statMsg->diagsignal_good = statMsg->diagsignal > DIAG_SIGNAL_LOWER && statMsg->diagsignal < DIAG_SIGNAL_UPPER;
            
            statMsg->connected = processor->hasDataForField(UwrtGyroField::VRATE);

            //this does not mean connection yet, need to check that the time is within 1s
            if(statMsg->connected)
            {
                statMsg->connected = statMsg->connected && 
                    get_clock()->now() - processor->getField(UwrtGyroField::VRATE).timestamp < 1s;
            }

            //publish status
            statusPub->publish(*statMsg);
            statusResource.unlockResource();
        }


        void rateTimerCb()
        {
            //get fog frame id from params resource
            NodeParameters *params = nodeParamsResource.lockResource();
            std::string frame = params->frame;
            double variance = params->variance;
            nodeParamsResource.unlockResource();

            RateStatus *rate = rateStatusResource.lockResource();
            if(rate->samples > 0)
            {
                geometry_msgs::msg::TwistWithCovarianceStamped twistMsg;
                twistMsg.header.frame_id = frame;
                twistMsg.header.stamp = get_clock()->now();
                twistMsg.twist.covariance[35] = variance;
                twistMsg.twist.twist.angular.z = rate->accum / (double) rate->samples;
                rate->accum = 0;
                rate->samples = 0;
                twistPub->publish(twistMsg);
            }

            rateStatusResource.unlockResource();
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

            //get temperature value from status
            int gyroTemperature = statusResource.lockResource()->temperature;
            statusResource.unlockResource();

            //get params struct
            NodeParameters *params = nodeParamsResource.lockResource();

            //publish raw message if someone wants it
            if(rawDataPub->get_subscription_count() > 0)
            {
                riptide_msgs2::msg::Int32Stamped rawMsg;
                rawMsg.header.frame_id = params->frame;
                rawMsg.header.stamp = data.timestamp;
                rawMsg.data = signedRawReading;
                rawDataPub->publish(rawMsg);
            }

            //calculate and store rate
            double 
                normalizedRawReading = (signedRawReading - params->rateNormMean) / params->rateNormStd,
                normalizedGyroTemperature = (gyroTemperature - params->tempNormMean) / params->tempNormStd;

            //formula: z(x,y) = a + b*x^3 + c*x*y + d*cos(f*x + g)
            double rate = 
                params->a + 
                params->b * normalizedRawReading * normalizedRawReading * normalizedRawReading +
                params->c * normalizedRawReading * normalizedGyroTemperature +
                params->d * cos(params->f * normalizedRawReading + params->g);
            
            //add to tare if necessary
            TareStatus *tareStatus = tareStatusResource.lockResource();
            if(tareStatus->active && tareStatus->acquiredSamples < tareStatus->desiredSamples)
            {
                tareStatus->offset += rate / (double) tareStatus->desiredSamples;
                tareStatus->acquiredSamples++;
            } else
            {
                //apply tare to twist before publishing
                rate -= tareStatus->offset;
            }

            tareStatusResource.unlockResource();
            nodeParamsResource.unlockResource();

            //now add the rate to the resource, which will be divided into an average later
            RateStatus *rateStatus = rateStatusResource.lockResource();
            rateStatus->accum += rate;
            rateStatus->samples++;
            rateStatusResource.unlockResource();
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

            RCLCPP_INFO(get_logger(), "Thread func exiting.");
        }

        rclcpp_action::GoalResponse handleTareGoal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const TareGyro::Goal> goal)
        {
            RCLCPP_INFO(get_logger(), "Received gyro tare goal");
            (void)uuid;
            (void)goal;

            bool isTaring = tareStatusResource.lockResource()->active;
            tareStatusResource.unlockResource();
            if(!isTaring)
            {
                RCLCPP_INFO(get_logger(), "Accepting and executing tare");
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            } else 
            {
                RCLCPP_ERROR(get_logger(), "Rejecting tare because another is in progress");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }


        rclcpp_action::CancelResponse handleTareCancel(
            const std::shared_ptr<TareGyroGH> goalHandle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel tare");
            (void) goalHandle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }


        void handleTareAccepted(const std::shared_ptr<TareGyroGH> goalHandle)
        {
            std::thread{std::bind(&GyroDriver::doTare, this, _1), goalHandle}.detach();
        }


        void doTare(const std::shared_ptr<TareGyroGH> goalHandle)
        {
            TareGyro::Goal::ConstSharedPtr goal = goalHandle->get_goal();
            RCLCPP_INFO(get_logger(), "Beginning tare using %d samples with a timeout of %f seconds",
                        goal->num_samples, goal->timeout_seconds);

            //reset offset, set tare parameters, and activate
            TareStatus *status = tareStatusResource.lockResource();
            double oldOffset = status->offset;
            status->offset = 0;
            status->acquiredSamples = 0;
            status->desiredSamples = goal->num_samples;
            status->active = true; //setting this flag causes the tare to be updated by the serial callback
            tareStatusResource.unlockResource();

            //wait for tare to complete
            rclcpp::Time startTime = get_clock()->now();
            while((get_clock()->now() - startTime).seconds() < goal->timeout_seconds)
            {
                usleep(1000); //sleep 1ms
                status = tareStatusResource.lockResource();
                bool shouldBreak = status->acquiredSamples >= status->desiredSamples || goalHandle->is_canceling();
                tareStatusResource.unlockResource();
                if(shouldBreak)
                {
                    break;
                }
            }

            //deactivate tare
            RCLCPP_INFO(get_logger(), "Stopping tare");
            status = tareStatusResource.lockResource();
            double finalOffset = status->offset;
            status->active = false;
            tareStatusResource.unlockResource();

            //compose result
            TareGyro::Result::SharedPtr result = std::make_shared<TareGyro::Result>();

            if(goalHandle->is_canceling())
            {
                RCLCPP_INFO(get_logger(), "Tare canceled");

                //restore old tare offset
                status = tareStatusResource.lockResource();
                status->offset = oldOffset;
                tareStatusResource.unlockResource();

                result->success = false;
                result->result = "Tare canceled";
                goalHandle->canceled(result);
                return;
            }
            
            rclcpp::Duration elapsedTime = get_clock()->now() - startTime;
            if(elapsedTime.seconds() > goal->timeout_seconds)
            {
                RCLCPP_ERROR(get_logger(), "Tare timed out");

                //restore old tare offset
                status = tareStatusResource.lockResource();
                status->offset = oldOffset;
                tareStatusResource.unlockResource();

                result->success = false;
                result->result = "Tare timed out.";
                goalHandle->abort(result);
                return;
            }

            //if we get here, success! save tare to file
            RCLCPP_INFO(get_logger(), "Tare succeeded! Storing result to %s", tareFileName.c_str());
            std::ofstream file(tareFileName);
            file << std::to_string(finalOffset) << std::endl;
            file.close();

            //store results and end action
            result->success = true;
            result->result = "Tare succeeded.";
            goalHandle->succeed(result);
        }

        //consts
        const std::string tareFileName;

        //params
        ProtectedResource<NodeParameters*> nodeParamsResource;        

        //status
        ProtectedResource<riptide_msgs2::msg::GyroStatus*> statusResource;
        ProtectedResource<TareStatus*> tareStatusResource;
        ProtectedResource<RateStatus*> rateStatusResource;

        //tools
        std::shared_ptr<LinuxSerialTransceiver> transceiver;
        SerialProcessor::SharedPtr processor;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<riptide_msgs2::msg::Int32Stamped>::SharedPtr rawDataPub;
        rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twistPub;
        rclcpp::Publisher<riptide_msgs2::msg::GyroStatus>::SharedPtr statusPub;
        rclcpp_action::Server<TareGyro>::SharedPtr tareServer;
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
