#include "rclcpp/rclcpp.hpp"
#include "riptide_gyro/serial_library.hpp"
#include "gyro_cal_rig_msgs/msg/gyro_rig_status.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

//rigid settings
#define RIG_BAUD 9600

#define STEPPER_ENABLED 0x01
#define RIG_HEATING 0b00000010
#define STEPPER_STALLED 0x02

enum GyroRigField
{
    RIG_DESIRED_STATUS,
    RIG_DESIRED_RATE,
    RIG_ACTUAL_STATUS,
    RIG_ACTUAL_RATE,
};

enum GyroRigFrame
{
    RIG_DESIRED_FRAME,
    RIG_ACTUAL_FRAME
};

const SerialFramesMap RIG_STATUS_FRAMES = {
    {
        RIG_DESIRED_FRAME,
        SerialFrame ({
            FIELD_SYNC,
            RIG_DESIRED_STATUS,
            RIG_DESIRED_RATE,
            RIG_DESIRED_RATE,
            RIG_DESIRED_RATE,
            RIG_DESIRED_RATE
        })
    },
    {
        RIG_ACTUAL_FRAME,
        SerialFrame ({
            FIELD_SYNC,
            RIG_ACTUAL_STATUS,
            RIG_ACTUAL_RATE,
            RIG_ACTUAL_RATE,
            RIG_ACTUAL_RATE,
            RIG_ACTUAL_RATE
        })
    }
};

const char syncValue = ';';

class GyroRigDriver : public rclcpp::Node
{
    public:
    GyroRigDriver() : rclcpp::Node("gyro_rig_driver")
    {
        RCLCPP_INFO(get_logger(), "Starting gyro rig node");

        //parameters
        declare_parameter("port", "/dev/ttyUSB0");
        std::string port = get_parameter("port").as_string();

        //initialize serial library
        try
        {
            transceiver = std::make_shared<uwrt_gyro::LinuxSerialTransceiver>(port, RIG_BAUD, 0, 1);
            processor = std::make_shared<uwrt_gyro::SerialProcessor>(*transceiver, RIG_STATUS_FRAMES, RIG_ACTUAL_FRAME, &syncValue, 1);
            processor->setNewMsgCallback(std::bind(&GyroRigDriver::serialMsgReceived, this));

            //initial values
            SerialData zeroData;
            zeroData.data[0] = '\0';
            zeroData.numData = 1;
            processor->setField(GyroRigField::RIG_DESIRED_STATUS, zeroData, rclcpp::Time(0));
            processor->setField(GyroRigField::RIG_DESIRED_RATE, zeroData, rclcpp::Time(0));
        } catch(FatalSerialLibraryException& ex)
        {
            RCLCPP_FATAL(get_logger(), "Failed to initialize serial processor! %s", ex.what().c_str());
            exit(1);
        }

        //ROS connections
        rigStatePub = create_publisher<gyro_cal_rig_msgs::msg::GyroRigStatus>("status/cal_rig", 10);
        rigCommandSub = create_subscription<gyro_cal_rig_msgs::msg::GyroRigStatus>(
            "command/cal_rig",
            10,
            std::bind(&GyroRigDriver::rigCommandCb, this, _1));

        //start timer
        timer = create_wall_timer(
            100ms,
            std::bind(&GyroRigDriver::timerCb, this));
        
        RCLCPP_INFO(get_logger(), "Gyro rig node started.");
    }


    void timerCb()
    {
        try
        {
            //process incoming messages
            processor->update(get_clock()->now());

            //send out desired state
            processor->send(GyroRigFrame::RIG_DESIRED_FRAME);
        } catch(NonFatalSerialLibraryException& ex)
        {
            RCLCPP_WARN(get_logger(), "Caught exception: %s", ex.what().c_str());
        }
    }
    
    
    void rigCommandCb(gyro_cal_rig_msgs::msg::GyroRigStatus::ConstSharedPtr msg)
    {
        rclcpp::Time now = get_clock()->now();

        SerialData status;
        status.data[0] = 0;
        status.data[0] |= (msg->enabled ? STEPPER_ENABLED : 0);
        status.data[0] |= (msg->heat ? RIG_HEATING : 0);
        status.numData = 1;
        processor->setField(GyroRigField::RIG_DESIRED_STATUS, status, now);

        SerialData rate;
        size_t sz = uwrt_gyro::convertToCString<int32_t>(msg->rate, rate.data, sizeof(int32_t));
        rate.numData = sz;
        processor->setField(GyroRigField::RIG_DESIRED_RATE, rate, now);
    }


    void serialMsgReceived()
    {
        if(processor->hasDataForField(GyroRigField::RIG_ACTUAL_STATUS)
            && processor->hasDataForField(GyroRigField::RIG_ACTUAL_RATE))
        {
            //msg received contained status information. publish it
            SerialDataStamped status = processor->getField(GyroRigField::RIG_ACTUAL_STATUS);
            gyro_cal_rig_msgs::msg::GyroRigStatus state;
            state.header.stamp = status.timestamp;
            state.enabled = status.data.data[0] & STEPPER_ENABLED;
            state.heat = status.data.data[0] & RIG_HEATING;
            state.stalled = status.data.data[0] & STEPPER_STALLED;
            
            SerialData rateData = processor->getField(GyroRigField::RIG_ACTUAL_RATE).data;
            state.rate = uwrt_gyro::convertFromCString<int32_t>(rateData.data, rateData.numData);
            rigStatePub->publish(state);
        } else
        {
            RCLCPP_WARN(get_logger(), "Received serial message does not contain all information!");
        }
    }    

    private:
    std::shared_ptr<uwrt_gyro::LinuxSerialTransceiver> transceiver;
    std::shared_ptr<uwrt_gyro::SerialProcessor> processor;
    rclcpp::Publisher<gyro_cal_rig_msgs::msg::GyroRigStatus>::SharedPtr rigStatePub;
    rclcpp::Subscription<gyro_cal_rig_msgs::msg::GyroRigStatus>::SharedPtr rigCommandSub;
    rclcpp::TimerBase::SharedPtr timer;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<GyroRigDriver> node = std::make_shared<GyroRigDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
