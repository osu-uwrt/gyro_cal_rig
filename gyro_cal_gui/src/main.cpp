#include <ui_rig_gui.h>
#include <QApplication>
#include <QMainWindow>
#include <QFileDialog>
#include <QTimer>
#include <thread>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <riptide_msgs2/msg/int32_stamped.hpp>
#include <riptide_msgs2/msg/gyro_status.hpp>
#include <gyro_cal_rig_msgs/msg/gyro_rig_status.hpp>
#include <gyro_cal_rig_msgs/action/calibrate_gyro.hpp>

#define GYRO_STALE_TIME 200ms
#define RIG_STALE_TIME 1s
#define GYRO_CAL_ACTION "cal_rig/calibrate"
#define CALIBRATING_TOPIC "status/cal_rig/calibrating"
#define GYRO_RAW_TOPIC "gyro/raw"
#define GYRO_STATUS_TOPIC "gyro/status"
#define RIG_STATUS_TOPIC "status/cal_rig"
#define RIG_COMMAND_TOPIC "command/cal_rig"

#define STEPPER_COUNTS_PER_REV 240 * 256 * 10.8

using namespace std::placeholders;
using namespace std::chrono_literals;

class GyroCalGuiNode : public rclcpp::Node
{
    public:
    typedef std::shared_ptr<GyroCalGuiNode> SharedPtr;
    typedef gyro_cal_rig_msgs::action::CalibrateGyro CalibrateGyro;
    typedef rclcpp_action::ClientGoalHandle<CalibrateGyro> CalibrateGyroGH;

    GyroCalGuiNode(Ui_GyroRigGui *ui)
     : rclcpp::Node("gyro_cal_gui"),
       ui(ui)
    {
        rclcpp::Time currentTime = get_clock()->now();
        lastGyroRawTime = currentTime;
        lastGyroStatusTime = currentTime;
        lastRigStatusTime = currentTime;
        
        //ui connection
        ui->centralwidget->connect(ui->uiBrowseLogDir, &QPushButton::clicked, [this]() { this->onBrowseLogDirButtonPressed(); });
        ui->centralwidget->connect(ui->uiRigStartCal, &QPushButton::clicked, [this]() { this->onCalibrateButtonPressed(); });
        ui->centralwidget->connect(ui->uiRigCommand, &QPushButton::clicked, [this]() { this->onCommandButtonPressed(); });

        //update timer for timeout detection
        timer = create_wall_timer(
            100ms,
            std::bind(&GyroCalGuiNode::timerCb, this)
        );

        //subscriptions
        calibratingSub = create_subscription<std_msgs::msg::Bool>(
            CALIBRATING_TOPIC,
            10,
            std::bind(&GyroCalGuiNode::calibratingCb, this, _1)
        );

        gyroRawSub = create_subscription<riptide_msgs2::msg::Int32Stamped>(
            GYRO_RAW_TOPIC,
            rclcpp::SensorDataQoS(),
            std::bind(&GyroCalGuiNode::gyroRawCb, this, _1)
        );

        gyroStatusSub = create_subscription<riptide_msgs2::msg::GyroStatus>(
            GYRO_STATUS_TOPIC,
            10,
            std::bind(&GyroCalGuiNode::gyroStatusCb, this, _1)
        );

        rigStatusSub = create_subscription<gyro_cal_rig_msgs::msg::GyroRigStatus>(
            RIG_STATUS_TOPIC,
            10,
            std::bind(&GyroCalGuiNode::rigStatusCb, this, _1)
        );

        //publishers
        rigCommandPub = create_publisher<gyro_cal_rig_msgs::msg::GyroRigStatus>(
            RIG_COMMAND_TOPIC,
            10
        );

        //action client
        calibrateGyroClient = rclcpp_action::create_client<CalibrateGyro>(this, GYRO_CAL_ACTION);
    }

    private slots:
    void onBrowseLogDirButtonPressed()
    {
        QString dirPath = QFileDialog::getExistingDirectory(ui->centralwidget, "Browse for log directory", "~");
        ui->uiLogDir->setText(dirPath);
    }

    void onCalibrateButtonPressed()
    {
        setStatus("Starting gyro calibration", false);
        if(!calibrateGyroClient->wait_for_action_server(2s))
        {
            setStatus("Calibration server unavailable.", true);
            return;
        }

        double
            minRate = ui->uiRigMinRate->value(),
            maxRate = ui->uiRigMaxRate->value(),
            rateStep = ui->uiRigRateStep->value(),
            minTemp = ui->uiRigMinTemp->value(),
            maxTemp = ui->uiRigMaxTemp->value(),
            tempStep = ui->uiRigTempStep->value();
        
        bool 
            useBothDirections = ui->uiRigUseBothDirs->isChecked(),
            reverse = false;
        
        int i;
        
        CalibrateGyro::Goal goal;
        goal.log_directory = ui->uiLogDir->text().toStdString();
        goal.seconds_per_rate = ui->uiRigTimePerRate->value();
        
        //add rates
        int numRateSteps = (maxRate - minRate) / rateStep;
        
        do
        {
            for(i = 0; i < numRateSteps; i++)
            {
                double rpsRate = i * rateStep + minRate; //rotations per second rate
                goal.rates.push_back(rpsRate * (reverse ? -1 : 1));
            }

            reverse = !reverse;
        } while(reverse && useBothDirections);

        //add temps
        int numTempSteps = (maxTemp - minTemp) / tempStep;
        for(i = 0; i < numTempSteps; i++)
        {
            goal.temps.push_back(i * tempStep + minTemp);
        }

        if(goal.rates.size() == 0)
        {
            RCLCPP_WARN(get_logger(), "Adding 0 to rates because it is empty.");
            goal.rates.push_back(0);
        }

        if(goal.temps.size() == 0)
        {
            RCLCPP_WARN(get_logger(), "Adding 0 to temps because it is empty.");
            goal.temps.push_back(0);
        }

        requestedRates = goal.rates;
        requestedTemps = goal.temps;

        //goal options
        rclcpp_action::Client<CalibrateGyro>::SendGoalOptions send_goal_options;
        send_goal_options.goal_response_callback =
            std::bind(&GyroCalGuiNode::calGoalResponseCb, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&GyroCalGuiNode::calFeedbackCb, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&GyroCalGuiNode::calResultCb, this, _1);
        
        //send goal
        setStatus("Sending goal", false);
        this->calibrateGyroClient->async_send_goal(goal, send_goal_options);
    }

    void onCommandButtonPressed()
    {
        gyro_cal_rig_msgs::msg::GyroRigStatus cmd;
        cmd.enabled = ui->uiRigManualEnabled->isChecked();
        cmd.rate = ui->uiRigManualRate->value() * STEPPER_COUNTS_PER_REV;
        cmd.heat = ui->uiRigManualHeat->isChecked();
        rigCommandPub->publish(cmd);
    }

    private:
    //utilities
    void setStatus(const std::string& status, bool error)
    {
        if(error)
        {
            ui->uiCalStatus->setStyleSheet("color: red");
            RCLCPP_ERROR(get_logger(), status.c_str());
        } else
        {
            ui->uiCalStatus->setStyleSheet("color: black");
            RCLCPP_INFO(get_logger(), status.c_str());
        }

        ui->uiCalStatus->setText(QString::fromStdString(status));
    }

    //general callbacks
    void timerCb()
    {
        rclcpp::Time currentTime = get_clock()->now();
        ui->uiGyroOnline->setChecked(currentTime - lastGyroRawTime < GYRO_STALE_TIME || currentTime - lastGyroStatusTime < GYRO_STALE_TIME);
        ui->uiRigOnline->setChecked(currentTime - lastRigStatusTime < RIG_STALE_TIME);
        ui->uiRigRosTime->setText(QString::fromStdString(std::to_string(currentTime.nanoseconds() / 1000000000.0)));
    }

    void calibratingCb(std_msgs::msg::Bool::ConstSharedPtr msg)
    {
        ui->uiRigCalibrating->setChecked(msg->data);
    }

    void gyroRawCb(riptide_msgs2::msg::Int32Stamped::ConstSharedPtr msg)
    {
        ui->uiGyroRate->setText(QString::fromStdString(std::to_string(msg->data)));
        lastGyroRawTime = msg->header.stamp;
    }

    void gyroStatusCb(riptide_msgs2::msg::GyroStatus::ConstSharedPtr msg)
    {
        ui->uiGyroTemp->setText(QString::fromStdString(std::to_string(msg->temperature)));
        lastGyroStatusTime = msg->header.stamp;
    }

    void rigStatusCb(gyro_cal_rig_msgs::msg::GyroRigStatus::SharedPtr msg)
    {
        ui->uiRigEnabled->setChecked(msg->enabled);
        ui->uiRigHeating->setChecked(msg->heat);
        ui->uiRigMotorStalled->setChecked(msg->stalled);
        ui->uiRigMotorRate->setText(QString::fromStdString(std::to_string(msg->rate)));
        lastRigStatusTime = msg->header.stamp;
    }

    //action client callbacks
    void calGoalResponseCb(std::shared_ptr<GyroCalGuiNode::CalibrateGyroGH> handle)
    {
        if (!handle) {
            setStatus("Calibration rejected!", true);
        } else {
            setStatus("Calibration in progress", false);
        }
    }

    void calFeedbackCb(
        CalibrateGyroGH::SharedPtr,
        const std::shared_ptr<const CalibrateGyro::Feedback> feedback)
    {
        ui->uiCurrentRate->setText(QString::fromStdString(std::to_string(feedback->current_rate_idx)));
        ui->uiCurrentTemp->setText(QString::fromStdString(std::to_string(feedback->current_temp_idx)));

        //compute progress
        int
            possibleCombinations = requestedRates.size() * requestedTemps.size(),
            triedCombinations = requestedRates.size() * (feedback->current_temp_idx) + feedback->current_rate_idx,
            combinationsToGo = possibleCombinations - triedCombinations;
        
        double progressPercent = triedCombinations / (double) possibleCombinations;
        ui->uiCalProgress->setValue((int) (progressPercent * 100));

        std::string statusString = "Calibration in progress (" + std::to_string(combinationsToGo) + " combinations to go)";
        setStatus(statusString, false);
    }

    void calResultCb(const CalibrateGyroGH::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                setStatus("Calibration aborted", true);
                return;
            case rclcpp_action::ResultCode::CANCELED:
                setStatus("Calibration canceled", false);
                return;
            default:
                setStatus("Unknown result code", true);
                return;
        }
        
        ui->uiCalProgress->setValue(100);
        setStatus("Calibration succeeded!", false);
    }

    
    private:
    Ui_GyroRigGui *ui;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr calibratingSub;
    rclcpp::Subscription<riptide_msgs2::msg::Int32Stamped>::SharedPtr gyroRawSub;
    rclcpp::Subscription<riptide_msgs2::msg::GyroStatus>::SharedPtr gyroStatusSub;
    rclcpp::Subscription<gyro_cal_rig_msgs::msg::GyroRigStatus>::SharedPtr rigStatusSub;
    rclcpp::Publisher<gyro_cal_rig_msgs::msg::GyroRigStatus>::SharedPtr rigCommandPub;
    rclcpp_action::Client<CalibrateGyro>::SharedPtr calibrateGyroClient;
    std::vector<float> requestedRates;
    std::vector<int16_t> requestedTemps;

    rclcpp::Time
        lastGyroRawTime,
        lastGyroStatusTime,
        lastRigStatusTime;
};

GyroCalGuiNode::SharedPtr node;

void rosMain(Ui_GyroRigGui *ui)
{
    node = std::make_shared<GyroCalGuiNode>(ui);
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QMainWindow w;
    Ui_GyroRigGui *ui = new Ui_GyroRigGui();
    ui->setupUi(&w);
    rclcpp::init(argc, argv);
    rosMain(ui);
    // std::thread rosThread(rosMain, ui);
    QTimer timer(ui->centralwidget);
    timer.callOnTimeout([](){ rclcpp::spin_some(node); });
    timer.setInterval(1ms);
    timer.start();
    w.show();
    int guiResult = a.exec();
    rclcpp::shutdown();
    // rosThread.join();
    return guiResult;
}
