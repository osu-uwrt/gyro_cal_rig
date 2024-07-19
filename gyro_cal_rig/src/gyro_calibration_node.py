#!/usr/bin/env python3

import os
import time
import datetime
import rclpy
import rclpy.action
from rclpy.action import ActionServer
from threading import Lock
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from gyro_cal_rig_msgs.action import CalibrateGyro
from gyro_cal_rig_msgs.msg import GyroRigStatus
from riptide_msgs2.msg import Int32Stamped, GyroStatus
from std_msgs.msg import Bool

RIG_CALIBRATE_ACTION = "cal_rig/calibrate"

RIG_COMMAND_TOPIC = "command/cal_rig"
RIG_STATUS_TOPIC = "status/cal_rig"
RIG_CALIBRATING_TOPIC = "status/cal_rig/calibrating"

GYRO_RAW_TOPIC = "gyro/raw"
GYRO_STATUS_TOPIC = "gyro/status"

CAL_LOG_COLUMNS = ["sec", "nanosec", "gyro_raw", "gyro_temp", "rig_rate", "rig_heating", "rig_enabled", "rig_stalled"]

STEPPER_COUNTS_PER_REV = 7.557241247324732e+05

def timeAsStr():
    return datetime.datetime.now().strftime("%m-%d-%Y_%H-%M-%S")

class DataLogger:
    def __init__(self, filePath, columnNames):
        self.names = columnNames
        self.file = open(filePath, 'w')
        
        # print(f"Opening {filePath}", flush=True)
        
        #write header to file
        self.logData(columnNames)
    
    def __del__(self):
        self.file.close()
    
    def logData(self, columns):
        # print(f"Logging {columns} to {self.file.name}", flush=True)
        self.file.write(f"{','.join([str(d) for d in columns])}\n")
    
    def flush(self):
        self.file.flush()


class GyroCalibrationNode(Node):
    def __init__(self):
        super().__init__("gyro_calibration_node")
        self.get_logger().info("Gyro calibrator starting")
        
        self._action_server = ActionServer(
            self,
            CalibrateGyro,
            RIG_CALIBRATE_ACTION,
            goal_callback=self.calGoalCb,
            cancel_callback=self.calCancelCb,
            execute_callback=self.doCalibration)

        #publishers
        self._rigCommandPub = self.create_publisher(GyroRigStatus, RIG_COMMAND_TOPIC, 10)
        self._calibratingPub = self.create_publisher(Bool, RIG_CALIBRATING_TOPIC, 10)
        
        #subscriptions
        self._rigStatusSub = self.create_subscription(GyroRigStatus, RIG_STATUS_TOPIC, self.rigStatusCb, 10)
        self._gyroRawSub = self.create_subscription(Int32Stamped, GYRO_RAW_TOPIC, self.gyroRawCb, qos_profile=qos_profile_sensor_data)
        self._gyroStatusSub = self.create_subscription(GyroStatus, GYRO_STATUS_TOPIC, self.gyroStatusCb, 10)
        
        #timers
        self._timer = self.create_timer(1, self.publishCalibrating)
        
        #sticky msgs
        self._rigStatus = GyroRigStatus()
        self._gyroRaw = Int32Stamped()
        self._gyroStatus = GyroStatus()
        
        #state
        self._calInProgress = False #true if calibration data is being collected
        self.logDir = ""
        self.lock = Lock()
        
        #handle parameters
        self.declare_parameter("allowable_temp_error", 500)
        self.declare_parameter("rig_ack_timeout", 3.0)
        self.add_on_set_parameters_callback(self.readParameters)
        self.readParameters()

        self.get_logger().info("Gyro calibration action ready")
    
    
    def publishCalibrating(self):
        msg = Bool()
        msg.data = self._calInProgress
        self._calibratingPub.publish(msg)
    
    
    def readParameters(self):
        self._tempRange = self.get_parameter("allowable_temp_error").value
        self._ackTimeout = self.get_parameter("rig_ack_timeout").value
    
    
    def stopRig(self):
        cmd = GyroRigStatus()
        cmd.enabled = False
        self._rigCommandPub.publish(cmd)
    
    
    def calGoalCb(self, handle):
        self.get_logger().info("Goal requested")
        return rclpy.action.GoalResponse.ACCEPT

    def calCancelCb(self, handle):
        self.get_logger().info("Cancel requested")
        return rclpy.action.CancelResponse.ACCEPT
    
    
    def doCalibration(self, goal_handle):
        result = CalibrateGyro.Result()
        if self._calInProgress:
            goal_handle.abort()
            result.result = "ERROR: Calibration already in progress"
            return result
            
        request = goal_handle.request
        logDir = request.log_directory
        self.get_logger().info(
            "PERFORMING CALIBRATION\n" + 
            "Calibration details:\n" + 
            f"  rates: {request.rates}" +
            f"  temps: {request.temps}" + 
            f"  log dir: {logDir}")
        
        
        datetimeStr = timeAsStr()
        calLogPath = os.path.join(logDir, f"{datetimeStr}_calibration.csv")
        
        with self.lock:
            self.calLogger = DataLogger(calLogPath, CAL_LOG_COLUMNS)
            
        self._calInProgress = True

        try:
            for i in range(0, len(request.temps)):
                #try to collect rates while reaching temp. if temp is 0 dont worry about it
                while not self.rigAtTemp(request.temps[i]) and request.temps[i] != 0 and not goal_handle.is_cancel_requested:
                    self.collectRates(i, goal_handle)
                        
                #now at temp, collect rates one more time
                if not goal_handle.is_cancel_requested:
                    self.collectRates(i, goal_handle)
        except RuntimeError as ex:
            self.stopRig()
            self._calInProgress = False
            result.result = f"ERROR: {ex}"
            self.get_logger().error(f"ERROR: {ex}")
            goal_handle.abort()
            return result
        
        self.stopRig()
        self._calInProgress = False
        with self.lock:
            self.calLogger.flush()
            
        if goal_handle.is_cancel_requested:
            self.get_logger().info("Calibration cancel requested")
            goal_handle.canceled()
            result.result = "Calibration canceled."
            self.get_logger().info("Calibration canceled.")
        else:
            goal_handle.succeed()    
            result.result = "Calibration successful."
            self.get_logger().info("Calibration successful.")
            
        return result
    
    def collectRates(self, temp_idx, handle):
        rates = handle.request.rates
        for i in range(0, len(rates)):
            #report what we are currently doing
            feedback = CalibrateGyro.Feedback()
            feedback.current_rate_idx = i
            feedback.current_temp_idx = temp_idx
            handle.publish_feedback(feedback)
            
            #set rig
            desiredStatus = GyroRigStatus()
            desiredStatus.rate = int(rates[i] * STEPPER_COUNTS_PER_REV)
            desiredStatus.heat = self.rigShouldHeat(handle.request.temps[temp_idx])
            desiredStatus.enabled = True
            self._rigCommandPub.publish(desiredStatus)
            
            #wait for rig to acknowledge
            ackStartTime = self.get_clock().now()
            while self.get_clock().now().seconds_nanoseconds()[0] - ackStartTime.seconds_nanoseconds()[0] < self._ackTimeout:
                if self._rigStatus.rate == desiredStatus.rate \
                and self._rigStatus.heat == desiredStatus.heat \
                and self._rigStatus.enabled == desiredStatus.enabled:
                    break
            
            if self.get_clock().now().seconds_nanoseconds()[0] - ackStartTime.seconds_nanoseconds()[0] >= self._ackTimeout:
                raise RuntimeError("Could not send command to rig!")
            
            #successfully set rig. sit collecting data now
            time.sleep(handle.request.seconds_per_rate)
            
            if handle.is_cancel_requested:
                self.get_logger().info("Preempting rate collection")
                return False
            
        return True
    
    #TODO implement these
    def rigAtTemp(self, setpt):
        return \
            self._gyroStatus.temperature > setpt - self._tempRange and \
            self._gyroStatus.temperature < setpt + self._tempRange
    
    def rigShouldHeat(self, setpt):
        if self._rigStatus.heat:
            return self._gyroStatus.temperature < setpt - self._tempRange + 10
        
        return self._gyroStatus.temperature < setpt - self._tempRange

    def updateLog(self):
        now = self.get_clock().now()
        
        if self._calInProgress:
            with self.lock:
                #CAL_LOG_COLUMNS = ["sec", "nanosec", "gyro_raw", "gyro_temp", "rig_rate", "rig_heating", "rig_enabled", "rig_stalled"]
                self.calLogger.logData(
                    [
                        now.to_msg().sec, 
                        now.to_msg().nanosec,
                        self._gyroRaw.data,
                        self._gyroStatus.temperature,
                        self._rigStatus.rate,
                        self._rigStatus.heat,
                        self._rigStatus.enabled,
                        self._rigStatus.stalled
                    ]
                )

    def rigStatusCb(self, msg):
        self._rigStatus = msg
        self.updateLog()
    
    def gyroRawCb(self, msg):
        self._gyroRaw = msg
        self.updateLog()

    def gyroStatusCb(self, msg):
        self._gyroStatus = msg
        self.updateLog()
        

def main(args = None):
    rclpy.init(args = args)
    node = GyroCalibrationNode()
    rclpy.spin(node, executor=MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
