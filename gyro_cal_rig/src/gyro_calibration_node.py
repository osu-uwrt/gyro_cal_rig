#!/usr/bin/env python3

import os
import time
import datetime
from threading import Lock
import rclpy
from rclpy.executors import MultiThreadedExecutor
from gyro_cal_rig_msgs.action import CalibrateGyro
from gyro_cal_rig_msgs.msg import GyroRigStatus
from riptide_msgs2.msg import Int32Stamped, GyroStatus
from rclpy.qos import qos_profile_sensor_data
from data_logger import DataLogger
from procedure_node import GyroProcedureNode, RIG_STATUS_TOPIC, GYRO_RAW_TOPIC, GYRO_STATUS_TOPIC

CAL_LOG_COLUMNS = ["sec", "nanosec", "gyro_raw", "gyro_temp", "rig_rate", "rig_heating", "rig_enabled", "rig_stalled"]

class GyroCalibrationNode(GyroProcedureNode):
    def __init__(self):
        super().__init__("gyro_calibration_node", self.doCalibration)
        
        self.lock = Lock()
        
        #subscriptions
        self._rigStatusSub = self.create_subscription(GyroRigStatus, RIG_STATUS_TOPIC, self.rigStatusCb, 10)
        self._gyroRawSub = self.create_subscription(Int32Stamped, GYRO_RAW_TOPIC, self.gyroRawCb, qos_profile=qos_profile_sensor_data)
        self._gyroStatusSub = self.create_subscription(GyroStatus, GYRO_STATUS_TOPIC, self.gyroStatusCb, 10)
        
        self.get_logger().info("Gyro calibration node ready")
    
    
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
            f"  rates: {request.rates}\n" +
            f"  temps: {request.temps}\n" + 
            f"  log dir: {logDir}")
        
        try:
            datetimeStr = self.timeAsStr()
            calLogPath = os.path.join(logDir, f"{datetimeStr}_calibration.csv")
            
            with self.lock:
                self.calLogger = DataLogger(calLogPath, CAL_LOG_COLUMNS)
                
            self._calInProgress = True

            for i in range(0, len(request.temps)):
                #try to collect rates while reaching temp. if temp is 0 dont worry about it
                while not self.rigAtTemp(request.temps[i]) and request.temps[i] != 0 and not goal_handle.is_cancel_requested:
                    self.collectRates(i, goal_handle)
                        
                #now at temp, collect rates one more time
                if not goal_handle.is_cancel_requested:
                    self.collectRates(i, goal_handle)
        except BaseException as ex:
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
    
    def collectRates(self, tempIdx, handle):
        rates = handle.request.rates
        for i in range(0, len(rates)):
            #report what we are currently doing
            feedback = CalibrateGyro.Feedback()
            feedback.current_rate_idx = i
            feedback.current_temp_idx = tempIdx
            handle.publish_feedback(feedback)
            
            self.sendRigCommand(rates[i], self.rigShouldHeat(handle.request.temps[tempIdx]))
            
            #successfully set rig. sit collecting data now
            time.sleep(handle.request.seconds_per_rate)
            
            if handle.is_cancel_requested:
                self.get_logger().info("Preempting rate collection")
                return False
            
        return True
    
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
