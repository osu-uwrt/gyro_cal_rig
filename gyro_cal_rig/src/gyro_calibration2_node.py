#!/usr/bin/env python3

import os
import time
import numpy as np
from threading import Lock
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter_service import SetParameters
from gyro_cal_rig_msgs.action import CalibrateGyro
from gyro_cal_rig_msgs.msg import GyroRigStatus
from riptide_msgs2.msg import GyroStatus, Int32Stamped
from data_logger import DataLogger
from procedure_node import GyroProcedureNode, RIG_STATUS_TOPIC, GYRO_STATUS_TOPIC

RATE_BUFFER_SIZE = 100
GYRO_SET_PARAMS_CLIENT = "riptide_gyro/set_parameters"
CAL_LOG_COLUMNS = ["sec", "nanosec", "gyro_temp", "rig_rate", "odom_rate", "rig_heating", "rig_enabled", "rig_stalled"]

class GyroCalibrationNode(GyroProcedureNode):
    def __init__(self):
        super().__init__("gyro_calibration2_node", self.doCalibration2)
        
        self.lock = Lock()
                
        #subscriptions
        self._rigStatusSub = self.create_subscription(GyroRigStatus, RIG_STATUS_TOPIC, self.rigStatusCb, 10)
        self._gyroStatusSub = self.create_subscription(GyroStatus, GYRO_STATUS_TOPIC, self.gyroStatusCb, 10)
        self._odomSub = self.create_subscription(Int32Stamped, "gyro/raw", self.rawCb, 10)       
        
        #clients
        self._gyroParameterClient = self.create_client(SetParameters, GYRO_SET_PARAMS_CLIENT) 
        
        #avg
        self._rateBuffer = np.zeros(RATE_BUFFER_SIZE)
        self._rateBufferLoc = 0
        self._standingRate = 0
        
        self.get_logger().info("Gyro calibration node ready")
    
    
    def doCalibration2(self, goal_handle):
        result = CalibrateGyro.Result()
        if self._calInProgress:
            goal_handle.abort()
            result.result = "ERROR: Calibration2 already in progress"
            return result
            
        request = goal_handle.request
        logDir = request.log_directory
        self.get_logger().info(
            "PERFORMING CALIBRATION2\n" + 
            "Calibration2 details:\n" + 
            f"  rates: {request.rates}\n" +
            f"  temps: {request.temps}\n" + 
            f"  log dir: {logDir}")
        
        try:
            datetimeStr = self.timeAsStr()
            calLogPath = os.path.join(logDir, f"{datetimeStr}_calibration2.csv")
            
            with self.lock:
                self.calLogger = DataLogger(calLogPath, CAL_LOG_COLUMNS)
                
            self._calInProgress = True

            for i in range(0, len(request.temps)):
                #try to collect rates while reaching temp. if temp is 0 dont worry about it
                while not self.rigAtTemp(request.temps[i]) and request.temps[i] != 0 and not goal_handle.is_cancel_requested:
                    self.stopRig(heat = True)
                    time.sleep(1)
                        
                # now at temp, collect rates
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
            self.get_logger().info("Calibration2 cancel requested")
            goal_handle.canceled()
            result.result = "Calibration2 canceled."
            self.get_logger().info("Calibration2 canceled.")
        else:
            goal_handle.succeed()    
            result.result = "Calibration2 successful."
            self.get_logger().info("Calibration2 successful.")
            
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
                # CAL_LOG_COLUMNS = ["sec", "nanosec", "gyro_temp", "rig_rate", "odom_rate", "rig_heating", "rig_enabled", "rig_stalled"]
                self.calLogger.logData(
                    [
                        now.to_msg().sec, 
                        now.to_msg().nanosec,
                        self._gyroStatus.temperature,
                        self._rigStatus.rate,
                        self._standingRate,
                        self._rigStatus.heat,
                        self._rigStatus.enabled,
                        self._rigStatus.stalled
                    ]
                )

    def rigStatusCb(self, msg):
        self._rigStatus = msg
        self.updateLog()

    def gyroStatusCb(self, msg):
        self._gyroStatus = msg
        self.updateLog()
        
    def rawCb(self, msg):
        self._rateBuffer[self._rateBufferLoc] = msg.data
        self._rateBufferLoc += 1
        if self._rateBufferLoc >= RATE_BUFFER_SIZE:
            self._rateBufferLoc = 0
            self._standingRate = np.average(self._rateBuffer)
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
