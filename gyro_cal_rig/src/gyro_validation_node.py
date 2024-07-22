#!/usr/bin/env python3

import os
import time
from threading import Lock
import rclpy
import rclpy.action
from math import pi
from rclpy.executors import MultiThreadedExecutor
from transforms3d.euler import quat2euler
from nav_msgs.msg import Odometry
from gyro_cal_rig_msgs.action import CalibrateGyro
from gyro_cal_rig_msgs.msg import GyroRigStatus
from riptide_msgs2.msg import GyroStatus
from procedure_node import GyroProcedureNode
from data_logger import DataLogger
from procedure_node import GyroProcedureNode, RIG_STATUS_TOPIC, GYRO_STATUS_TOPIC

LOG_COLUMNS = ["sec", "nanosec", "gyro_temp", "rig_rate", "rig_heating", "rig_enabled", "rig_stalled", "expected_yaw", "actual_yaw"]
RATE_STEP_SIZE = 37786

class GyroValidationNode(GyroProcedureNode):
    def __init__(self):
        super().__init__("gyro_validation_node", self.validateCb)
        self.lock = Lock()
        
        #subscriptions
        self._rigStatusSub = self.create_subscription(GyroRigStatus, RIG_STATUS_TOPIC, self.rigStatusCb, 10)
        self._gyroStatusSub = self.create_subscription(GyroStatus, GYRO_STATUS_TOPIC, self.gyroStatusCb, 10)
        self._odomSub = self.create_subscription(Odometry, "odometry/filtered", self.odomCb, 10)
        
        self._rigStatus = GyroRigStatus()
        self._gyroStatus = GyroStatus()
        self._odom = Odometry()
        self._expectedYaw = 0
        
        self.get_logger().info("Gyro validation node ready")
        

    def validateCb(self, handle):
        result = CalibrateGyro.Result()
        if self._calInProgress:
            handle.abort()
            return result

        request = handle.request
        logDir = request.log_directory
        self.get_logger().info(
            "PERFORMING VALIDATION\n" +
            "Validation details:\n" + 
            f"  rates: {request.rates}\n" +
            f"  temps: {request.temps}\n" +
            f"  log dir: {logDir}")

        datetimeStr = self.timeAsStr()
        valLogPath = os.path.join(logDir, f"{datetimeStr}_validation.csv")
        
        with self.lock:
            self.valLogger = DataLogger(valLogPath, LOG_COLUMNS)
                
        self._calInProgress = True
        
        try:
            for i in range(0, len(request.temps)):
                while not self.rigAtTemp(request.temps[i]) and request.temps[i] != 0 and not handle.is_cancel_requested:
                    self.collectPositions(i, handle)
                
                if not handle.is_cancel_requested:
                    self.collectPositions(i, handle)
        except BaseException as ex:
            self.stopRig()
            self._calInProgress = False
            result.result = f"ERROR: {ex}"
            self.get_logger().error(f"ERROR: {ex}")
            handle.abort()
            return result
        
        self.stopRig()
        self._calInProgress = False
        with self.lock:
            self.valLogger.flush()
        
        if handle.is_cancel_requested:
            self.get_logger().info("Validation cancel requested")
            handle.canceled()
            result.result = "Validation canceled."
            self.get_logger().info("Validation canceled.")
        else:
            handle.succeed()
            result.result = "Validation successful."
            self.get_logger().info("Validation successful.")
        
        return result

    
    def collectPositions(self, tempIdx, handle):
        rates = handle.request.rates
        for i in range(0, len(rates)):
            #feedback
            feedback = CalibrateGyro.Feedback()
            feedback.current_rate_idx = i
            feedback.current_temp_idx = tempIdx
            handle.publish_feedback(feedback)
            
            # four step process:
            #  - drive rig at rate for time
            #  - stop rig and rest for time
            #  - drive in other direction for time
            #  - stop and rest for time
            
            self.resetExpectedYaw()
            
            self.rampToSpeed(rates[i], self.rigShouldHeat(handle.request.temps[tempIdx]))
            time.sleep(handle.request.seconds_per_rate)
            self.sendRigCommand(0, self.rigShouldHeat(handle.request.temps[tempIdx]))
            time.sleep(handle.request.seconds_per_rate)
            
            self.rampToSpeed(-1 * rates[i], self.rigShouldHeat(handle.request.temps[tempIdx]))
            time.sleep(handle.request.seconds_per_rate)
            self.sendRigCommand(0, self.rigShouldHeat(handle.request.temps[tempIdx]))
            time.sleep(handle.request.seconds_per_rate)
            
            if handle.is_cancel_requested:
                self.get_logger().info("Preempting validation")
                return False

        return True

        
    def rampToSpeed(self, rate, heat):
        while abs(rate - self._rigStatus.rate) > RATE_STEP_SIZE:
            direction = 1 if rate - self._rigStatus.rate > 0 else -1
            newRate = self._rigStatus.rate + direction * RATE_STEP_SIZE
            self.sendRigCommand(newRate, heat)
        
        self.sendRigCommand(rate, heat)
    
    
    def getCurrentYaw(self):
        q = self._odom.pose.pose.orientation
        return quat2euler([q.w, q.x, q.y, q.z])[2]
    
    
    def resetExpectedYaw(self):
        self._expectedYaw = self.getCurrentYaw()
        
    
    def secondsFromTimeMsg(self, time):
        return time.sec + time.nanosec / 1000000000.0
    
    
    def updateLog(self):
        now = self.get_clock().now()
        
        if self._calInProgress:
            with self.lock:
                #["sec", "nanosec", "gyro_temp", "rig_rate", "rig_heating", "rig_enabled", "rig_stalled", "expected_yaw", "actual_yaw"]
                self.valLogger.logData(
                    [
                        now.to_msg().sec, 
                        now.to_msg().nanosec,
                        self._gyroStatus.temperature,
                        self._rigStatus.rate,
                        self._rigStatus.heat,
                        self._rigStatus.enabled,
                        self._rigStatus.stalled,
                        self._expectedYaw,
                        self.getCurrentYaw()
                    ]
                )
    
    
    def rigStatusCb(self, msg: GyroRigStatus):
        #update expected yaw
        timeSinceLast = self.secondsFromTimeMsg(msg.header.stamp) - self.secondsFromTimeMsg(self._rigStatus.header.stamp) #returns seconds
        radiansTravelled = timeSinceLast * msg.rate / self._stepsPerRev
        self._expectedYaw += radiansTravelled * 2 * pi
        self._rigStatus = msg
        self.updateLog()
        

    def gyroStatusCb(self, msg: GyroStatus):
        self._gyroStatus = msg
        self.updateLog()
        
    def odomCb(self, msg: Odometry):
        self._odom = msg
        self.updateLog()


def main(args = None):
    rclpy.init(args = args)
    node = GyroValidationNode()
    rclpy.spin(node, executor=MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
