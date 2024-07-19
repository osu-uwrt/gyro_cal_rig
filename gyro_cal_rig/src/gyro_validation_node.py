#!/usr/bin/env python3

import os
import time
from threading import Lock
import rclpy
from rclpy.executors import MultiThreadedExecutor
import rclpy.action
from rclpy.action.server import ActionServer
from gyro_cal_rig_msgs.action import CalibrateGyro
from procedure_node import GyroProcedureNode, timeAsStr
from data_logger import DataLogger

LOG_COLUMNS = ["sec", "nanosec", "gyro_temp", "rig_rate", "rig_heating", "rig_enabled", "rig_stalled", "expected_yaw", "actual_yaw"]

class GyroValidationNode(GyroProcedureNode):
    def __init__(self):
        super().__init__("gyro_validation_node", self.validateCb)
        self.lock = Lock()
        

    def validateCb(self, handle):
        self.get_logger().info("PERFORMING VALIDATION")
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

        datetimeStr = timeAsStr()
        calLogPath = os.path.join(logDir, f"{datetimeStr}_validation.csv")
        
        with self.lock:
            self.valLogger = DataLogger(calLogPath, LOG_COLUMNS)
        
        self._calInProgress = True
        
        try:
            for i in range(0, len(request.temps)):
                while not self.rigAtTemp(request.temps[i]) and request.temps[i] != 0 and not handle.is_cancel_requested:
                    self.collectPositions(i, handle)
                
                if not handle.is_cancel_requested:
                    self.collectPositions(i, handle)
        except RuntimeError as ex:
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
            
            self.sendRigCommand(rates[i], self.rigShouldHeat(handle.request.temps[tempIdx]))
            time.sleep(handle.request.seconds_per_rate)
            self.sendRigCommand(0, self.rigShouldHeat(handle.request.temps[tempIdx]))
            time.sleep(handle.request.seconds_per_rate)
            
            self.sendRigCommand(-1 * rates[i], self.rigShouldHeat(handle.request.temps[tempIdx]))
            time.sleep(handle.request.seconds_per_rate)
            self.sendRigCommand(0, self.rigShouldHeat(handle.request.temps[tempIdx]))
            time.sleep(handle.request.seconds_per_rate)
            
            if handle.is_cancel_requested:
                self.get_logger().info("Preempting validation")
                return False

            return True


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