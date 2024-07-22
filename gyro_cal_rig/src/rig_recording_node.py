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

LOG_COLUMNS = ["sec", "nanosec", "rig_rate"]
RATE_STEP_SIZE = 37786

class RigRecordingNode(GyroProcedureNode):
    def __init__(self):
        super().__init__("rig_recording_node", self.rigRecordCb)
        self.lock = Lock()
        
        #subscriptions
        self._rigStatusSub = self.create_subscription(GyroRigStatus, RIG_STATUS_TOPIC, self.rigStatusCb, 10)
        self._rigStatus = GyroRigStatus()
        
        self.get_logger().info("Rig recording node ready")
        

    def rigRecordCb(self, handle):
        result = CalibrateGyro.Result()
        if self._calInProgress:
            handle.abort()
            return result

        request = handle.request
        logDir = request.log_directory
        self.get_logger().info("RECORDING RIG STATUS\n")
        
        try:
            datetimeStr = self.timeAsStr()
            recLogPath = os.path.join(logDir, f"{datetimeStr}_recording.csv")
            
            with self.lock:
                self.recLogger = DataLogger(recLogPath, LOG_COLUMNS)
        except BaseException as ex:
            self.stopRig()
            self._calInProgress = False
            result.result = f"ERROR: {ex}"
            self.get_logger().error(f"ERROR: {ex}")
            handle.abort()
            return result
                
        self._calInProgress = True
        
        while not handle.is_cancel_requested:
            time.sleep(0.1)
            
        handle.canceled()
        self._calInProgress = False
        
        return result

    
    def updateLog(self):
        now = self.get_clock().now()
        
        if self._calInProgress:
            with self.lock:
                #["sec", "nanosec", "rig_rate"]
                self.recLogger.logData(
                    [
                        now.to_msg().sec, 
                        now.to_msg().nanosec,
                        self._rigStatus.rate,
                    ]
                )
    
    
    def rigStatusCb(self, msg: GyroRigStatus):
        #update expected yaw
        self._rigStatus = msg
        self.updateLog()


def main(args = None):
    rclpy.init(args = args)
    node = RigRecordingNode()
    rclpy.spin(node, executor=MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
