#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer
from gyro_cal_rig_msgs.action import CalibrateGyro
from gyro_cal_rig_msgs.msg import GyroRigStatus

RIG_COMMAND_TOPIC = "command/cal_rig"
RIG_STATUS_TOPIC = "status/cal_rig"

class GyroCalibrationNode(Node):
    def __init__(self):
        super().__init__("gyro_calibration_node")
        self.get_logger().info("Gyro calibrator starting")
        
        self._action_server = ActionServer(
            self,
            CalibrateGyro,
            'cal_rig/calibrate',
            self.doCalibration)

        self._rigCommandPub = self.create_publisher(GyroRigStatus, RIG_COMMAND_TOPIC, 10)
        self._rigStatusSub = self.create_subscription(GyroRigStatus, RIG_STATUS_TOPIC, self.rigStatusCb, 10)
        self._rigStatus = GyroRigStatus()
        
        #TODO: make these parameters
        self._delayTime = 10
        self._tempRange = 500
        self._ackTimeout = 3

        self.get_logger().info("Gyro calibration action ready")
    
    
    def doCalibration(self, goal_handle):
        request = goal_handle.request
        self.get_logger().info(
            "PERFORMING CALIBRATION\n" + 
            "Calibration details:\n" + 
            f"  rates: {request.rates}" +
            f"  temps: {request.temps}")

        result = CalibrateGyro.Result()

        try:
            for i in range(0, len(request.temps)):
                #try to collect rates while reaching temp
                while not self.rigAtTemp(request.temps[i]):
                    self.collectRates(i, goal_handle)
                        
                #now at temp, collect rates one more time
                self.collectRates(i, goal_handle)
        except RuntimeError as ex:
            result.result = f"ERROR: {ex}"
            goal_handle.abort()
            return result
        
        result.result = "Calibration successful."
        goal_handle.succeed()
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
            desiredStatus.rate = int(rates[i])
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
            time.sleep(self._delayTime)
            
        return True
    
    #TODO implement these
    def rigAtTemp(self, setpt):
        return True
    
    def rigShouldHeat(self, setpt):
        return False

    def rigStatusCb(self, msg):
        self._rigStatus = msg


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
