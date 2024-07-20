import datetime
import rclpy
import time
from threading import Lock
from rclpy.node import Node
from rclpy.action import ActionServer
from gyro_cal_rig_msgs.action import CalibrateGyro
from gyro_cal_rig_msgs.msg import GyroRigStatus
from riptide_msgs2.msg import Int32Stamped, GyroStatus
from std_msgs.msg import Bool

RIG_CALIBRATE_ACTION = "cal_rig/run"
RIG_COMMAND_TOPIC = "command/cal_rig"
RIG_STATUS_TOPIC = "status/cal_rig"
RIG_CALIBRATING_TOPIC = "status/cal_rig/calibrating"
GYRO_RAW_TOPIC = "gyro/raw"
GYRO_STATUS_TOPIC = "gyro/status"


class GyroProcedureNode(Node):
    def __init__(self, nodeName, calFunc):
        super().__init__(nodeName)
        self.get_logger().info("Gyro calibrator starting")
        
        #action server
        self._action_server = ActionServer(
            self,
            CalibrateGyro,
            RIG_CALIBRATE_ACTION,
            goal_callback=self.calGoalCb,
            cancel_callback=self.calCancelCb,
            execute_callback=calFunc)
        
        #parameters
        self.declare_parameter("steps_per_revolution", 0.0)
        self._stepsPerRev = self.get_parameter("steps_per_revolution").value
        if self._stepsPerRev == 0:
            self.get_logger().error("Potential error loading parameters, steps per rev is 0!")

        #publishers
        self._rigCommandPub = self.create_publisher(GyroRigStatus, RIG_COMMAND_TOPIC, 10)
        self._calibratingPub = self.create_publisher(Bool, RIG_CALIBRATING_TOPIC, 10) 
        
        #timers
        self._timer = self.create_timer(1, self.publishCalibrating)
        
        #sticky msgs
        self._rigStatus = GyroRigStatus()
        self._gyroRaw = Int32Stamped()
        self._gyroStatus = GyroStatus()
        
        #state
        self._calInProgress = False #true if calibration data is being collected
        self.logDir = ""
        
        #handle parameters
        self.declare_parameter("allowable_temp_error", 500)
        self.declare_parameter("rig_ack_timeout", 3.0)
        self.add_on_set_parameters_callback(self.readParameters)
        self.readParameters()

        self.get_logger().info("Gyro calibration action ready")
    
    def timeAsStr(self):
        return datetime.datetime.now().strftime("%m-%d-%Y_%H-%M-%S")

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

    def rigAtTemp(self, setpt):
        return \
            self._gyroStatus.temperature > setpt - self._tempRange and \
            self._gyroStatus.temperature < setpt + self._tempRange
    
    def rigShouldHeat(self, setpt):
        if self._rigStatus.heat:
            return self._gyroStatus.temperature < setpt - self._tempRange + 10
        
        return self._gyroStatus.temperature < setpt - self._tempRange
    
    def sendRigCommand(self, rate, heat):
        #set rig
        desiredStatus = GyroRigStatus()
        desiredStatus.rate = rate
        desiredStatus.heat = heat
        desiredStatus.enabled = True
        self._rigCommandPub.publish(desiredStatus)
        
        #wait for rig to acknowledge
        ackStartTime = self.get_clock().now()
        while self.get_clock().now().seconds_nanoseconds()[0] - ackStartTime.seconds_nanoseconds()[0] < self._ackTimeout:
            self._rigCommandPub.publish(desiredStatus)
            time.sleep(0.1)
            if self._rigStatus.rate == desiredStatus.rate \
            and self._rigStatus.heat == desiredStatus.heat \
            and self._rigStatus.enabled == desiredStatus.enabled:
                break
        
        if self.get_clock().now().seconds_nanoseconds()[0] - ackStartTime.seconds_nanoseconds()[0] >= self._ackTimeout:
            raise RuntimeError("Could not send command to rig!")
