import logging            
from time import sleep, time
from math import pi, sin, cos
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from ChassisInterface import ChassisInterface

class ChassisNode(Node):
    CameraElevationMeters = 0.4
    WheelBaselineMeters = 0.430
    WheelRadiusMeters = 0.115
    ReportIntervalSec = 0.02
    SecInNsec = 10**-9
    RadPerSecInWheelFeedback = 1.

    MaxSpeedMetersSec = 1.0 # linear velocity from vel_cmd is scaled and bounded to this
    MaxYawRateRadSec = 1.0  # yaw rate from vel_cmd is scaled and bounded to this. todo : set to actual max yaw rate of chassis

    def __init__(self):
        super().__init__('chassis')

        self.odom_pub = self.create_publisher(Odometry, "odom_dirty", 10)
        self.odom_broadcaster = TransformBroadcaster(self)
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.Vx = 0
        self.yawRate = 0

        self.cmdSpeed = 0
        self.cmdSteering = 0

        self.tsFeedback = self.get_clock().now().nanoseconds
        self.chassis = ChassisInterface()
        self.chassis.setWheelCallback(self.processWheelFeedback)

        self.sub = self.create_subscription(Twist, 'cmd_vel', self.processCmd, 10)
        logging.info('chassis node initialized')

    def processWheelFeedback(self, DeviceIdIgnored, respTuple):
        now = self.get_clock().now()
        deltaTimeSec = (now.nanoseconds - self.tsFeedback) * ChassisNode.SecInNsec

        zeroRotation = Quaternion()
        zeroTranslation = Vector3()
        zeroTranslation.x = 0.0
        zeroTranslation.y = 0.0
        zeroTranslation.z = 0.0

        if deltaTimeSec >= ChassisNode.ReportIntervalSec:
            self.tsFeedback = now.nanoseconds
            wheelLRadSec = respTuple[0] * ChassisNode.RadPerSecInWheelFeedback
            wheelRRadSec = respTuple[1] * ChassisNode.RadPerSecInWheelFeedback
            self.calculatePose(deltaTimeSec, wheelRRadSec, wheelLRadSec)

            logging.debug('chassis wheel feedback : {0} {1} speed {2} yawRate {3} pos [{4} {5}] yaw {6}'.format(
                wheelLRadSec, wheelRRadSec, self.Vx, self.yawRate, self.x, self.y, self.yaw))

            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.yaw / 2)
            quaternion.w = cos(self.yaw / 2)

            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.covariance[0] = 0.1
            odom.pose.covariance[7] = 0.1
            odom.pose.covariance[35] = 0.1
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = self.Vx
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = self.yawRate
            odom.twist.covariance[0] = 0.01
            odom.twist.covariance[7] = 0.01
            odom.twist.covariance[35] = 0.01
            self.odom_pub.publish(odom)

            transform_stamped_msg = TransformStamped()
            transform_stamped_msg.header.stamp = now.to_msg()
            transform_stamped_msg.header.frame_id = 'odom'
            transform_stamped_msg.child_frame_id = 'base_link'
            transform_stamped_msg.transform.translation = zeroTranslation
            transform_stamped_msg.transform.translation.x = self.x
            transform_stamped_msg.transform.translation.y = self.y
            transform_stamped_msg.transform.rotation = quaternion
            self.odom_broadcaster.sendTransform(transform_stamped_msg)

            transform_stamped_msg = TransformStamped()
            transform_stamped_msg.header.stamp = now.to_msg()
            transform_stamped_msg.header.frame_id = 'map'
            transform_stamped_msg.child_frame_id = 'odom'
            transform_stamped_msg.transform.translation = zeroTranslation
            transform_stamped_msg.transform.rotation = zeroRotation
            self.odom_broadcaster.sendTransform(transform_stamped_msg)

            transform_stamped_msg = TransformStamped()
            transform_stamped_msg.header.stamp = now.to_msg()
            transform_stamped_msg.header.frame_id = 'base_link'
            transform_stamped_msg.child_frame_id = 'camera_link'
            transform_stamped_msg.transform.translation = zeroTranslation
            transform_stamped_msg.transform.translation.z = ChassisNode.CameraElevationMeters
            self.odom_broadcaster.sendTransform(transform_stamped_msg)

    def processCmd(self, data):
        maxSpeed = ChassisNode.MaxSpeedMetersSec
        maxYawRate = ChassisNode.MaxYawRateRadSec

        #todo : add deadzone for Vx and yaw rate
        #todo : reset chassis control to zeroes if cmd_vel has expired
        cmdSpeedBounded = max(min(data.linear.x, maxSpeed), -maxSpeed)  
        self.cmdSpeed = cmdSpeedBounded / maxSpeed
        self.chassis.setSpeed(self.cmdSpeed)

        cmdYawRateBounded = max(min(data.angular.z, maxYawRate), -maxYawRate)
        self.cmdSteering = cmdYawRateBounded / maxYawRate
        self.chassis.setSteering(self.cmdSteering)
        logging.debug('chassis cmd speed {0}/{1} steering {2}/{3}'.format(
            data.linear.x, self.cmdSpeed, data.angular.z, self.cmdSteering))

    #dtSec : integration interval
    #Lvel, Rvel : wheels angular speed, rad/sec
    def calculatePose(self, dtSec, Lvel, Rvel):
        # Lvel= -Lvel
        self.Vx = (ChassisNode.WheelRadiusMeters)*(Rvel+Lvel)/2
        self.yawRate = -1.6*(ChassisNode.WheelRadiusMeters)*(Rvel-Lvel)/ChassisNode.WheelBaselineMeters
        self.yaw += dtSec * self.yawRate
        self.y += dtSec * sin(self.yaw) * self.Vx
        self.x += dtSec * cos(self.yaw) * self.Vx

def main():
    rclpy.init()
    odom = ChassisNode()
    rclpy.spin(odom)
    rclpy.shutdown()


if __name__ == '__main__':
    main()