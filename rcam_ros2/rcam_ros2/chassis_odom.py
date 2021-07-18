import logging            
from time import sleep, time
from math import pi, sin, cos
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from ChassisInterface import ChassisInterface

class ChassisOdom(Node):
    WheelBaselineMeters = 0.430
    WheelRadiusMeters = 0.115
    ReportIntervalSec = 0.02
    SecInNsec = 10**-9
    RadPerSecInWheelFeedback = 1.    

    def __init__(self):
        super().__init__('chassis_odom')

        self.odom_pub = self.create_publisher(Odometry, "odom_dirty", 10)
        self.odom_broadcaster = TransformBroadcaster(self)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.timeLast = self.get_clock().now().nanoseconds
        self.chassis = ChassisInterface()
        self.chassis.setWheelCallback(self.processWheelFeedback)
        logging.info('chassis_odom initialized')

    def processWheelFeedback(self, deviceId, respTuple):
        now = self.get_clock().now()
        deltaTimeSec = (now.nanoseconds - self.timeLast) * ChassisOdom.SecInNsec

        zeroRotation = Quaternion()
        zeroTranslation = Vector3()
        zeroTranslation.x = 0.0
        zeroTranslation.y = 0.0
        zeroTranslation.z = 0.0

        if deltaTimeSec >= ChassisOdom.ReportIntervalSec:
            self.timeLast = now.nanoseconds
            wheelLRadSec = respTuple[0] * ChassisOdom.RadPerSecInWheelFeedback;
            wheelRRadSec = respTuple[1] * ChassisOdom.RadPerSecInWheelFeedback;
            Vx, Vy, Vtheta = self.calculate_odom(deltaTimeSec, wheelRRadSec, wheelLRadSec)

            logging.debug('chassis_odom wheel feedback : {0} {1} speed {2} yawRate {3} pos [{4} {5}] yaw {6}'.format(
                wheelLRadSec, wheelRRadSec, Vx, Vtheta, self.x, self.y, self.theta));

            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.theta / 2)
            quaternion.w = cos(self.theta / 2)

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
            odom.twist.twist.linear.x = Vx
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = Vtheta
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
            transform_stamped_msg.transform.translation.z = 0.4
            self.odom_broadcaster.sendTransform(transform_stamped_msg)

    def calculate_odom(self, delta, Lvel, Rvel):
        # Lvel= -Lvel
        vel = (ChassisOdom.WheelRadiusMeters)*(Rvel+Lvel)/2
        Vx = vel
        Vy = 0

        # New theta for caclulationg rotation matrix:
        Vtheta = -1.6*(ChassisOdom.WheelRadiusMeters)*(Rvel-Lvel)/ChassisOdom.WheelBaselineMeters
        # Rotation matrix
        self.theta += delta * Vtheta
        self.y += delta * sin(self.theta)*Vx
        self.x += delta * cos(self.theta)*Vx
        return Vx, Vy, Vtheta


def main():
    rclpy.init()
    odom = ChassisOdom()
    rclpy.spin(odom)
    rclpy.shutdown()


if __name__ == '__main__':
    main()