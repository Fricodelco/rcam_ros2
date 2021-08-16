#!/usr/bin/python3
# -*- coding: utf-8 -*-

from time import sleep, time
from math import pi, sin, cos
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from rclpy.parameter import Parameter


class WheelOdom(Node):
    def __init__(self):
        super().__init__('wheel_odom')
        simTime = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([simTime])
        self.create_timer(0.1, self.cb)
        self.odom_broadcaster = TransformBroadcaster(self)
        print('fake odom')
    
    def cb(self):
        now = self.get_clock().now()
        q = self.quaternion_from_euler(1.57,3.14,1.57)
        quaternion = Quaternion()
        quaternion.x = q[1]
        quaternion.y = q[2]
        quaternion.z = q[3]
        quaternion.w = q[0]
        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = now.to_msg()
        transform_stamped_msg.header.frame_id = 'base_link'
        transform_stamped_msg.child_frame_id = 'camera_link'
        transform_stamped_msg.transform.translation.x = 0.5
        transform_stamped_msg.transform.translation.y = 0.0
        transform_stamped_msg.transform.translation.z = 0.3
        transform_stamped_msg.transform.rotation.x = quaternion.x
        transform_stamped_msg.transform.rotation.y = quaternion.y
        transform_stamped_msg.transform.rotation.z = quaternion.z
        transform_stamped_msg.transform.rotation.w = quaternion.w
        self.odom_broadcaster.sendTransform(transform_stamped_msg)
        transform_stamped_msg.header.frame_id = 'map'
        transform_stamped_msg.child_frame_id = 'odom'
        transform_stamped_msg.transform.translation.x = 0.0
        transform_stamped_msg.transform.translation.y = 0.0
        transform_stamped_msg.transform.translation.z = 0.0
        transform_stamped_msg.transform.rotation.x = 0.0
        transform_stamped_msg.transform.rotation.y = 0.0
        transform_stamped_msg.transform.rotation.z = 0.0
        transform_stamped_msg.transform.rotation.w = 1.0
        self.odom_broadcaster.sendTransform(transform_stamped_msg)
        
    def quaternion_from_euler(self, roll, pitch, yaw):    
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)
        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr
        return q


def main():
    rclpy.init()
    odom = WheelOdom()
    rclpy.spin(odom)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
