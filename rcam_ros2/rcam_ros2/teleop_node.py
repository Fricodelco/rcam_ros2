#!/usr/bin/python3
# -*- coding: utf-8 -*-

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from threading import Thread


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', QoSProfile(depth=10))
        self.linearVelocity = 0.0
        self.angularVelocity = 0.0

    def setLinearVelocity(self, val):
        self.linearVelocity = val
        self._publish()

    def setAngularVelocity(self, val):
        self.angularVelocity = val
        self._publish()

    def _publish(self):
        twist = Twist()
        twist.linear.x = self.linearVelocity
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.angularVelocity

        self.pub.publish(twist)

class TeleopWrapper:
    def __init__(self):
        self.thread = Thread(target=self._threadFunc)
        self.thread.start()

    def _threadFunc(self):
        rclpy.init()
        self.node = TeleopNode()
        rclpy.spin(self.node)
        rclpy.shutdown()

    def setLinearVelocity(self, val):
        self.node.setLinearVelocity(val)

    def setAngularVelocity(self, val):
        self.node.setAngularVelocity(val)
