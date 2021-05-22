from time import sleep, time
from math import pi, sin, cos
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion

class WheelOdom(Node):
    def __init__(self):
        super().__init__('wheel_odom')
        self.create_timer(0.1, self.cb)
        self.odom_broadcaster = TransformBroadcaster(self)
        print('fake odom')
    def cb(self):
        now = self.get_clock().now()
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = 0.0
        quaternion.w = 1.0
        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = now.to_msg()
        transform_stamped_msg.header.frame_id = 'odom'
        transform_stamped_msg.child_frame_id = 'base_link'
        transform_stamped_msg.transform.translation.x = 0.0
        transform_stamped_msg.transform.translation.y = 0.0
        transform_stamped_msg.transform.translation.z = 0.0
        transform_stamped_msg.transform.rotation.x = quaternion.x
        transform_stamped_msg.transform.rotation.y = quaternion.y
        transform_stamped_msg.transform.rotation.z = quaternion.z
        transform_stamped_msg.transform.rotation.w = quaternion.w
        self.odom_broadcaster.sendTransform(transform_stamped_msg)
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = 0.0
        quaternion.w = 1.0
        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = now.to_msg()
        transform_stamped_msg.header.frame_id = 'map'
        transform_stamped_msg.child_frame_id = 'odom'
        transform_stamped_msg.transform.translation.x = 0.0
        transform_stamped_msg.transform.translation.y = 0.0
        transform_stamped_msg.transform.translation.z = 0.0
        transform_stamped_msg.transform.rotation.x = quaternion.x
        transform_stamped_msg.transform.rotation.y = quaternion.y
        transform_stamped_msg.transform.rotation.z = quaternion.z
        transform_stamped_msg.transform.rotation.w = quaternion.w
        self.odom_broadcaster.sendTransform(transform_stamped_msg)
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = 0.0
        quaternion.w = 1.0
        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = now.to_msg()
        transform_stamped_msg.header.frame_id = 'base_link'
        transform_stamped_msg.child_frame_id = 'camera_link'
        transform_stamped_msg.transform.translation.x = 0.0
        transform_stamped_msg.transform.translation.y = 0.0
        transform_stamped_msg.transform.translation.z = 0.4
        transform_stamped_msg.transform.rotation.x = quaternion.x
        transform_stamped_msg.transform.rotation.y = quaternion.y
        transform_stamped_msg.transform.rotation.z = quaternion.z
        transform_stamped_msg.transform.rotation.w = quaternion.w
        self.odom_broadcaster.sendTransform(transform_stamped_msg)


def main():
    rclpy.init()
    odom = WheelOdom()
    rclpy.spin(odom)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
