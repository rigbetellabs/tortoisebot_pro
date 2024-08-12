#!/usr/bin/env python3

import rospy
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int32

NS_TO_SEC = 1000000000

class DiffTf(object):
    def __init__(self):
        rospy.init_node("wheel_odom")
        self.nodename = "wheel_odom"
        rospy.loginfo(f"-I- {self.nodename} started")

        self.rate_hz = rospy.get_param("~rate_hz", 100.0)
        self.rate = rospy.Rate(self.rate_hz)

        self.ticks_meter = float(rospy.get_param('~ticks_meter', 2055)) # 8-1-24
        self.base_width = float(rospy.get_param('~base_width', 0.200))  # 8-1-24

        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')

        self.encoder_min = rospy.get_param('~encoder_min', -32768)
        self.encoder_max = rospy.get_param('~encoder_max', 32768)
        self.encoder_low_wrap = rospy.get_param('~encoder_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min)
        self.encoder_high_wrap = rospy.get_param('~encoder_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min)

        self.enc_left = None
        self.enc_right = None
        self.left = 0.0
        self.right = 0.0
        self.lmult = 0.0
        self.rmult = 0.0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.dx = 0.0
        self.dr = 0.0
        self.then = rospy.Time.now()

        self.left_sub = rospy.Subscriber("left_ticks", Int32, self.lwheel_callback, queue_size=10)
        self.right_sub = rospy.Subscriber("right_ticks", Int32, self.rwheel_callback, queue_size=10)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
       # self.odom_broadcaster = TransformBroadcaster()

    def update(self):
        now = rospy.Time.now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.to_sec()

        if self.enc_left is None:
            d_left = 0
            d_right = 0
        else:
            d_left = (self.left - self.enc_left) / self.ticks_meter
            d_right = (self.right - self.enc_right) / self.ticks_meter
        self.enc_left = self.left
        self.enc_right = self.right

        d = (d_left + d_right) / 2
        th = (d_right - d_left) / self.base_width
        self.dx = d / elapsed
        self.dr = th / elapsed

        if d != 0:
            x = cos(th) * d
            y = -sin(th) * d
            self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
            self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
        if th != 0:
            self.th = self.th + th

        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2)
        quaternion.w = cos(self.th / 2)

        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = now
        transform_stamped_msg.header.frame_id = self.odom_frame_id
        transform_stamped_msg.child_frame_id = self.base_frame_id
        transform_stamped_msg.transform.translation.x = self.x
        transform_stamped_msg.transform.translation.y = self.y
        transform_stamped_msg.transform.translation.z = 0.0
        transform_stamped_msg.transform.rotation.x = quaternion.x
        transform_stamped_msg.transform.rotation.y = quaternion.y
        transform_stamped_msg.transform.rotation.z = quaternion.z
        transform_stamped_msg.transform.rotation.w = quaternion.w

       # self.odom_broadcaster.sendTransform(transform_stamped_msg)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.dr
        self.odom_pub.publish(odom)

    def lwheel_callback(self, msg):
        enc = msg.data

       # rospy.loginfo("left_ticks %d", enc)
        if enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap:
            self.lmult = self.lmult + 1

        if enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap:
            self.lmult = self.lmult - 1

        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min))
        self.prev_lencoder = enc

    def rwheel_callback(self, msg):
        enc = msg.data
        #rospy.loginfo("right_ticks %d", enc)
        if enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap:
            self.rmult = self.rmult + 1

        if enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap:
            self.rmult = self.rmult - 1

        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc

def main():
    diff_tf = DiffTf()
    while not rospy.is_shutdown():
        diff_tf.update()
        diff_tf.rate.sleep()

if __name__ == '__main__':
    main()
