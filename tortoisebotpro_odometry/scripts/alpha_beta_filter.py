#!/usr/bin/env python

"""
Note :  This script only filters the x and y received from ekf on topic /fused_odom/ekf.
        This script publishes only the x and y values of position, not the quaternions, not the twists (needs to be added)
        The tf published contained contains all the info.
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class Filter():
    def __init__(self):
        rospy.init_node('alpha_beta_filter', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz

        # self.filter_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.filter_pub = rospy.Publisher('filtered_odom/alpha_beta', Odometry, queue_size=10)

        rospy.Subscriber("fused_odom/ekf", Odometry, self.efk_callback)
        rospy.Subscriber("cmd_vel", Twist, self.vel_callback)

        self.odom_broadcaster = TransformBroadcaster()

        # Sensor measurement pulbication rate
        fused_pub_rate = 7.5 # Update to parameter
        self.del_t = 1/fused_pub_rate

        self.fused_odom = Pose() # sensor measurement from ekf
        
        self.vel = Twist() # sensor memasurement from cmd_vel
        self.vel.linear.x = 0
        self.vel.linear.y = 0

        self.filter_odom = Odometry()
        self.filter_odom.header.frame_id = "odom"
        self.filter_odom.child_frame_id = "base_link"

        self.transform_stamped_msg = TransformStamped()
        self.transform_stamped_msg.header.frame_id = "odom"
        self.transform_stamped_msg.child_frame_id = "base_link"

        # Init ?
        self.fused_odom.orientation.x = 0
        self.fused_odom.orientation.y = 0
        self.fused_odom.orientation.z = 0
        self.fused_odom.orientation.w = 1

        self.x_e_prev = 0
        self.vx_e_prev = 0
        
        self.y_e_prev = 0
        self.vy_e_prev = 0

        self.alpha = 0.3
        self.beta = 0.4


    def update(self):

        # ---- position in x ---- #

        x_p = self.x_e_prev + self.del_t * self.vx_e_prev
        vx_p = self.vx_e_prev

        x_e = x_p + self.alpha * (self.fused_odom.position.x - x_p)
        # vx_e = vx_p + self.beta * (self.vel.linear.x - vx_p)          # Use velocity from cmd vel
        vx_e = vx_p + self.beta * (self.fused_odom.position.x - x_p) / self.del_t # Use velocity from lidar odometry

        self.x_e_prev = x_e
        self.vx_e_prev = vx_e


        # ---- position in y ---- #

        y_p = self.y_e_prev + self.del_t * self.vy_e_prev
        vy_p = self.vy_e_prev

        y_e = y_p + self.alpha * (self.fused_odom.position.y - y_p)
        # vy_e = vy_p + self.beta * (self.vel.linear.y - vy_p)
        vy_e = vy_p + self.beta * (self.fused_odom.position.y - y_p) / self.del_t

        self.y_e_prev = y_e
        self.vy_e_prev = vy_e

        # ---- odom ---- #

        self.filter_odom.header.stamp = rospy.Time.now()
        self.filter_odom.pose.pose.position.x = x_e
        self.filter_odom.pose.pose.position.y = y_e
        self.filter_odom.pose.pose.position.z = 0.0
        self.filter_odom.pose.pose.orientation.x = self.fused_odom.orientation.x
        self.filter_odom.pose.pose.orientation.y = self.fused_odom.orientation.y
        self.filter_odom.pose.pose.orientation.z = self.fused_odom.orientation.z
        self.filter_odom.pose.pose.orientation.w = self.fused_odom.orientation.w
        self.filter_odom.twist.twist.linear.x = vx_e
        self.filter_odom.twist.twist.linear.y = vy_e
        self.filter_odom.twist.twist.linear.z = 0.0
        self.filter_odom.twist.twist.angular.x = 0.0
        self.filter_odom.twist.twist.angular.y = 0.0
        self.filter_odom.twist.twist.angular.z = 0.0 # Subscribe to fused odom and pub the same


        # ---- Transforms ---- #

        self.transform_stamped_msg.header.stamp = rospy.Time.now()
        self.transform_stamped_msg.transform.translation.x = x_e
        self.transform_stamped_msg.transform.translation.y = y_e
        self.transform_stamped_msg.transform.translation.z = 0.0
        self.transform_stamped_msg.transform.rotation.x = self.fused_odom.orientation.x
        self.transform_stamped_msg.transform.rotation.y = self.fused_odom.orientation.y
        self.transform_stamped_msg.transform.rotation.z = self.fused_odom.orientation.z
        self.transform_stamped_msg.transform.rotation.w = self.fused_odom.orientation.w


        # ---- Meta ---- #
        
        # rospy.loginfo_throttle(0.1, "OK")

        self.filter_pub.publish(self.filter_odom)
        self.odom_broadcaster.sendTransform(self.transform_stamped_msg)

    def efk_callback(self, msg):
        self.fused_odom = msg.pose.pose
        
    def vel_callback(self, msg):
        self.vel = msg

if __name__ == '__main__':
    try:
        filter = Filter()
        while not rospy.is_shutdown():
            filter.update()
            filter.rate.sleep()
    except rospy.ROSInterruptException:
        rospy.logfatal("Something is fissy")