#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

message = rospy.get_param('~message')
rate = rospy.get_param('~rate')

rospy.init_node('publisher')
rate = rospy.Rate(rate)
pub = rospy.Publisher('/message', String, queue_size=10)

while not rospy.is_shutdown():
    pub.publish(message)
    rate.sleep()
