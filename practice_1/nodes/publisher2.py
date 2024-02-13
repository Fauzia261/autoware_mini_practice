#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
message = rospy.get_param('~message', 'Hi!')
rospy.init_node('publisher')
rate = rospy.Rate(4)
pub = rospy.Publisher('/message', String, queue_size=10)

while not rospy.is_shutdown():
    pub.publish(message)
    rate.sleep()
