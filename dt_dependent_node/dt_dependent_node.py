#!/usr/bin/env python

import rospy
import numpy as np

from duckietown_msgs.msg import WheelsCmdStamped

def continuous_publisher():
    vel_pub = rospy.Publisher('/default/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
    rospy.init_node('continuous_test_publisher')
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = WheelsCmdStamped()
        msg.vel_left = 1.0
        msg.vel_right = 1.0

        vel_pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    continuous_publisher()
