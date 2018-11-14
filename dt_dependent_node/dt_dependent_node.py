#!/usr/bin/env python

import rospy
import numpy as np

from duckietown_msgs.msg import WheelsCmdStamped, LanePose


class lane_controller(object):

    def __init__(self):
        self.lane_pos_obj = 0.2
        self.det_err_threshold = 0.2
        self.det_err_slowdown = 3.0
        self.corr_d = 3.0
        self.corr_phi = 0.5
        self.last_d = 0.0
        self.last_phi = 0.0
        self.vel_right = 0.0
        self.vel_left = 0.0
        self.vel_cmd = WheelsCmdStamped()
        self.vel_pub = rospy.Publisher('/default/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        self.lane_reading = rospy.Subscriber("/default/lane_filter_node/lane_pose", LanePose, self.PoseHandling, queue_size=1)

    def PoseHandling(self,lane_pose):
        if lane_pose.phi > 0.:
            self.vel_left = 1.0
            self.vel_right = 1.0 - lane_pose.phi*self.corr_phi
        elif lane_pose.phi < 0.:
            self.vel_left = 1.0 + lane_pose.phi*self.corr_phi
            self.vel_right = 1.0
        else:
            self.vel_left = 1.0
            self.vel_right = 1.0

        if lane_pose.d > self.lane_pos_obj:
            self.vel_right-=self.corr_d*(lane_pose.d-self.lane_pos_obj)
        else:
            self.vel_left +=self.corr_d*(lane_pose.d-self.lane_pos_obj)

        if (np.abs(self.last_d - lane_pose.d) > self.det_err_threshold) or (np.abs(self.last_phi - lane_pose.phi) > self.det_err_threshold):
            self.vel_left = self.vel_left/self.det_err_slowdown
            self.vel_right = self.vel_right/self.det_err_slowdown

        self.last_d = lane_pose.d
        self.last_phi = lane_pose.phi


    def PubVel(self):
        self.vel_cmd.vel_left = self.vel_left
        self.vel_cmd.vel_right = self.vel_right
        self.vel_pub.publish(self.vel_cmd)


def continuous_publisher():
    rospy.init_node('continuous_test_publisher')
    lane_control_node = lane_controller()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        lane_control_node.PubVel()


if __name__ == '__main__':
    continuous_publisher()
