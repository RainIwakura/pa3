#!/usr/bin/env python

import gym
import numpy
import time
import qlearn
import doubleqlearn
from gym import wrappers
import rospy
import rospkg
from robotx_gazebo.msg import UsvDrive
from nav_msgs.msg import Odometry



# import our training environment


class DiffDrive():
    def __init__(self):
        self.goal_state = (rospy.get_param('/wamv/desired_point/x'), rospy.get_param('/wamv/desired_point/x'))
        rospy.init_node("wamv_diff")


        self.current_coord = None
        self.cmd_drive_data = None
        self.measurement_data = None
        rospy.Subscriber("/cmd_drive", UsvDrive, self.cmd_drive_callback)
        rospy.Subscriber("/wamv/odom", Odometry, self.diff_drive_callback)


        self.publisher = rospy.Publisher("/cmd_drive", UsvDrive, queue_size=10)

        rospy.spin()



    def measurement_callback(self, measurement):
        self.measurement_data = (measurement.longitude, measurement.latitude)
        

    def get_xyz(self):
        xy = self.current_coord.pose.pose.position
        angle_with_respect_to_axis =  self.current_coord.pose.pose.orientation

        return (xy.x, xy.y, angle_with_respect_to_axis.z)


    def cmd_drive_callback(self, data):
        self.cmd_drive_data = data


    def diff_drive_callback(self):










