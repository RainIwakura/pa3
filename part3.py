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
        


