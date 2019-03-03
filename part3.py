#!/usr/bin/env python

import gym
import numpy
import time
import qlearn
import doubleqlearn
import rospy
import rospkg
import math


from robotx_gazebo.msg import UsvDrive
from nav_msgs.msg import Odometry
from gym import wrappers
from simulated_sensor.msg import Measurement

# import our training environment


class DiffDrive():
    def __init__(self):
        self.goal_state = (rospy.get_param('/wamv/desired_point/x'), rospy.get_param('/wamv/desired_point/x'))
        rospy.init_node("wamv_diff")


        self.current_coord = None
        self.cmd_drive_data = None
        self.measurement_data = None
        rospy.Subscriber("/cmd_drive", UsvDrive, self.cmd_drive_callback)
        rospy.Subscriber("/simulated_sensor/raw", Measurement, self.measurement_callback)
        rospy.Subscriber("/wamv/odom", Odometry, self.diff_drive_callback)


        self.publisher = rospy.Publisher("/cmd_drive", UsvDrive, queue_size=10)

        rospy.spin()



    def measurement_callback(self, measurement):
        self.measurement_data = (measurement.longitude, measurement.latitude)
        print('print measurement data ', self.measurement_data)
        

    def get_xyz(self):

        xy = self.current_coord.pose.pose.position
        angle_with_respect_to_axis =  self.current_coord.pose.pose.orientation


        print('Odometry data ', xy.x, xy.y, angle_with_respect_to_axis.z)

        return (xy.x, xy.y, angle_with_respect_to_axis.z)


    def cmd_drive_callback(self, data):
        self.cmd_drive_data = data


    def diff_drive_callback(self, inpt):
        self.current_coord = inpt


        self.control_robot()



    def control_robot(self):
        x,y,theta = sefl.get_xyz()

        goal = self.goal_state()
        gx = goal[0]
        gy = goal[1]



        delta_y = gy - y
        delta_x = gx - x
        angle_with_respect_to_axis = math.atan2(delta_y/delta_x)


        new_theta = angle_with_respect_to_axis - theta


        if new_theta > 0:
            self.publisher.publish(UsvDrive(right=1, left=0))
        elif new_theta < 0:
            self.publisher.publish(UsvDrive(right=0, left=1))
        elif math.abs(new_theta) < 0.01:
            self.publisher.publish(UsvDrive(right=1, left=1))














