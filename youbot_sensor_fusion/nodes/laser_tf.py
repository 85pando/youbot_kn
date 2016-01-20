#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This code publishes a fixed transform in the center of the laserCloud produced by sensor_fusion.py

Copyright (C) 2015-2016 Stephan Heidinger
stephan.heidinger@uni-konstanz.de

This work is licensed under a Creative Commons Attribution-ShareAlike 3.0 Unported License.

See the full license text here: https://creativecommons.org/licenses/by-sa/3.0/legalcode
"""

import roslib
roslib.load_manifest('youbot_sensor_fusion')

import rospy
import tf

# default python boilerplate
if __name__ == '__main__':
  rospy.init_node('laserCloud_tf_broadcaster')
  br   = tf.TransformBroadcaster()
  rate = rospy.Rate(10.0)
  
  while not rospy.is_shutdown():
    # the combined cloud is at the center of the robot.
    # lengths in the transform are in m
    br.sendTransform((0.0, 0.0, 0.0), # translation x,y,z
                     (0.0, 0.0, 0.0, 1.0), # rotation x,y,z,w
                     rospy.Time.now(),
                     "laser_cloud", # new frame
                     "base_link")   # parent
    # front laser FIXME correct values when sensors are correctly attached
    br.sendTransform((0.0, 0.22, 0.0), # translation x,y,z
                     (0.0, 0.00, 0.0, 1.0), # rotation x,y,z,w
                     rospy.Time.now(),
                     "front_laser", # new frame
                     "laser_cloud")   # parent
    # right laser FIXME correct values when sensors are correctly attached
    br.sendTransform((0.11, 0.0, 0.0), # translation x,y,z
                     (0.00, 0.0, 0.0, 1.0), # rotation x,y,z,w
                     rospy.Time.now(),
                     "right_laser", # new frame
                     "laser_cloud")   # parent
    # left laser FIXME correct values when sensors are correctly attached
    br.sendTransform((-0.11, 0.0, 0.0), # translation x,y,z
                     ( 0.00, 0.0, 0.0, 1.0), # rotation x,y,z,w
                     rospy.Time.now(),
                     "left_laser", # new frame
                     "laser_cloud")   # parent
    rate.sleep()
    