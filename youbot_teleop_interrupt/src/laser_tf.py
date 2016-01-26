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
roslib.load_manifest('youbot_teleop_interrupt')

import rospy
import tf
from math import pi

class Displacement:
  """Encapsulates the translation from the /base_link"""
  def __init__(self, x=0, y=0, z=0, xrot=0, yrot=0, zrot=0):
    self.x = x
    self.y = y
    self.z = z
    self.xrot = xrot
    self.yrot = yrot
    self.zrot = zrot
    
  def getTranslation(self):
    return (self.x, self.y, self.z)
  
  def getRotationQuat(self):
    return tf.transformations.quaternion_from_euler(self.xrot, self.yrot, self.zrot)

# default python boilerplate
if __name__ == '__main__':
  rospy.init_node('laser_frame_broadcaster')
  br   = tf.TransformBroadcaster()
  rate = rospy.Rate(10.0)
  
  # variables for locations of the sensors
  sensorC = Displacement(zrot=-pi/2)
  frontL  = Displacement(x=.22)
  rightL  = Displacement(y=-.11, zrot=-pi/2)
  leftL   = Displacement(y= .11, zrot= pi/2)
  # end variables for locations of the sensors
  
  
  while not rospy.is_shutdown():
    # the combined cloud is at the center of the robot.
    # lengths in the transform are in m
    # rotations are in radian
    br.sendTransform(
                    #(0.0, 0.0, 0.0), # translation x,y,z
                    sensorC.getTranslation(),
                    #tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0), # rotation x,y,z,w
                    sensorC.getRotationQuat(),
                    rospy.Time.now(),
                    "sensor_cloud_frame", # new frame
                    "base_link")    # parent
    # front laser FIXME correct values when sensors are correctly attached
    #quat = quaternion_from_euler(0.0, 0.0, pi/2)
    br.sendTransform(
                    #(0.22, 0.0, 0.0), # translation x,y,z
                    frontL.getTranslation(),
                    #tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                    frontL.getRotationQuat(),
                    rospy.Time.now(),
                    "front_laser",    # new frame
                    "base_link")   # parent
    # right laser FIXME correct values when sensors are correctly attached
    br.sendTransform(
                    #(0.0, -0.11, 0.0), # translation x,y,z
                    rightL.getTranslation(),
                    #tf.transformations.quaternion_from_euler(0.0, 0.0, -pi/2), # rotation x,y,z,w
                    rightL.getRotationQuat(),
                    rospy.Time.now(),
                    "right_laser",    # new frame
                    "base_link")   # parent
    # left laser FIXME correct values when sensors are correctly attached
    br.sendTransform(
                    #(0.0, 0.11, 0.0), # translation x,y,z
                    leftL.getTranslation(),
                    #tf.transformations.quaternion_from_euler(0.0, 0.0, pi/2), # rotation x,y,z,w
                    leftL.getRotationQuat(),
                    rospy.Time.now(),
                    "left_laser",     # new frame
                    "base_link")   # parent
    rate.sleep()
    