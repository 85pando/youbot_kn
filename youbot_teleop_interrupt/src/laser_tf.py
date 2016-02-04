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
  sensorC = Displacement()
  # FIXME correct values when sensors are correctly attached
  frontL  = Displacement(x=.22)
  # FIXME correct values when sensors are correctly attached
  rightL  = Displacement(y=-.11, zrot=-pi/2)
  # FIXME correct values when sensors are correctly attached
  leftL   = Displacement(y= .11, zrot= pi/2)
  # FIXME correct values when sensors are correctly attached
  camera  = Displacement(x= .195, z= .145)
  # end variables for locations of the sensors
  
  
  while not rospy.is_shutdown():
    # the combined cloud is at the center of the robot.
    # lengths in the transform are in m
    # rotations are in radian
    br.sendTransform(
                    sensorC.getTranslation(),   # translation
                    sensorC.getRotationQuat(),  # rotation
                    rospy.Time.now(),           # timestamp
                    "sensor_cloud_frame",       # new frame
                    "base_link",                # parent
                    )
    # front laser
    #quat = quaternion_from_euler(0.0, 0.0, pi/2)
    br.sendTransform(
                    frontL.getTranslation(),    # translation
                    frontL.getRotationQuat(),   # rotation
                    rospy.Time.now(),           # timestamp
                    "front_laser_link",         # new frame
                    "base_link",                # parent
                    )
    # right laser
    br.sendTransform(
                    rightL.getTranslation(),    # translation
                    rightL.getRotationQuat(),   # rotation
                    rospy.Time.now(),           # timestamp
                    "right_laser_link",         # new frame
                    "base_link",                # parent
                    )
    # left laser
    br.sendTransform(
                    leftL.getTranslation(),     # translation
                    leftL.getRotationQuat(),    # rotation
                    rospy.Time.now(),           # timestamp
                    "left_laser_link",          # new frame
                    "base_link",                # parent
                    )
    # depth camera
    br.sendTransform(
                    camera.getTranslation(),    # translation
                    camera.getRotationQuat(),   # rotation
                    rospy.Time.now(),           # timestamp
                    "camera_link",              # new frame
                    # FIXME parent should be the correct arm link which the camera will be attached to
                    "base_link",                # parent
                    )
    rate.sleep()