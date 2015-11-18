#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This script will take the data of a LaserScan and create a point cloud out of it.

Copyright (C) 2015 Stephan Heidinger
stephan.heidinger@uni-konstanz.de

This work is licensed under a Creative Commons Attribution-ShareAlike 3.0 Unported License.

See the full license text here: https://creativecommons.org/licenses/by-sa/3.0/legalcode
"""

### Imports
import sys
import rospy
from os import system
from sensor_msgs.msg import LaserScan
from math import pi, sqrt, pow, sin, cos #, tan, atan2
### End Imports

class PointCloudCreator:
  """
  This class processes incoming LaserScans and creates a usable point-Cloud
  """

  def __init__(self):
    return None

  def receiveFrontLaser(self, frontLaser):
    """
    The front LaserScanner has sent a new scan. Create a point cloud.
    :param frontLaser: A laserScan as specified by the ROS framework http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
    :return A PointCloud with coordinates relative to the sensor.
    """
    self.relativeFrontCloud = self.convertLaserScanToPointCloud(frontLaser)
    # TODO rotate+translate the cloud according to sensor position on robot

  def convertLaserScanToPointCloud(self, laserData):
    """
    This method takes any LaserScan and produces a pointCloud relative to the sensor position.
    :param laserData: A laserScan as specified by the ROS framework http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
    :return A PointCloud with coordinates relative to the sensor.
    """

    # TODO preprocess header information
    self.nrOfMeasurements = len(laserData.ranges)
    self.halfMeasure = (nrOfMeasurements-1)/2
    cloudPoint = []

    # for each point in the scan
    for index in range(self.nrOfMeasurements):
      xCoord = None
      yCoord = None
      # * TODO decide if it can be used as a point
      #if laserData.ranges[index] ISNOTANUMBERORWHATEVERWERECEIVEHERE:
        #skip
      #else:
        # decide whether the measurement is in the left or right half
        # then calculate relative coordinates
        if index == self.halfMeasure:
          # direction is directly forward
          xCoord = 0
          yCoord = laserData.ranges[index]
        else:
          if index < self.halfMeasure:
            # direction in right half
            angle = index * laserData.angle_increment
          else:
            # direction in left half
            angle = pi - index * laserData.angle_increment
          # x = cos(alpha) * r
          xCoord = cos(angle) * laserData.ranges[index]
          # y = sin(alpha) * r
          yCoord = sin(angle) * laserData.ranges[index]
        # put into output array
        cloudPoint.append( (xCoord,yCoord) )
#      else:
#        print("index", index, "is out of range")
    return sorted(cloudPoint)

# default python boilerplate
if __name__ == "__main__":

  # create instance of the class
  cloudCreator = PointCloudCreator()

  rospy.init_node('create_point_cloud_from_laser', anonymous=True)
  # want to receive messages from laser scanner
  rospy.Subscriber('/youbot/scan_front', LaserScan, cloudCreator.receiveFrontLaser)

  # keep alive
  rospy.spin()
