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
from __future__ import print_function
from __future__ import division
import sys
import rospy
from os import system
from sensor_msgs.msg import LaserScan
from math import pi, sqrt, pow, sin, cos #, tan, atan2
### Imports for tests
import random
### End Imports

class PointCloudCreator:
  """
  This class processes incoming LaserScans and creates a usable point-Cloud
  """

  def __init__(self):
    self.frontRotation    = 0
    self.frontTranslation = (0,.290)
    self.frontHeight      = .005
    self.rightRotation    = 90
    self.rightTranslation = (.190,0)
    self.rightHeight      = .005
    self.backRotation     = 180
    self.backTranslation  = (0,-.290)
    self.backHeight      = .005
    self.leftRotation     = 270
    self.leftTranslation  = (-.190,0)
    self.leftHeight      = .005
    return None



  def receiveFrontLaser(self, frontLaser):
    """
    The front LaserScanner has sent a new scan. Create a point cloud.
    :param frontLaser: A laserScan as specified by the ROS framework http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
    :return A PointCloud with coordinates relative to the center of the robot.
    """
    frontCloud = self.convertLaserScanToPointCloud(frontLaser, self.frontHeight)
    self.frontCloud = self.relocatePointCloud(frontCloud,
                                              self.frontRotation,
                                              self.frontTranslation
                                              )
    ## FIXME Remove this code ## it just simulates another sensor update
    self.receiveRightLaser(frontLaser)
    ## END remove this code ##

    self.writePointCloudToFile('frontCloud.csv', self.frontCloud)
    return None


  def receiveRightLaser(self, rightLaser):
    """
    The right LaserScanner has sent a new scan. Create a point cloud.
    :param rightLaser: A laserScan as specified by the ROS framework http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
    :return A PointCloud with coordinates relative to the center of the robot.
    """
    rightCloud = self.convertLaserScanToPointCloud(rightLaser, self.rightHeight)
    ## FIXME Remove this code ## it just mirrors the data of the first sensor
    newCloud = []
    random.seed("42")
    for point in rightCloud:
      xrand = random.randint(0,10)/1000
      yrand = random.randint(0,10)/1000
      x = point[0] + xrand
      y = point[1] + yrand
      z = point[2]
      newCloud.append((y, x, z))
    rightCloud = newCloud
    ## END remove this code ##
    self.rightCloud = self.relocatePointCloud(rightCloud,
                                              self.rightRotation,
                                              self.rightTranslation
                                              )
    self.writePointCloudToFile('rightCloud.csv', self.rightCloud)
    return None


  def writePointCloudToFile(self, fileName, pointCloud):
    f = open(fileName, 'w')
    # print header
    f.write('x,y,z\n')
    #print('x,y')
    for point in pointCloud:
      # print one point per line
      f.write(str(point[0]))
      f.write(',')
      f.write(str(point[1]))
      f.write(',')
      f.write(str(point[2]))
      f.write('\n')
    f.close()
    print("====== wrote", fileName, len(pointCloud), "======")
    return None


  def relocatePointCloud(self, pointCloud, rotation, translation):
    """
    This method takes a point cloud and rotates and translates it to it actual location.
    :param oldPointCloud: The point cloud is expected to be in the form of an array of points with x and y coordinates like so [(x1,y1), (x2,y2), ...]
    :param rotation: Rotation is given clockwise. Currently only 0, 90, 180 and 270 degree are supported.
    :param translation: The translation should represent the location of the sensor in the real robot. It has to be a tuple (x,y).
    :return The rotated and translated point cloud.
    """

    # rotate the point cloud
    rotatedPointCloud = []
    if rotation == 0:
      # don't need to rotate here
      rotatedPointCloud = pointCloud
      pass
    elif rotation == 90:
      # rotate each point 90 degree
      for point in pointCloud:
        x = point[0]
        y = point[1]
        z = point[2]
        rotatedPointCloud.append((y, -x, z))
    elif rotation == 180:
      # rotate each point 180 degree
      for point in pointCloud:
        x = point[0]
        y = point[1]
        z = point[2]
        rotatedPointCloud.append((-x, -y, z))
    elif rotation == 270:
      # rotate each point 270 degree
      for point in pointCloud:
        x = point[0]
        y = point[1]
        z = point[2]
        rotatedPointCloud.append((-y, x, z))
    else:
      raise InvalidRotationException(rotation)

    # don't need the original pointCloud any more
    del pointCloud

    # translate each point cloud
    rotatedAndTranslatedPointCloud = []
    for point in rotatedPointCloud:
      x  = point[0]
      xt = translation[0]
      y  = point[1]
      yt = translation[1]
      z = point[2]
      rotatedAndTranslatedPointCloud.append((x+xt, y+yt, z))

    # don't need rotated point any more
    del rotatedPointCloud

    return rotatedAndTranslatedPointCloud



  def convertLaserScanToPointCloud(self, laserData, zCoord):
    """
    This method takes any LaserScan and produces a pointCloud relative to the sensor position.
    :param laserData: A laserScan as specified by the ROS framework http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
    :return A PointCloud with coordinates relative to the sensor.
    """

    # preprocess header information
    self.nrOfMeasurements = len(laserData.ranges)
    self.halfMeasure      = (self.nrOfMeasurements-1)/2
    self.range_min        = laserData.range_min #+ 0.01 * laserData.range_min
    self.range_max        = laserData.range_max #- 0.01 * laserData.range_max
    cloudPoints = []

    # for each point in the scan
    for index in range(self.nrOfMeasurements):
      xCoord = None
      yCoord = None
      # points are valid if they are closer than the maximum and farther than the minimum distance
      if ( (laserData.ranges[index] < self.range_max) and
           (laserData.ranges[index] > self.range_min) ):
        # decide whether the measurement is in the left or right half or forward
        if index == self.halfMeasure:
          # direction is directly forward
          xCoord = 0
          yCoord = laserData.ranges[index]
        else:
          angle = index * laserData.angle_increment
          # calculate relative coordinates
          xCoord = cos(angle) * laserData.ranges[index]
          yCoord = sin(angle) * laserData.ranges[index]
        # put into output array
        cloudPoints.append( (xCoord,yCoord,zCoord) )
    return sorted(cloudPoints)



class InvalidRotationException(Exception):
  def __init__(self, angle):
    self.angle = angle
  def __str__(self):
    return repr("Invalid rotation angle:" + self.angle)



# default python boilerplate
if __name__ == "__main__":

  # create instance of the class
  cloudCreator = PointCloudCreator()

  rospy.init_node('create_point_cloud_from_laser', anonymous=True)
  # want to receive messages from laser scanner
  rospy.Subscriber('/youbot/scan_front', LaserScan, cloudCreator.receiveFrontLaser)

  # keep alive
  rospy.spin()
