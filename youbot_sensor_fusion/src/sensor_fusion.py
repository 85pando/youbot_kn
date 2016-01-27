#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This script will take the data of LaserScans and create a point clouds out of it.

Copyright (C) 2015-2016 Stephan Heidinger
stephan.heidinger@uni-konstanz.de

This work is licensed under a Creative Commons Attribution-ShareAlike 3.0 Unported License.

See the full license text here: https://creativecommons.org/licenses/by-sa/3.0/legalcode
"""

### Imports
from __future__ import print_function
from __future__ import division
import sys
import rospy
import csv
from os import system, path, chdir, devnull
import subprocess
from sensor_msgs.msg import LaserScan
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from math import pi, sqrt, pow, sin, cos #, tan, atan2
from extractPointsFromVTK import extractPoints
### End Imports

class PointCloudCreator:
  """
  This class processes incoming LaserScans and creates a usable point-Cloud
  """

  def __init__(self, cloudPublisher):
    # lengths are in m
    # rotations are in degree
    self.frontRotation    = 0
    self.frontTranslation = (0,.220)
    self.frontHeight      = .001
    self.rightRotation    = 90
    self.rightTranslation = (.110,0)
    self.rightHeight      = .001
    self.backRotation     = 180
    self.backTranslation  = (0,-.220)
    self.backHeight       = .001
    self.leftRotation     = 270
    self.leftTranslation  = (-.110,0)
    self.leftHeight       = .001
    
    # timestamps
    self.frontTimestamp  = rospy.Time()
    self.rightTimestamp  = rospy.Time()
    self.leftTimestamp   = rospy.Time()
    self.kinectTimestamp = rospy.Time()
    
    # values for ICP stuff
    self.resetICPCheckValues() # this sets the ready values
    self.laserICPyamlPath = '/tmp/laserICP.yaml'
    self.frontCloudPath   = '/tmp/frontCloud.csv'
    self.rightCloudPath   = '/tmp/rightCloud.csv'
    self.leftCloudPath    = '/tmp/leftCloud.csv'
    self.kinectCloudPath  = '/tmp/kinectCloud.csv'
    self.icpResultDir     = '/tmp/'
    self.icpResultPath    = '/tmp/test_data_out.vtk'
    self.devnull          = open(devnull, 'w')
    
    # publisher
    self.cloudPublisher = cloudPublisher
    self.seq = 0
    self.frame_id = "/sensor_cloud_frame"
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
    del frontCloud
    self.writePointCloudToFile(self.frontCloudPath, self.frontCloud)
    # frontCloud can now be processed
    self.frontTimestamp.secs  = frontLaser.header.stamp.secs
    self.frontTimestamp.nsecs = frontLaser.header.stamp.nsecs
    self.frontReady = True
    self.executeIcp()
    return None


  def receiveRightLaser(self, rightLaser):
    """
    The right LaserScanner has sent a new scan. Create a point cloud.
    :param rightLaser: A laserScan as specified by the ROS framework http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
    :return A PointCloud with coordinates relative to the center of the robot.
    """
    rightCloud = self.convertLaserScanToPointCloud(rightLaser, self.rightHeight)
    self.rightCloud = self.relocatePointCloud(rightCloud,
                                              self.rightRotation,
                                              self.rightTranslation
                                              )
    del rightCloud
    self.writePointCloudToFile(self.rightCloudPath, self.rightCloud)
    # don't need to save rightCloud, this will come from the ICP result
    self.rightTimestamp.secs  = rightLaser.header.stamp.secs
    self.rightTimestamp.nsecs = rightLaser.header.stamp.nsecs
    self.rightReady = True
    self.executeIcp
    return None


  def resetICPCheckValues(self):
    """
    Resets some values, so that we know when we can execute the next icp
    """
    self.frontReady  = False
    self.rightReady  = False
    # those sensors are not yet present, so we need not wait for them
    self.leftReady   = True
    self.kinectReady = True
    return None


  def executeIcp(self):
    """
    Executes icp via the pmicp programm from libpointmatcher. Publishes the result to topic "/sensor_cloud" and does not store it any further.
    """
    if (not self.frontReady) or (not self.rightReady) or (not self.leftReady) or (not self.kinectReady):
      # there is a cloud that is not yet ready
      return None
    # all clouds are ready, reset check values
    self.resetICPCheckValues()
    #decide what the timestamp should be #FIXME probably take the kinect timestamp here, as this is probably the one with the lowest frequency
    timestamp = rospy.Time()
    if (self.frontTimestamp.secs == self.rightTimestamp.secs):
      # when all timestamps are in the same second, check the nanoseconds
      # TODO when more sensors, check if there is one oldest
      if (self.frontTimestamp.nsecs < self.rightTimestamp.nsecs):
        timestamp = self.frontTimestamp
      else:
        timestamp = self.rightTimestamp
    else:
      # if not all timestamps in the same second, take the oldest
      if (self.frontTimestamp.secs < self.rightTimestamp.secs):
        timestamp = self.frontTimestamp
      else:
        timestamp = self.rightTimestamp
    
    # create config file for ICP if it does not exist
    if not path.isfile(self.laserICPyamlPath):
      print("Creating ICP-configfile")
      self.createLaserICPConfig()
    
    # change into /tmp/ so that result is in the expected place
    chdir("/tmp/")
    frontCloud = self.frontCloud
    
    
    # execute ICP front <-> right
    subprocess.call(['pmicp', '--config', self.laserICPyamlPath,
                     self.frontCloudPath, self.rightCloudPath],
                     stdout=self.devnull,stderr=self.devnull)

    # extract new rightCloud from vtk-file
    rightCloud = extractPoints(self.icpResultPath)
    
    # execute ICP front <-> left, FIXME uncomment this, when second laser present
    #subprocess.call(['pmicp', '--config', self.laserICPyamlPath,
    #                 self.frontCloudPath, self.leftCloudPath],
    #                 stdout=self.devnull)
    # extract new leftCloud from vtk-file, FIXME use comment, as soon as left laser present
    leftCloud = [] # extractPoints(self.icpResultPath)
    
    # combine frontCloud + rightCloud
    frontRightCloud = frontCloud + rightCloud
    del frontCloud
    del rightCloud
    
    # combine frontRightCloud + leftCloud
    laserCloud = frontRightCloud + leftCloud
    del leftCloud
    
    # execute ICP laserCloud <-> kinectCloud, FIXME uncomment this, when kinect present
    #subprocess.call(['pmicp', '--config', self.laserICPyamlPath,
    #                 self.kinectCloudPath, self.rightCloudPath],
    #                 stdout=self.devnull)
    #
    # extract new kinectCloud from vtk-file, FIXME use comment as soon as kinect present
    kinectCloud = [] # extractPoints(self.icpResultPath)
    
    # combine finalCloud = laserCloud + kinectCloud
    finalCloud = laserCloud + kinectCloud
    del laserCloud
    del kinectCloud
    
    #self.writePointCloudToFile("/tmp/icpResult.csv", finalCloud)
    
    self.publishPointCloud(finalCloud, timestamp)
    return None



  def writePointCloudToFile(self, fileName, pointCloud):
    """
    Writes a given pointCloud to a given fileName.
    :fileName The file name to write to.
    :pointCloud The pointCloud that will be written to file.
    """
    f = open(fileName, 'w')
    outwriter = csv.writer(f)
    # write header
    outwriter.writerow(["x", "y", "z"])
    outwriter.writerows(pointCloud)
    f.close()
    return None


  def publishPointCloud(self, pointCloud, timestamp):
    """
    Publishes the given pointCloud to the topic "/sensor_cloud" as PoinCloud2 message.
    :pointCloud The pointCloud to be published.
    """
    # create variable of pc2 type
    pc2cloud = PointCloud2()
    # convert existing pointCloud into correct format
    pc2cloud = pc2.create_cloud_xyz32(pc2cloud.header, pointCloud)
    pc2cloud.header.seq = self.seq
    self.seq = self.seq + 1
    pc2cloud.header.stamp.secs  = timestamp.secs
    pc2cloud.header.stamp.nsecs = timestamp.nsecs
    # insert correct frame_id here
    pc2cloud.header.frame_id = self.frame_id
    # actually publish the pointcloud
    self.cloudPublisher.publish(pc2cloud)
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
    elif rotation == 90 or rotation == -270:
      # rotate each point 90 degree
      for point in pointCloud:
        x = point[0]
        y = point[1]
        z = point[2]
        rotatedPointCloud.append([y, -x, z])
    elif rotation == 180 or rotation == -180:
      # rotate each point 180 degree
      for point in pointCloud:
        x = point[0]
        y = point[1]
        z = point[2]
        rotatedPointCloud.append([-x, -y, z])
    elif rotation == 270 or rotation == -90:
      # rotate each point 270 degree
      for point in pointCloud:
        x = point[0]
        y = point[1]
        z = point[2]
        rotatedPointCloud.append([-y, x, z])
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
      rotatedAndTranslatedPointCloud.append([x+xt, y+yt, z])

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
    self.range_min        = laserData.range_min
    self.range_max        = laserData.range_max
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
        cloudPoints.append( [xCoord,yCoord,zCoord] )
    return cloudPoints


  def createLaserICPConfig(self):
    """
    Creates the config file needed for the pmip executable to do ICP. Writes this to self.laserICPyamlPath, where it will be read from.
    """
    f = open(self.laserICPyamlPath, 'w')
    f.write("readingDataPointsFilters:\n")
    f.write("  - SurfaceNormalDataPointsFilter:\n")
    f.write("      knn: 10\n")
    f.write("\n")
    f.write("referenceDataPointsFilters:\n")
    f.write("  - SurfaceNormalDataPointsFilter:\n")
    f.write("      knn: 10\n")
    f.write("\n")
    f.write("matcher:\n")
    f.write("  KDTreeMatcher:\n")
    f.write("    knn: 1\n")
    f.write("    epsilon: 0\n")
    f.write("    maxDist: 0.25\n")
    f.write("\n")
    f.write("outlierFilters:\n")
    f.write("  - TrimmedDistOutlierFilter:\n")
    f.write("      ratio: 0.85\n")
    f.write("\n")
    f.write("errorMinimizer:\n")
    f.write("  PointToPointErrorMinimizer\n")
    f.write("\n")
    f.write("transformationCheckers:\n")
    f.write("  - CounterTransformationChecker:\n")
    f.write("      maxIterationCount: 40\n")
    f.write("  - DifferentialTransformationChecker:\n")
    f.write("      minDiffRotErr: 0.001\n")
    f.write("      minDiffTransErr: 0.01\n")
    f.write("      smoothLength: 4\n")
    f.write("\n")
    f.write("inspector:\n")
    f.write("  VTKFileInspector\n")
    f.write("\n")
    f.write("logger:\n")
    f.write("  FileLogger\n")

    f.close()


class InvalidRotationException(Exception):
  def __init__(self, angle):
    self.angle = angle
  def __str__(self):
    return repr("Invalid rotation angle:" + self.angle)



# default python boilerplate
if __name__ == "__main__":

  rospy.init_node('sensor_fusion', anonymous=True)

  # create publishers
  cloudPublisher = rospy.Publisher('/sensor_cloud', PointCloud2, queue_size=1)
  
  # create instance of the class
  cloudCreator = PointCloudCreator(cloudPublisher)
  
  # want to receive messages from laser scanner
  rospy.Subscriber('/scan', LaserScan, cloudCreator.receiveFrontLaser)
  rospy.Subscriber('/scan_right', LaserScan, cloudCreator.receiveRightLaser)
  # keep alive
  rospy.spin()
