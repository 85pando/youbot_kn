#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This script will move the base in a safe way, stopping before it hits anythin on forward directed movement.
"""

# This code is created by Stephan Heidinger (stephan.heidinger@uni-konstanz.de) and published under Creative Commons Attribution license.

from os import system

import sys

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

### BEGIN MAGIC NUMBERS
NROFDIRS      = 9
SCANNERANGLE  = 180
NROFDATAPOINTS = 640
### END MAGIC NUMBERS

### BEGIN GLOBAL VARIABLES
laserData = LaserScan()
### END GLOBAL VARIABLES

def receiveCmd(cmdData):
  """A movement command is received, process it!"""
  # want to access the global variable
  global laserData
  # TODO check laserScan for objects
  # TODO if no object in movement direction, publish
  if laserData.ranges[NROFDATAPOINTS/2-1] <= 0.4:
    print("No safe movement possible, distance is %1.4f.\n" %laserData.ranges[319])
    return None
  else:
    #print("publishing")
    #print(cmdData)
    pub.publish(cmdData)
    return None

def receiveLaser(laser):
  """The LaserScanner has send a new scan. Make it processable."""
  # want to write into global variable
  global laserData
  laserData = laser
  #laserData['seqNr'] = laser.header.seq
  #laserData['time']  = laser.header.stamp.secs
  #for i in range(len(laser.ranges)):
    #laserData[i] = laser.ranges[i]
  #print(laserData.ranges[319])
  return None


# default python boilerplate
if __name__ == "__main__":
  # want to send movement message to actual control
  pub = rospy.Publisher('/youbot/cmd_vel', Twist, queue_size=1)
  # want to receive movement commands from other nodes
  rospy.Subscriber('cmd_vel_safe', Twist, receiveCmd)
  # want to receive messages from the laser scanner
  rospy.Subscriber('/youbot/scan_front', LaserScan, receiveLaser)
  # initialize node
  rospy.init_node('base_move_interrupt', anonymous=False)
  # keep alive
  rospy.spin()
