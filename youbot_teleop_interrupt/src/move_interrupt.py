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
distanceLeft  = 0.184
distanceRight = 0.184
distanceFront = 0.230
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

def calibrate():
  """Calibrate the position of the laser-Scanner on the youbot. It is possible to just enter the values for the safe box or measure them using the laser and some calibration object (e.g. a piece of paper)."""
  global laserData
  global distanceLeft
  global distanceRight
  global distanceFront
  if query_yes_no("Do you know the values for the safe box?", default="no"):
    # directly enter safe box values
    valuesOk = False
    while not valuesOk:
      # left border
      print("Enter value for left border:")
      distanceLeft = readNumber()
      # right border
      print("Enter value for right border:")
      distanceRight = readNumber()
      # front border
      print("Enter value for front border:")
      distanceFront = readNumber()
      # check if values are ok
      print("left border: %1.3f, right border: %1.3f, front border: %1.3f" % (distanceLeft, distanceRight, distanceFront))
      valuesOk = query_yes_no("Are these values correct?", default="yes")
  else:
    # TODO enter values using the laser scanner itsself
    valuesOk = False
    while not valuesOk:
      # left border
      print("%s border: Put calibration object on the %s border, then press return." % ("left", "left"))
      sys.stdin.readline()
      distanceLeft = laserData.ranges[NROFDATAPOINTS-1]
      # right border
      print("%s border: Put calibration object on the %s border, then press return." % ("right", "right"))
      sys.stdin.readline()
      distanceRight = laserData.ranges[0]
      # front border
      print("%s border: Put calibration object on the %s border, then press return." % ("front", "front"))
      sys.stdin.readline()
      distanceFront = laserData.ranges[NROFDATAPOINTS/2-1]
      # Check if values are good
      print("left border: %1.3f, right border: %1.3f, front border: %1.3f" % (distanceLeft, distanceRight, distanceFront))
      valuesOk = query_yes_no("Do these values seem correct?", default="yes")

def readNumber():
  """Read in a number from stdin. Check if the string is a number and prompts until it is otherwise."""
  value = sys.stdin.readline()
  while not isNumber(value):
    print("Please enter a numerical value")
    value = sys.stdin.readline()
  return float(value)

def isNumber(s):
  """Can this string be interpreted as a number?"""
  try:
    float(s)
    return True
  except ValueError:
    return False

def query_yes_no(question, default="yes"):
  """Ask a yes/no question via raw_input() and return their answer.

  "question" is a string that is presented to the user.
  "default" is the presumed answer if the user just hits <Enter>.
      It must be "yes" (the default), "no" or None (meaning
      an answer is required of the user).

  The "answer" return value is True for "yes" or False for "no".

  Taken from http://code.activestate.com/recipes/577058/
  """
  valid = {"yes": True, "y": True,  "Y": True,
           "no": False, "n": False, "N": False}
  if default is None:
    prompt = " [y/n] "
  elif default == "yes":
    prompt = " [Y/n] "
  elif default == "no":
    prompt = " [y/N] "
  else:
    raise ValueError("invalid default answer: '%s'" % default)

  while True:
    sys.stdout.write(question + prompt)
    choice = raw_input().lower()
    if default is not None and choice == '':
      return valid[default]
    elif choice in valid:
      return valid[choice]
    else:
      sys.stdout.write("Please respond with 'yes' or 'no' "
                       "(or 'y' or 'n').\n")

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
  # ask the user if the laserScanner is to be calibrated
  if query_yes_no("Do you want to calibrate the laser-scanner?", default="no"):
    calibrate();
  else:
    print("Laser-scanner will not be calibrated. Using default values:\n\tleft border: %1.3f, right border: %1.3f, front border: %1.3f" % (distanceLeft, distanceRight, distanceFront))
  # keep alive
  rospy.spin()
