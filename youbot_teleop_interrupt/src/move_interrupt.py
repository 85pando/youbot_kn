#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This script will move the base in a safe way, stopping before it hits anythin on forward directed movement.
"""

# This code is created by Stephan Heidinger (stephan.heidinger@uni-konstanz.de) and published under Creative Commons Attribution license.

from os   import system
from math import sin, atan2, tan, pi, sqrt, pow
import sys

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

### BEGIN MAGIC NUMBERS
NROFDATAPOINTS        = 640
ROBOTWIDTH            = 380
ROBOTLENGTH           = 580
### END MAGIC NUMBERS

### BEGIN GLOBAL VARIABLES
laserData      = LaserScan()
distanceLeft   = 0.200 # wheel end is 190
distanceRight  = 0.200 # wheel end is 190
distanceFront  = 0.100 #
distanceBack   = 0.590 # length of the robot is 580
lastCmd        = Twist()
### END GLOBAL VARIABLES

def receiveCmd(cmdData):
  """A movement command is received, process it!"""
  if isMovementDirectionClear(cmdData):
    # if no object in movement direction, publish
    pub.publish(cmdData)
    return None
  else:
    stopRobot()
    return None

def isMovementDirectionClear(cmdData):
  # check laserScan for objects (linear movement)
  if cmdData.linear.x > 0:
    # forward
    if cmdData.linear.y > 0:
      # forward left
      if not safeBoxDiagonal(cmdData, NROFDATAPOINTS-1):
        # check for tail of robot
        return False
      elif not safeBox(cmdData.linear.x):
        # check safe box
        return False
    elif cmdData.linear.y < 0:
      # forward right
      if not safeBoxDiagonal(cmdData, 0):
        # check for tail of robot
        return False
      elif not safeBox(cmdData.linear.x):
        # check safeBox
        return False
    else:
      # pure forward movement
      if not safeBox(cmdData.linear.x):
        return False
  else:
    # no x-movement
    if cmdData.linear.y > 0:
      # left only movement, can not really see much here
      if laserData.ranges[NROFDATAPOINTS-1] <= distanceLeft + cmdData.linear.y:
        return False
    elif cmdData.linear.y < 0:
      # right only movement, can not really see much here
      if laserData.ranges[0] <= distanceRight - cmdData.linear.y:
        return False
    else:
      # no movement, so we don't care
      pass
  # check laserScan for objects (rotational movement)
  if cmdData.angular.z != 0:
    # turning left
    if not safeBox(0):
      return False
  # did not find anything in the way
  return True

def safeBoxDiagonal(cmdData, ray):
  """Calculates if the diagonal safeBox is indeed safe.

    cmdData -- the movement order
    ray     -- Which ray to use for calulating the diagonal safeBox. For right movement this should normally be 0 and for left movement NROFDATAPOINTS-1
  """
  x = cmdData.linear.x
  y = abs(cmdData.linear.y) # this is abs, because right movement is negative
  angle = atan2(x,y)
  distanceValue = tan(pi/2 - angle) * distanceBack + distanceRight
  movementDistance = sqrt(pow(x,2) + pow(y,2))
  if laserData.ranges[ray] < distanceValue + movementDistance:
    return False
  # diagonal safeBox seems safe
  return True

def safeBox(movementDistance):
  """
  Calculates, if the safebox is indeed safe (i.e. nothing in there).

    movementDistance -- How far will the robot move? Use 0 for rotation.
  """
  for i in range(int(numberOfMeasurementPoints)):
    # the angle of the current laser-beam
    iAngle          = safeboxRightAngle + i * measurementAngleInc
    # how far is the object, measured from the line perpendicular to the forward direction
    forwardSpace    = laserData.ranges[int(safeboxRightCorner+i*measurementIncrement)] * sin(iAngle)
    # how much do we want to move
    forwardMovement = distanceFront + movementDistance
    # can we do this?
    if forwardSpace <= forwardMovement:
      return False
  # We have found nothing in the Safe Box
  return True

def stopRobot():
  """Print an interrupt message."""
  global lastCmd
  print("Movement interrupted: %u s" % laserData.header.stamp.secs)
  lastCmd.linear.x  = 0
  lastCmd.linear.y  = 0
  lastCmd.angular.z = 0
  pub.publish(lastCmd)

def receiveLaser(laser):
  """The LaserScanner has send a new scan. Make it processable."""
  global lastCmd
  global laserData
  # store new measurements
  laserData = laser
  # if we have movement, we need to look ahead
  if lastCmd.linear.x != 0 and lastCmd.linear.y != 0 and lastCmd.angular.z != 0 :
    if not isMovementDirectionClear(lastCmd):
      # sudden obstacle, stop the robot
      stopRobot()
  return None

def calibrate():
  """Calibrate the position of the laser-Scanner on the youbot. It is possible to just enter the values for the safe box or measure them using the laser and some calibration object (e.g. a piece of paper)."""
  global distanceLeft
  global distanceRight
  global distanceFront
  global distanceBack
  if query_yes_no("Do you know the values for the safe box?", default="no"):
    print("Default values:\n\tleft border: %1.3f, right border: %1.3f, front border: %1.3f, back border: %1.3f" % (distanceLeft, distanceRight, distanceFront, distanceBack))
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
      # back border
      print("Enter value for back border:")
      distanceBack = readNumber()
      # check if values are ok
      printSizeWarning()
      print("left border: %1.3f, right border: %1.3f, front border: %1.3f, back border: %1.3f" % (distanceLeft, distanceRight, distanceFront, distanceBack))
      valuesOk = query_yes_no("Are these values correct?", default="yes")
  else:
    # enter values using the laser scanner itsself
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
      print("%s border: Put calibration object on the %s border+length (see documentation), then press return. You can also enter this value manually or type 'n' to use the current value (%1.3f)." % ("back", "right", distanceBack))
      line = sys.stdin.readline()
      if line == "n":
        print("using default value")
      elif isNumber(line):
        print("using numerical value")
        distanceBack = float(line)
      else:
        print("using sensor data")
        distanceBack = laserData.ranges[0] - distanceRight
      # back border
      # Check if values are good
      printSizeWarning()
      print("left border: %1.3f, right border: %1.3f, front border: %1.3f, back border: %1.3f" % (distanceLeft, distanceRight, distanceFront, distanceBack))
      valuesOk = query_yes_no("Do these values seem correct?", default="yes")
  calculateSafeBox()
  return None

def printSizeWarning():
  """Prints a warning when the lengths for the safeboxes are smaller than the actual robot."""
  global distanceLeft
  global distanceRight
  global distanceBack
  if (distanceLeft + distanceRight) >= ROBOTWIDTH:
    print("Warning: The width of your safeBox is smaller than the robot.")
  if (distanceRight + distanceBack) >= (ROBOTLENGTH + ROBOTWIDTH/2):
    print("Warning: The length of your safeBox is smaller than the robot.")

def calculateSafeBox():
  """Calculates new values for the safe box. This has to be used after calibration or distances have changed"""
  global safeboxRightAngle
  global safeboxLeftAngle
  global safeboxRightCorner
  global safeboxLeftCorner
  global safeboxFrontRange
  global safeboxAngleRange
  global numberOfMeasurementPoints # how many rays will be used to check for objects
  global measurementIncrement      # how many rays are in between two measurement points
  global measurementAngleInc       # which angle is between two measurement points
  # calculate angle for corner, both angles are calculated from the zero ray of the scanner
  safeboxRightAngle =      atan2(distanceFront,distanceRight)
  safeboxLeftAngle  = pi - atan2(distanceFront,distanceLeft)
  # make sure we are in the right quadrant:
  if not (safeboxRightAngle >= 0):
    raise SafeBoxAngleException("Right Angle below zero: %1.3f" % safeboxRightAngle)
  elif not (safeboxRightAngle <= pi/2):
    raise SafeBoxAngleException("Right Angle above pi/2: %1.3f" % safeboxRightAngle)
  if not (safeboxLeftAngle >= pi/2):
    raise SafeBoxAngleException("Left Angle below pi/2: %1.3f" % safeboxLeftAngle)
  elif not (safeboxLeftAngle <= pi):
    raise SafeBoxAngleException("Left Angle above pi: %1.3f" % safeboxLeftAngle)
  # calculate data point for corner
  safeboxRightCorner = safeboxRightAngle / pi * (NROFDATAPOINTS-1)
  safeboxLeftCorner  = safeboxLeftAngle  / pi * (NROFDATAPOINTS-1)
  # calculate front stuff
  safeboxFrontRange = safeboxLeftCorner - safeboxRightCorner
  safeboxAngleRange = safeboxLeftAngle  - safeboxRightAngle
  # calculate the rest
  numberOfMeasurementPoints = safeboxFrontRange / 2
  measurementIncrement      = safeboxFrontRange / (numberOfMeasurementPoints-1)
  measurementAngleInc       = safeboxAngleRange / (numberOfMeasurementPoints-1)

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

class Error(Exception):
  """Base class for exceptions in this module."""
  pass

class SafeBoxAngleException(Error):
  """This Exception shows, that one of the angles of the safebox is broken."""
  def __init__(self, message):
    self.message = message
  def __str__(self):
    return repr(self.message)

# default python boilerplate
if __name__ == "__main__":
  # want to send movement message to actual control
  pub = rospy.Publisher('/youbot/cmd_vel', Twist, queue_size=1)
  # want to receive movement commands from other nodes
  rospy.Subscriber('/youbot/cmd_vel_safe', Twist, receiveCmd)
  # want to receive messages from the laser scanner
  rospy.Subscriber('/youbot/scan_front', LaserScan, receiveLaser)
  # initialize node
  rospy.init_node('base_move_interrupt', anonymous=False)
  # ask the user if the laserScanner is to be calibrated
  if query_yes_no("Do you want to calibrate the laser-scanner?", default="no"):
    calibrate();
  else:
    try:
      calculateSafeBox()
    except SafeBoxAngleException as e:
      print(e)
    print("Using default values:\n\tleft border: %1.3f, right border: %1.3f, front border: %1.3f" % (distanceLeft, distanceRight, distanceFront))
  # keep alive
  rospy.spin()
