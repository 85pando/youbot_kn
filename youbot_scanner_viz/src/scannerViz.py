#!/usr/bin/env python
"""
This script shows the distances of the primary directions of the LaserScanner.
"""

from os import system
import rospy
from sensor_msgs.msg import LaserScan
from math import pi, sin, cos

### BEGIN MAGIC NUMBERS
NROFDIRS     = 9
NROFDIGITS   = 3 # if this is changed, printMainDirections has to be changed as well!
SCANNERANGLE = 180
### END MAGIC NUMBERS


# execute this every time data is received
def laserCallback(data):
  seqNr = data.header.seq
  laser = laserCalc(data)
  laser = snipDirections(laser)
  printMainDirections(seqNr, laser[0], laser[1])

"""
Print directions in a nice way.
"""
def printMainDirections(seqNr, nrOfDirs, ranges):
  angles = []
  for dir in range(nrOfDirs):
    angles.append(dir * SCANNERANGLE / (nrOfDirs-1))
  # prettyPrint - Build the Strings
  lineString = "+"
  for column in range(nrOfDirs):
    lineString += "-------+"
  angleString = "|"
  for angle in angles:
    angleString += "  %3.0f  |" % angle
  rangeString = "|"
  for rangei in ranges:
    rangeString += " %1.3f |" % rangei
  # prettyPrint - Print the Strings
  print("Sequence: %10u" % seqNr)
  print(lineString)
  print(angleString)
  print(lineString)
  print(rangeString)
  print(lineString)
  print("\n\n")

"""
Convert the raw data from the laserScan to an array of distances.
"""
def laserCalc(data):
  laser = []
  for range in data.ranges:
    laser.append(range)
  return laser

# the following example code on how to convert LaserScan Data is from
# http://nullege.com/codes/show/src@f@r@FroboMind-HEAD@fmApp@sdu@sdu_surveying_2014@survey@survey_log_node.py/40/sensor_msgs.msg.LaserScan
def laserCalc2(data):
  laser = []
  theta = data.angle_min
  for i in xrange(len(data.ranges)):
    r = data.ranges[i]
    laser.append([r*sin(theta), r*cos(theta)]) #polar to cartesian
    theta += data.angle_increment
  return laser

"""
Take only "main" directions from the whole 640 data points.
"""
def snipDirections(ranges):
  mainRanges = []
  for i in range(NROFDIRS):
    mainRanges.append(round(ranges[i*640/NROFDIRS],NROFDIGITS))
  return (NROFDIRS, mainRanges)

def init():
  # Possibly mutliple clients want to display scanner data
  rospy.init_node('scannerViz', anonymous=True)
  # Subscribe to Sensor data
  rospy.Subscriber("/youbot/scan_front", LaserScan, laserCallback)
  # Don't close this until node is stopped
  rospy.spin()



if __name__ == '__main__':
  init()
