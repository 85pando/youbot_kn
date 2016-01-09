#!/usr/bin/env python
"""
This script shows the distances of the primary directions of the LaserScanner.
"""

from os import system
import rospy
from sensor_msgs.msg import LaserScan

### BEGIN MAGIC NUMBERS
NROFDIRS      = 9
# if this is changed, printMainDirections has to be changed as well!
NROFDIGITS    = 3
SCANNERANGLE  = 180
NROFDATAPOINTS = 512
### END MAGIC NUMBERS

# execute this every time data is received
def laserCallback(data):
  seqNr = data.header.seq
  laser = laserCalc(data)
  laser = snipDirections(laser)
  printMainDirections(seqNr, laser[0], laser[1])

def printMainDirections(seqNr, nrOfDirs, ranges):
  """Print directions in a nice way."""
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
  print("Right, Sequence Number: %10u" % seqNr)
  print(lineString)
  print(angleString)
  print(lineString)
  print(rangeString)
  print(lineString)
  print("\n")

def laserCalc(data):
  """Convert the raw data from the laserScan to an array of distances."""
  laser = []
  for range in data.ranges:
    laser.append(range)
  return laser

def snipDirections(ranges):
  """Take only "main" directions from the whole 640 data points."""
  mainRanges = []
  for i in range(NROFDIRS):
    if (i != 0):
      mainRanges.append(round(ranges[(i*NROFDATAPOINTS/(NROFDIRS-1))-1],NROFDIGITS))
    else:
      mainRanges.append(round(ranges[0], NROFDIGITS))
  mainRanges[0] = round(ranges[0], NROFDIGITS)
  return (NROFDIRS, mainRanges)

def init():
  # Possibly mutliple clients want to display scanner data
  rospy.init_node('scannerViz', anonymous=True)
  # Subscribe to Sensor data
  #rospy.Subscriber("/youbot/scan_front", LaserScan, laserCallback)
  rospy.Subscriber("/scan_right", LaserScan, laserCallback)
  # Don't close this until node is stopped.
  rospy.spin()


# python boilerplate
if __name__ == '__main__':
  init()
