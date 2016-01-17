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
#import csv
#from os import system
#from math import pi, sqrt, pow, sin, cos #, tan, atan2
### Imports for tests
#import random
### End Imports

def extractPoints(infilePath):
  infile  = open(infilePath, "r")

  pointArea  = False

  #print("Entering line processing")
  
  pointCloud = []
  
  for line in infile:
    splitline = line.split(" ")

    if not pointArea and splitline[0] != "POINTS":
      #sprint(splitline)
      continue
    if not pointArea and splitline[0] == "POINTS":
      pointArea = True
      #print("POINTS found")
      continue

    # we are now below the header
    
    #print("below header")
    #print(splitline)
    point=[0,0,0]
    x = False
    y = False
    z = False
    value = None

    for block in splitline:
      # check if this is still points or already farther
      if block == "VERTICES":
        break
      if block == []:
        continue
      try:
        value = float(block)
      except ValueError:
        continue
      except:
        print("Oops:", splitline)
      # now we are reasonably sure this is actually a point
      # check the line for valid values and assign them to the next coordinate
      if not x:
        point[0] = value
        x = True
      elif not y:
        point[1] = value
        y = True
      elif not z:
        point[2] = value
        z = True
      if x and y and z:
        pointCloud.append(point)
      
  infile.close()
  return pointCloud


#default python boilerplate
if __name__ == "__main__":

  if len(sys.argv) != 2:
    print("Usage: ", sys.argv[0], "in-file.vtk", sep=" ")
    exit(1)

  infilePath  = sys.argv[1]
  print("In-File:", infilePath)

  print("\nStarting extraction of points from VTK")
  
  pointCloud = extractPoints(infilePath)
  print("======= PointCloud =======")
  print(pointCloud)
    