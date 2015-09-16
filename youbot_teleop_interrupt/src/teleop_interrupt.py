#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This script provides a command line interface to move the robot base around.
It will move the base in a safe way, stopping before it hits anythin on forward directed movement.
"""

# Initial code created by Graylin Trevor Jay (tjay@cs.brown.edu) an published under Crative Commens Attribution license.
# addition for signal interrupt by Koen Buys

# This code is created by Stephan Heidinger (stephan.heidinger@uni-konstanz.de) and published under Creative Commons Attribution license.


from os import system

#import youbot_driver_ros_interface
#import roslib; roslib.load_manifest('youbot_oodl')
import rospy

from geometry_msgs.msg import Twist

import sys
import select
import termios
import tty
import signal

msg = """
Moving around the youbot and stopping before it hits something
==============================================================
Moving:   Rotation: Circle:
   w        ↶  ↷     h  j
a  s  d     u  i     n  m

Alternative, set NUM-Block to numbers
7 8 9     ↖ ↑ ↗
4 5 6     ← ⎊ →
1 2 3     ↙ ↓ ↘

r/t   - increase/deacrease all      speeds by 10%
f/g   - increase/deacrease moving   speed  by 10%
v/b   - increase/deacrease rotation speed  by 10%
space - reset speeds

anything else: stop

CTRL+C to quit
"""

## Begin Magic Numbers
STATUSBARRIER = 15
## End Magic Numbers

## movement direction bindings
movementDirections = {
       # x, y, theta
# linear movement
  'w': ( 1, 0, 0) # forward
, 'a': ( 0, 1, 0) # left
, 's': (-1, 0, 0) # backward
, 'd': ( 0,-1, 0) # right
# NUM-Block linear movement
, '1': (-1, 1, 0) # backward, left
, '2': (-1, 0, 0) # backward
, '3': (-1,-1, 0) # backward, right
, '4': ( 0, 1, 0) # left
, '5': ( 0, 0, 0) # stop
, '6': ( 0,-1, 0) # right
, '7': ( 1, 1, 0) # forward, left
, '8': ( 1, 0, 0) # forward
, '9': ( 1,-1, 0) # forward, right
# rotational movement
, 'u': ( 0, 0, 1) # counter-clockwise
, 'i': ( 0, 0,-1) # clockwise
# circular movement (linear + rotation)
, 'h': ( 1, 0, 1) # ccw + forward
, 'j': ( 1, 0,-1) # cw  + forward
, 'n': (-1, 0, 1) # ccw + backward
, 'm': (-1, 0,-1) # cw  + backward
}

#speed modification bindings
speedModifier = {
  'r': (1.1,1.1)
, 't': (0.9,0.9)
, 'f': (1.1,1.0)
, 'g': (0.9,1.0)
, 'v': (1.0,1.1)
, 'b': (1.0,0.9)
}

class TimeoutException(Exception):
  pass

def getKey():
  def timeout_handler(signum, frame):
    raise TimeoutException()

  old_handler = signal.signal(signal.SIGALRM, timeout_handler)
  signal.alarm(1) # watchdog, check for key every ms
  tty.setraw(sys.stdin.fileno())
  select.select([sys.stdin], [], [], 0)
  try:
    key = sys.stdin.read(1)
    #print("Read key"); print(key)
  except TimeoutException:
    #print("Timeout")
    return "-"
  finally:
    signal.signal(signal.SIGALRM, old_handler)

  signal.alarm(0)
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key

def vels(speedV, turnV):
  return "current:\tspeed %1.2f\tturn %1.2f " % (speedV, turnV)

if __name__ == "__main__":
  settings = termios.tcgetattr(sys.stdin)

  # queue_size=1 => only the latest command will be kept
  pub = rospy.Publisher('/youbot/cmd_vel_safe', Twist, queue_size=1)
  rospy.init_node('teleop_interrupt')

  # movement and rotation direction values
  x      = 0
  y      = 0
  th     = 0

  # movement and rotation speed values
  speed = 0.1
  turn  = 0.3

  # number of status messages printed to console
  status = 0

  try:
    system('clear')
    print(msg)
    print(vels(speed,turn))
    while(True):
      key = getKey()
      if key in movementDirections.keys():
        # add movement stuff
        x  = movementDirections[key][0]
        y  = movementDirections[key][1]
        th = movementDirections[key][2]

      elif key in speedModifier.keys():
        # add speed stuff
        speed = speed * speedModifier[key][0]
        turn  = turn  * speedModifier[key][1]
        print(vels(speed,turn)); status += 1
      elif key == " ":
        # reset speed values
        speed = 0.1
        turn  = 0.3
        print(vels(speed,turn)); status += 1

      else:
        x = 0
        y = 0
        th = 0
        # if ctrl+c
        if (key == '\x03'):
          break

      # publish the correct messages
      # linear movement
      twist = Twist()
      twist.linear.x = x * speed
      twist.linear.y = y * speed
      twist.linear.z = 0

      # rotation
      twist.angular.x = 0
      twist.angular.y = 0
      twist.angular.z = th * turn

      # publish
      pub.publish(twist)

      # to many status messages, repeat instructions
      if (status >= STATUSBARRIER-1):
        system('clear')
        print(msg)
        status = (status + 1) % STATUSBARRIER

  except:
    print e

  finally:
    # send command velocities to 0 (stop the robot)
    print("Stopping the robot")
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub.publish(twist)
    print("Shutting down.")
