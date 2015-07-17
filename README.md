# Youbot KN

This repo contains nodes for the Konstanz Kuka Youbot.

## branches

* ``master`` used with the simulator
* ``on_robot`` used on the actual robot

## youbot_scanner_viz

Simple Python script to show what the laser scanner is reading.
Simple C++ program to show what the laser scanner is reading.

## youbot_teleop_interrupt

* `teleop_interrupt.py`: Teleop-node that uses the safe movement node `move_interrupt.py`. It is also vastly superior to the default teleop-node
* `move_interrupt.py`: Safe movement node that checks if there are any objects when moving in a forward based direction. When moving purely sideways this is not so easy, as we don't have a sensor here.
