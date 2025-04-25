# VADER Dynamixel
## Attribution

This repository belongs to Carnegie Mellon University, Masters of Science - Robotic Systems Development (MRSD) Team E - VADER

Team Members: Tom Gao, Abhishek Mathur, Rohit Satishkumar, Kshitij Bhat, Keerthi GV 

First Revision: February 2025

## Introduction and Overview

This repository contains driver code for the dynamixel end effectors found on our robot. The motors on the gripper are daisy-chained together, and connected via a U2D2 USB bridge to the computer with a USB cable. The cutter motor is also connected through a separate USB bridge. The IDs of the motors and the USB bridges are defined in `config/dynamixel_config.yaml`.

The main driver program is `dynamixel_node.py`. Change the values for `fakeCutter` and `fakeGripper` inside the program to choose between 1) real communication through USB devices and 2) fake execution which sends commands to specified ROS topics, which triggers the fake end effectors in the simulator.

The tendon controlled motors have behavior defined by the angles when the fingers are fully open and fully closed, which are defined in the python script. For the simulator, the angles are separately defined. Whenever the gripper/cutter is modified the new angles should be tuned using Dynamixel Wizard and recorded in the script.

## Usage
Clone this program into the workspace. Make sure fakeCutter and fakeGripper are set appropriately. Then, run:

```bash
rosrun vader_dynamixel dynamixel_node.py
```