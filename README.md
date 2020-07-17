# RoTools

A robot toolbox belonging to the **RoUniverse**, which is a personal project aiming
at developing handy robotic related tools during learning and implementing.

RoTools is compatible with both Python 2.7 and 3.7+

Author: Zhipeng Dong

## Install

Place the root folder to `/home/USER`

Add `export PYTHONPATH=${PYTHONPATH}:/home/USER/rotools` into the bashrc.

You may need to `source ~/.bashrc` for new terminal.


## Submodules

The submodules in RoTools/rotools are designed to run as a stand alone executable.
Refer README in each submodule for a detailed guide.

### simulation

Interfaces for RoTools to interact with 3rd-party software like the MoveIt! 
motion planing package and simulators such as Webots and CoppeliaSim.

### moveit

Interfaces for RoTools to interact with the MoveIt! motion planing package.

### policy

Policies for motion planning and trajectory generation.

### robot

Control models for different kinds of robots.

**serial**: control model for serial manipulators (robot arms)

### utility

Utilities for general function

## RoPort

RoPort is a ROS package that bridge the RoTools interface with ROS. This package should
be placed in the catkin_ws/src during usage.

## Coding Guide

This toolbox use *attrs* for a joyful coding of classes in Python, for an overview
of that refer [attrs]



[attrs]: <https://www.attrs.org/en/stable/overview.html>