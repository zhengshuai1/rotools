# RoTools

A robot toolbox belonging to the **RoUniverse**, which is a personal project aiming
at developing handy robotic related tools during learning and implementing.

RoTools is compatible with both Python 2.7 and 3.7+

Author: Zhipeng Dong

## Submodules

The submodules of RoTools are designed to running individually or jointly.
Refer README in each submodule for a detailed guide.

### interface

Interfaces for RoTools to interact with 3rd-party software like the MoveIt! 
motion planing package and simulators such as Webots.

### policy

Policies for motion planning and trajectory generation.

#### robot

Control models for different kinds of robots

**serial**: control model for serial manipulators (robot arms)

#### utility

Utilities for general function

## Coding Guide

This toolbox use *attrs* for a joyful coding of classes in Python, for an overview
of that refer [attrs]


[attrs]: <https://www.attrs.org/en/stable/overview.html>