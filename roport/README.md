# RoPort

RoPort is a middleware that allows using RoTools interface in ROS environment.

It provides: 

A MoveIt server for controlling the robot's kinematic chains.

A BehaviorTree based task scheduler.

Hardware/Simulation interfaces with multiple robots.

# Pre-requests

```
sudo apt-get install ros-melodic-behaviortree-cpp-v3
```

# Usage

MoveIt demo with Franka Panda robot:

```
roslaunch panda_moveit_config demo.launch
roslaunch roport roport_moveit_panda.launch
```

MoveIt demo with UBTech Walker robot:

```
roslaunch walker_moveit_config demo.launch
roslaunch roport roport_moveit_walker.launch
```

# Note

### Simultaneously execution

The python version MoveItServer do not support simultaneously execution. To
perform such function, you need to directly send goal to the controller client.
A example is given in the C++ version of MoveItServer. However, you may need to
implement the hardware_interface supporting the controllers.