# RoPort

RoPort is a middleware that bridges the RoTools interface with ROS environment.

# Pre-requests

```
sudo apt-get install ros-melodic-behaviortree-cpp-v3
```

# Usage

Demo with Franka Panda robot:

```
roslaunch panda_moveit_config demo.launch
roslaunch roport roport_panda.launch
```

Demo with UBTech Walker robot:

```
roslaunch walker_moveit_config demo.launch
roslaunch roport roport_walker.launch
```