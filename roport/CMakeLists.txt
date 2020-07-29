cmake_minimum_required(VERSION 3.0.2)
project(roport)

#Comment this if don't want debug
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  behaviortree_cpp_v3
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  hardware_interface
  controller_manager
  ubt_core_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BehaviorTree.msg
  NodeParameter.msg
  NodeStatus.msg
  StatusChange.msg
  StatusChangeLog.msg
  TreeNode.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetAllNames.srv
  GetGroupPose.srv
  GetGroupJointStates.srv
  ExecuteGroupPose.srv
  ExecuteGroupJointStates.srv
  ExecuteGroupPlan.srv
  ExecuteAllPoses.srv
  ExecuteAllPlans.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  ubt_core_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

include_directories(include ${catkin_INCLUDE_DIRS})

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/lib/rosout_logger.cpp
  src/lib/moveit_interface.cpp
  src/lib/robot_interface.cpp
  src/lib/duration.cpp
  src/lib/control_types.cpp
  src/lib/webots_interface.cpp
  src/lib/mz25_webots_interface.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(roport_task_scheduler src/roport_task_scheduler.cpp)
add_dependencies(roport_task_scheduler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roport_task_scheduler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(roport_moveit_server src/roport_moveit_server.cpp)
add_dependencies(roport_moveit_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roport_moveit_server
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(roport_walker_webots_interface src/roport_walker_webots_interface.cpp)
add_dependencies(roport_walker_webots_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roport_walker_webots_interface
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(roport_mz25_webots_interface src/roport_mz25_webots_interface.cpp)
add_dependencies(roport_mz25_webots_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roport_mz25_webots_interface
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
