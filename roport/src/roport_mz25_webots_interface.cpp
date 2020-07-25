#include <algorithm>
#include <array>
#include <atomic>
#include <string>
#include <utility>

#include <ros/ros.h>

#include "roport/robot_interface.h"
#include "roport/mz25_webots_interface.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "roport_mz25_webots_interface");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  roport::MZ25WebotsInterface robot_interface(nh);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}


