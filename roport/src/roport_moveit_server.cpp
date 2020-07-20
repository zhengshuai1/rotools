#include "roport/moveit_interface.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "roport_moveit_cpp_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::vector<std::string> group_names;
  pnh.getParam("group_names", group_names);
  std::vector<std::string> ee_links;
  pnh.getParam("ee_links", ee_links);
  std::vector<std::string> ref_frames;
  pnh.getParam("ref_frames", ref_frames);

  MoveItServer server(nh, group_names, ee_links, ref_frames);
  ROS_WARN("RoPort: MoveIt cpp server ready.");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::waitForShutdown();
  return 0;
}


