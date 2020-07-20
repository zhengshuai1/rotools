#ifndef ROPORT_MOVEIT_INTERFACE_H
#define ROPORT_MOVEIT_INTERFACE_H

#include <ros/ros.h>
#include <ros/console.h>

#include <actionlib/server/simple_action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <roport/GetAllNames.h>
#include <roport/GetGroupPose.h>
#include <roport/GetGroupJointStates.h>

#include <roport/ExecuteGroupPose.h>
#include <roport/ExecuteGroupPlan.h>
#include <roport/ExecuteGroupJointStates.h>

#include <roport/ExecuteAllPoses.h>
#include <roport/ExecuteAllPlans.h>

typedef moveit::planning_interface::MoveGroupInterface move_group;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client;


class MoveItInterface {

public:
  MoveItInterface(const std::vector<std::string>& group_names,
                  std::vector<std::string> ee_links, std::vector<std::string> ref_frames);

private:
  std::vector<std::shared_ptr<move_group>> groups_;
  std::vector<std::shared_ptr<action_client>> clients_;

  static std::vector<control_msgs::JointTolerance> buildTolerance(double position, double velocity,
                                                                  double acceleration, int jointNum);
  control_msgs::FollowJointTrajectoryGoal buildPath(const move_group::Plan& plan);

  bool executePlans(const std::vector<move_group::Plan>& plans);
  bool checkExecution();

protected:
  bool groupsMoveToPoses(std::vector<geometry_msgs::Pose> poses);
  bool groupsMoveToPoses(std::vector<geometry_msgs::PoseArray> poses);

};


class MoveItServer : public MoveItInterface {

public:
  MoveItServer(const ros::NodeHandle& nh, const std::vector<std::string>& group_names,
               std::vector<std::string> ee_links, std::vector<std::string> ref_frames);

private:
  ros::NodeHandle nh_;

  ros::ServiceServer srv_execute_all_position_;
  ros::ServiceServer srv_execute_all_rpy_;
  ros::ServiceServer srv_execute_all_poses_;
  ros::ServiceServer srv_execute_all_joint_states_;
  ros::ServiceServer srv_execute_all_plans_;

  bool executeAllPosesCB(roport::ExecuteAllPoses::Request &req, roport::ExecuteAllPoses::Response &resp);
//  bool executeGroupJointStatesCB(roport::ExecuteGroupJointStates::Request &req,
//                                 roport::ExecuteGroupJointStates::Response resp);
  bool executeAllPlansCB(roport::ExecuteAllPlans::Request &req, roport::ExecuteAllPlans::Response &resp);

};

#endif //ROPORT_MOVEIT_INTERFACE_H
