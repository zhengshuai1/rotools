#include "roport/moveit_interface.h"

#include <utility>


MoveItInterface::MoveItInterface(const std::vector<std::string>& group_names,
                                 std::vector<std::string> ee_links, std::vector<std::string> ref_frames)
{
  int cnt = 0;
  for (const auto& name : group_names) {
    std::shared_ptr<move_group> group = std::make_shared<move_group>(name,std::shared_ptr<tf2_ros::Buffer>(),
                                                                     ros::Duration(10));

    if (!ee_links.empty()) {
      group->setEndEffectorLink(ee_links[cnt]);
    }
    if (!ref_frames.empty()) {
      group->setPoseReferenceFrame(ref_frames[cnt]);
    }
    groups_.push_back(group);

    //std::string controller_name = name + std::string("_controller/follow_joint_trajectory");
    //std::shared_ptr<action_client> client = std::make_shared<action_client>(controller_name, true);;
    //if (!client->waitForServer(ros::Duration(1))) {
    //  throw std::runtime_error("RoPort: Wait for controller client timed out.");
    //}
    //clients_.push_back(client);
    cnt++;
  }
}

bool MoveItInterface::groupsMoveToPoses(std::vector<geometry_msgs::Pose> poses) {
  assert(poses.size() == groups_.size());

  std::vector<move_group::Plan> plans;
  for (int i = 0; i < poses.size(); ++i) {
    groups_[i]->setPoseTarget(poses[i]);
    move_group::Plan plan;
    groups_[i]->plan(plan);
    plans.push_back(plan);
  }

  if (!executePlans(plans))
    return false;

  //return checkExecution();
  return true;
}

bool MoveItInterface::groupsMoveToPoses(std::vector<geometry_msgs::PoseArray> poses) {
  assert(poses.size() == groups_.size());

  std::vector<move_group::Plan> plans;
  for (int i = 0; i < poses.size(); ++i) {
    groups_[i]->setPoseTargets(poses[i].poses);
    move_group::Plan plan;
    groups_[i]->plan(plan);
    plans.push_back(plan);
  }

  if (!executePlans(plans))
    return false;

  return true;
  //return checkExecution();
}

std::vector<control_msgs::JointTolerance> MoveItInterface::buildTolerance(double position, double velocity,
                                                                          double acceleration, int jointNum)
{
  control_msgs::JointTolerance gjt;
  gjt.position = position;
  gjt.velocity = velocity;
  gjt.acceleration = acceleration;
  return std::vector<control_msgs::JointTolerance>(jointNum, gjt);
}

control_msgs::FollowJointTrajectoryGoal MoveItInterface::buildPath(const move_group::Plan& plan) {
  control_msgs::FollowJointTrajectoryGoal path;
  path.goal_time_tolerance = ros::Duration(1);
  path.goal_tolerance = buildTolerance(0.01, 0, 0, 7);
  path.path_tolerance = buildTolerance(0.02, 0, 0, 7);
  path.trajectory = plan.trajectory_.joint_trajectory;
  return path;
}

bool MoveItInterface::executePlans(const std::vector<move_group::Plan>& plans) {
  try {
    int cnt = 0;
    for (const auto& plan : plans) {
      //control_msgs::FollowJointTrajectoryGoal t = buildPath(plan);
      //clients_[cnt]->sendGoal(t);
      groups_[cnt]->asyncExecute(plan);
      cnt++;
    }
    return true;
  } catch (std::runtime_error& e) {
    ROS_ERROR("RoPort: %s", e.what());
    return false;
  }
}

bool MoveItInterface::checkExecution() {
  for (int i = 0; i < clients_.size(); ++i) {
    bool ok = clients_[i]->waitForResult(ros::Duration(20.0));
    if (!ok) {
      std::string name = groups_[i]->getName();
      ROS_ERROR("RoPort: Execute plan for %s failed.", name.c_str());
      return false;
    }
  }
  return true;
}


MoveItServer::MoveItServer(const ros::NodeHandle& nh, const std::vector<std::string>& group_names,
                           std::vector<std::string> ee_links, std::vector<std::string> ref_frames) :
  MoveItInterface(group_names, std::move(ee_links), std::move(ref_frames)),
  nh_(nh)
{
  srv_execute_all_poses_ = nh_.advertiseService("execute_all_poses", &MoveItServer::executeAllPosesCB, this);
  srv_execute_all_plans_ = nh_.advertiseService("execute_all_plans", &MoveItServer::executeAllPlansCB, this);
}

bool MoveItServer::executeAllPosesCB(roport::ExecuteAllPoses::Request &req, roport::ExecuteAllPoses::Response &resp)
{
  try {
    bool ok = groupsMoveToPoses(req.goals.poses);
    if (ok) {
      resp.result_status = resp.SUCCEEDED;
    } else {
      resp.result_status = resp.FAILED;
    }
    return true;
  } catch (std::runtime_error& e) {
    ROS_ERROR("RoPort: %s", e.what());
    return false;
  }
}

bool MoveItServer::executeAllPlansCB(roport::ExecuteAllPlans::Request &req, roport::ExecuteAllPlans::Response &resp) {
  try {
    bool ok = groupsMoveToPoses(req.all_poses);
    if (ok) {
      resp.result_status = resp.SUCCEEDED;
    } else {
      resp.result_status = resp.FAILED;
    }
    return true;
  } catch (std::runtime_error& e) {
    ROS_ERROR("RoPort: %s", e.what());
    return false;
  }
}
