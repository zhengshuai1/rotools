#include <algorithm>
#include <array>
#include <atomic>
#include <string>
#include <utility>

#include <ros/ros.h>

#include "roport/robot_interface.h"
#include "roport/webots_interface.h"


//class ServiceContainer {
//public:
//  template <typename T, typename... TArgs>
//  ServiceContainer& advertiseService(TArgs&&... args) {
//    ros::ServiceServer server = franka_control::advertiseService<T>(std::forward<TArgs>(args)...);
//    services_.push_back(server);
//    return *this;
//  }
//
//private:
//  std::vector<ros::ServiceServer> services_;
//};

int main(int argc, char** argv) {
  ros::init(argc, argv, "roport_walker_webots_interface");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

//  std::vector<std::string> joint_names_vector;
//  if (!pnh.getParam("joint_names", joint_names_vector)) {
//    ROS_ERROR("RoPort: No joint_names parameters provided.");
//    return 1;
//  }

//  std::vector<std::string> joint_names = {"right_limb_j1", "right_limb_j2", "right_limb_j3", "right_limb_j4", "right_limb_j5", "right_limb_j6",
//                                          "right_limb_j7", "right_thumb_j1", "right_thumb_j2", "right_index_j1", "right_index_j2", "right_middle_j1",
//                                          "right_middle_j2", "right_ring_j1", "right_ring_j2", "right_pinky_j1", "right_pinky_j2", "left_limb_j1",
//                                          "left_limb_j2", "left_limb_j3", "left_limb_j4", "left_limb_j5", "left_limb_j6", "left_limb_j7",
//                                          "left_thumb_j1", "left_thumb_j2", "left_index_j1", "left_index_j2", "left_middle_j1", "left_middle_j2",
//                                          "left_ring_j1", "left_ring_j2", "left_pinky_j1", "left_pinky_j2", "head_j1", "head_j2", "right_leg_j1",
//                                          "right_leg_j2", "right_leg_j3", "right_leg_j4", "right_leg_j5", "right_leg_j6", "left_leg_j1",
//                                          "left_leg_j2", "left_leg_j3", "left_leg_j4", "left_leg_j5", "left_leg_j6"};
//  std::copy(joint_names_vector.cbegin(), joint_names_vector.cend(), joint_names.begin());

//  bool rate_limiting;
//  if (!pnh.getParamCached("rate_limiting", rate_limiting)) {
//    ROS_ERROR("Invalid or no rate_limiting parameter provided");
//    return 1;
//  }

//  urdf::Model urdf_model;
//  if (!urdf_model.initParamWithNodeHandle("robot_description", nh)) {
//    ROS_ERROR("RoPort: Could not initialize URDF model from robot_description.");
//    return 1;
//  }

//  std::string robot_ip;
//  if (!pnh.getParam("robot_ip", robot_ip)) {
//    ROS_ERROR("Invalid or no robot_ip parameter provided");
//    return 1;
//  }
//
//  std::string arm_id;
//  if (!pnh.getParam("arm_id", arm_id)) {
//    ROS_ERROR("Invalid or no arm_id parameter provided");
//    return 1;
//  }

  roport::WalkerWebotsHardwareInterface robot_interface(nh);

//  std::atomic_bool has_error(false);
//
//  ServiceContainer services;
//  services
//    .advertiseService<franka_control::SetJointImpedance>(
//      node_handle, "set_joint_impedance",
//      [&robot](auto&& req, auto&& res) {
//        return franka_control::setJointImpedance(robot, req, res);
//      })
//    .advertiseService<franka_control::SetCartesianImpedance>(
//      node_handle, "set_cartesian_impedance",
//      [&robot](auto&& req, auto&& res) {
//        return franka_control::setCartesianImpedance(robot, req, res);
//      })
//    .advertiseService<franka_control::SetEEFrame>(
//      node_handle, "set_EE_frame",
//      [&robot](auto&& req, auto&& res) { return franka_control::setEEFrame(robot, req, res); })
//    .advertiseService<franka_control::SetKFrame>(
//      node_handle, "set_K_frame",
//      [&robot](auto&& req, auto&& res) { return franka_control::setKFrame(robot, req, res); })
//    .advertiseService<franka_control::SetForceTorqueCollisionBehavior>(
//      node_handle, "set_force_torque_collision_behavior",
//      [&robot](auto&& req, auto&& res) {
//        return franka_control::setForceTorqueCollisionBehavior(robot, req, res);
//      })
//    .advertiseService<franka_control::SetFullCollisionBehavior>(
//      node_handle, "set_full_collision_behavior",
//      [&robot](auto&& req, auto&& res) {
//        return franka_control::setFullCollisionBehavior(robot, req, res);
//      })
//    .advertiseService<franka_control::SetLoad>(
//      node_handle, "set_load",
//      [&robot](auto&& req, auto&& res) { return franka_control::setLoad(robot, req, res); });
//
//  actionlib::SimpleActionServer<franka_control::ErrorRecoveryAction> recovery_action_server(
//    node_handle, "error_recovery",
//    [&](const franka_control::ErrorRecoveryGoalConstPtr&) {
//      try {
//        robot.automaticErrorRecovery();
//        has_error = false;
//        recovery_action_server.setSucceeded();
//        ROS_INFO("Recovered from error");
//      } catch (const franka::Exception& ex) {
//        recovery_action_server.setAborted(franka_control::ErrorRecoveryResult(), ex.what());
//      }
//    },
//    false);
//
//  franka::Model model = robot.loadModel();
//  auto get_rate_limiting = [&]() {
//    node_handle.getParamCached("rate_limiting", rate_limiting);
//    return rate_limiting;
//  };
//  auto get_internal_controller = [&]() {
//    node_handle.getParamCached("internal_controller", internal_controller);
//
//    franka::ControllerMode controller_mode;
//    if (internal_controller == "joint_impedance") {
//      controller_mode = franka::ControllerMode::kJointImpedance;
//    } else if (internal_controller == "cartesian_impedance") {
//      controller_mode = franka::ControllerMode::kCartesianImpedance;
//    } else {
//      ROS_WARN("Invalid internal_controller parameter provided, falling back to joint impedance");
//      controller_mode = franka::ControllerMode::kJointImpedance;
//    }
//
//    return controller_mode;
//  };
//  auto get_cutoff_frequency = [&]() {
//    node_handle.getParamCached("cutoff_frequency", cutoff_frequency);
//    return cutoff_frequency;
//  };
//  const size_t j_num = joint_names.size();
//  std::vector<double> position;
//  position.resize(j_num);
//  std::vector<double> velocity;
//  velocity.resize(j_num);
//  std::vector<double> effort;
//  effort.resize(j_num);

//  roport::RobotInterface robot_control(joint_names, urdf_model,
//                                       robot.currentJointPosition,
//                                       robot.currentJointVelocity,
//                                       robot.currentJointEffort,
//                                       robot.jointPositionCommand,
//                                       robot.jointVelocityCommand,
//                                       robot.jointEffortCommand);
//
//  // Initialize robot state before loading any controller
//  franka_control.update(robot.readOnce());
//
//  controller_manager::ControllerManager control_manager(&robot_control, nh);
//
//  recovery_action_server.start();
//
  // Start background threads for message handling
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
//  while (ros::ok()) {
//    ros::Time last_time = ros::Time::now();
//
//    // Wait until controller has been activated or error has been recovered
////    while (!robot_control.controllerActive() || has_error) {
////      robot_control.update(robot.readOnce());
////
////      ros::Time now = ros::Time::now();
////      control_manager.update(now, now - last_time);
////      last_time = now;
////
////      if (!ros::ok()) {
////        return 0;
////      }
////    }
//
//    try {
//      // Run control loop. Will exit if the controller is switched.
//      robot_control.control([&](const ros::Time& now, const ros::Duration& period) {
//        if (period.toSec() == 0.0) {
//          // Reset controllers before starting a motion
//          control_manager.update(now, period, true);
////          robot_control.reset();
//        } else {
//          control_manager.update(now, period);
////          robot_control.enforceLimits(period);
//        }
//        return ros::ok();
//      });
//    } catch (const std::runtime_error& e) {
//      ROS_ERROR("%s", e.what());
//      has_error = true;
//    }
//  }

  return 0;
}


