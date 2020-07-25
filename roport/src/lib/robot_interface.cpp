# include "roport/robot_interface.h"

#include <utility>


using namespace hardware_interface;

namespace roport
{
  RobotInterface::RobotInterface(std::vector<std::string> joint_names,
                                 const urdf::Model& urdf_model,
                                 const std::vector<double>& position,
                                 const std::vector<double>& velocity,
                                 const std::vector<double>& effort,
                                 const std::vector<double>& position_cmd,
                                 const std::vector<double>& velocity_cmd,
                                 const std::vector<double>& effort_cmd)
    : joint_names_(std::move(joint_names)),
      q_(position),
      dq_(velocity),
      tau_J_(effort),
      position_joint_cmd_(position_cmd),
      velocity_joint_cmd_(velocity_cmd),
      effort_joint_cmd_(effort_cmd)
  {
    init();
    ROS_INFO("RoPort: Robot interface initialized.");
  }

  RobotInterface::~RobotInterface()
  = default;

  void RobotInterface::init()
  {
    for (size_t i = 0; i < joint_names_.size(); i++) {
      JointStateHandle joint_handle_q(
        joint_names_[i],
        &q_[i],
        &dq_[i],
        &tau_J_[i]
      );

      joint_state_interface_.registerHandle(joint_handle_q);

      JointHandle position_joint_handle(joint_handle_q, &position_joint_cmd_.q[i]);
      position_joint_interface_.registerHandle(position_joint_handle);

      JointHandle velocity_joint_handle(joint_handle_q, &velocity_joint_cmd_.dq[i]);
      velocity_joint_interface_.registerHandle(velocity_joint_handle);

      JointHandle effort_joint_handle(joint_handle_q, &effort_joint_cmd_.tau_J[i]);
      effort_joint_interface_.registerHandle(effort_joint_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&effort_joint_interface_);
  }

  void RobotInterface::update(const roport::RobotState& robot_state)
  {
//    robot_state_ = robot_state;
  }

  void RobotInterface::control(const std::function<bool(const ros::Time &, const ros::Duration &)> &ros_cb) {
//    roport::Duration last_time = robot_state_.time;

//    run_function_([this, ros_cb, &last_time](const roport::RobotState& robot_state,
//                                             roport::Duration time_step) {
//      if (last_time != robot_state.time) {
//        last_time = robot_state.time;
//        return ros_cb(ros::Time::now(), ros::Duration(time_step.toSec()));
//      }
//      return true;
//    });
  }
}

