#ifndef ROPORT_MZ25_WEBOTS_INTERFACE_H
#define ROPORT_MZ25_WEBOTS_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <mutex>

using namespace hardware_interface;

namespace roport
{
  class MZ25WebotsInterface: public hardware_interface::RobotHW
  {
  public:
    MZ25WebotsInterface(ros::NodeHandle& nh);
    ~MZ25WebotsInterface() override;

    void init();
    void update(const ros::TimerEvent& e);
    void read();
    void write(ros::Duration elapsed_time);

  private:
    void jointStatesCb(const sensor_msgs::JointState::ConstPtr& msg);

    void sendJointPositionCmd(double j1, double j2, double j3, double j4, double j5, double j6);
    double getJointCmdValue(const std::string& jointName);

    ros::Subscriber jointStatesSubscriber;
    ros::Publisher jointPositionCmdPublisher;

  protected:
    ros::NodeHandle& nh_;
    ros::Timer non_realtime_loop_;

    std::shared_ptr<controller_manager::ControllerManager> controllerManager_;

    // hardware interfaces
    JointStateInterface jointStateInterface_;
    PositionJointInterface positionJointInterface_;

    // For each joint name, this contains its joint q, dq, and tau_J
    std::map<std::string, std::vector<double>> jointStates;

    std::timed_mutex jointStatesMutex;

    // Shared memory
    std::vector<std::string> jointNames;
    std::vector<double> currentJointPosition;
    std::vector<double> currentJointVelocity;
    std::vector<double> currentJointEffort;
    std::vector<double> jointPositionCommand;
  };

}


#endif //ROPORT_WEBOTS_INTERFACE_H
