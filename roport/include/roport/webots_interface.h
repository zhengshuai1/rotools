#ifndef ROPORT_WEBOTS_INTERFACE_H
#define ROPORT_WEBOTS_INTERFACE_H

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
  class WalkerWebotsInterface
  {
  public:
    WalkerWebotsInterface(ros::NodeHandle& nh, std::vector<std::string> joint_names);
    ~WalkerWebotsInterface();

    // Shared memory
    std::vector<double> currentJointPosition;
    std::vector<double> currentJointVelocity;
    std::vector<double> currentJointEffort;

    std::vector<double> jointPositionCommand;
    std::vector<double> jointVelocityCommand;
    std::vector<double> jointEffortCommand;

  private:
    void init();
    void read();
    void write();

    void headStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void leftHandStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void leftLimbStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void rightHandStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void rightLimbStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void legsStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void setJointsStates(const sensor_msgs::JointState::ConstPtr& msg, const std::vector<std::string>& joint_names);

    void sendLeftLimbPositionCommand(double shoulderPitch, double shoulderRoll, double shoulderYaw, double elbowRoll, double elbowYaw, double wristPitch, double wristRoll);
    void sendRightLimbPositionCommand(double shoulderPitch, double shoulderRoll, double shoulderYaw, double elbowRoll, double elbowYaw, double wristPitch, double wristRoll);
    void sendHeadPositionCommand(double pitch, double yaw);
    void sendLegsPositionCommand(double leftHipYaw, double leftHipRoll, double leftHipPitch,double leftKneePitch, double leftAnklePitch, double leftAnkleRoll, double rightHipYaw, double rightHipRoll, double rightHipPitch, double rightKneePitch, double rightAnklePitch, double rightAnkleRoll);
    double getJointCommandedValue(const std::string& jointName);

    ros::Subscriber headStateSub;
    ros::Subscriber leftHandStateSub;
    ros::Subscriber leftLimbStateSub;
    ros::Subscriber rightHandStateSub;
    ros::Subscriber rightLimbStateSub;
    ros::Subscriber legsStateSub;

    ros::Publisher leftLimbCommandPublisher;
    ros::Publisher rightLimbCommandPublisher;
    ros::Publisher headCommandPublisher;
    ros::Publisher legsCommandPublisher;

  protected:
    ros::NodeHandle& nh_;
    std::vector<std::string> joint_names_;
    ros::Timer non_realtime_loop_;

    std::map<std::string,std::vector<double>> jointStates;
    std::timed_mutex jointStatesMutex;


  };

  class WalkerWebotsHardwareInterface: public hardware_interface::RobotHW
  {
  public:
    WalkerWebotsHardwareInterface(ros::NodeHandle& nh);
    ~WalkerWebotsHardwareInterface();
    void init();
    void update(const ros::TimerEvent& e);
    void read();
    void write(ros::Duration elapsed_time);

  private:
    void headStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void leftHandStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void leftLimbStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void rightHandStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void rightLimbStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void legsStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void setJointsStates(const sensor_msgs::JointState::ConstPtr& msg, const std::vector<std::string>& jointNames);

    void sendLeftLimbPositionCommand(double shoulderPitch, double shoulderRoll, double shoulderYaw, double elbowRoll, double elbowYaw, double wristPitch, double wristRoll);
    void sendRightLimbPositionCommand(double shoulderPitch, double shoulderRoll, double shoulderYaw, double elbowRoll, double elbowYaw, double wristPitch, double wristRoll);
    void sendHeadPositionCommand(double pitch, double yaw);
    void sendLegsPositionCommand(double leftHipYaw, double leftHipRoll, double leftHipPitch,double leftKneePitch, double leftAnklePitch, double leftAnkleRoll, double rightHipYaw, double rightHipRoll, double rightHipPitch, double rightKneePitch, double rightAnklePitch, double rightAnkleRoll);
    double getJointCommandedValue(std::string jointName);

    ros::Subscriber headStateSub;
    ros::Subscriber leftHandStateSub;
    ros::Subscriber leftLimbStateSub;
    ros::Subscriber rightHandStateSub;
    ros::Subscriber rightLimbStateSub;
    ros::Subscriber legsStateSub;

    ros::Publisher leftLimbCommandPublisher;
    ros::Publisher rightLimbCommandPublisher;
    ros::Publisher headCommandPublisher;
    ros::Publisher legsCommandPublisher;

    bool enable_leg_control;
  protected:
    ros::NodeHandle& nh_;
    ros::Timer non_realtime_loop_;
    PositionJointInterface positionJointInterface;
    std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    // Interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;


    std::map<std::string,std::vector<double>>  jointStates;
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
