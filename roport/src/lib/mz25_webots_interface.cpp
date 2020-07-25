#include "roport/mz25_webots_interface.h"

#include <utility>


namespace roport
{
  MZ25WebotsInterface::MZ25WebotsInterface(ros::NodeHandle& nh) : nh_(nh)
  {
    init();
    ROS_INFO("Init completed");
    controllerManager_ = std::make_shared<controller_manager::ControllerManager>(this, nh_);
    double loop_hz_ = 100;
    non_realtime_loop_ = nh_.createTimer(ros::Duration(1.0/loop_hz_), &MZ25WebotsInterface::update, this);
    ROS_INFO("Loop started");
  }

  MZ25WebotsInterface::~MZ25WebotsInterface() = default;

  void MZ25WebotsInterface::init()
  {
    // The names are defined in the webots api
    jointNames = {"shoulder_joint", "upperArm_joint", "foreArm_joint",
                  "wrist1_joint", "wrist2_joint", "wrist3_joint"};

    jointStatesSubscriber = nh_.subscribe(
      "mz25/measured_joint_states", 1, &MZ25WebotsInterface::jointStatesCb, this
    );
    ROS_INFO_STREAM("Subscribed to joint states");

    jointPositionCmdPublisher = nh_.advertise<sensor_msgs::JointState>("mz25/joint_states_cmd", 1);
    ROS_INFO_STREAM("Registered joint controllers");

    // Resize vectors
    currentJointPosition.resize(jointNames.size());
    currentJointVelocity.resize(jointNames.size());
    currentJointEffort.resize(jointNames.size());
    jointPositionCommand.resize(jointNames.size());

    std::fill(currentJointPosition.begin(), currentJointPosition.end(), 0);
    std::fill(currentJointVelocity.begin(), currentJointVelocity.end(), 0);
    std::fill(currentJointEffort.begin(),   currentJointEffort.end(), 0);
    std::fill(jointPositionCommand.begin(), jointPositionCommand.end(), 0);
    ROS_INFO_STREAM("Initialized buffers");

    // Initialize Controller
    for (int i = 0; i < jointNames.size(); ++i) {
      // Create joint state interface
      JointStateHandle jointStateHandle(jointNames[i], &currentJointPosition[i],
                                        &currentJointVelocity[i], &currentJointEffort[i]);
      jointStateInterface_.registerHandle(jointStateHandle);

      // Create position joint interface
      JointHandle jointPositionHandle(jointStateHandle, &jointPositionCommand[i]);
      positionJointInterface_.registerHandle(jointPositionHandle);
    }

    registerInterface(&jointStateInterface_);
    registerInterface(&positionJointInterface_);
    ROS_INFO_STREAM("Initialized MZ25 joint states interfaces");
  }

  void MZ25WebotsInterface::update(const ros::TimerEvent& e)
  {
    ///ROS_INFO("Looping");
    ros::Duration elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controllerManager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
  }

  void MZ25WebotsInterface::read()
  {
    for (int i = 0; i < jointNames.size(); i++) {
      try {
        //ROS_INFO_STREAM("Getting state of joint "<<jointNames[i]);
        std::unique_lock<std::timed_mutex> lock(jointStatesMutex, std::chrono::seconds(10));
        if (lock) {
          std::vector<double> jointState = jointStates.at(jointNames[i]);
          currentJointPosition.at(i) = jointState[0];
          currentJointVelocity.at(i) = jointState[1];
          currentJointEffort.at(i) = jointState[2];
        }
        else {
          ROS_ERROR("Couldn't acquire mutex, couldn't get joint state");
        }
        //ROS_INFO_STREAM("Got state of joint "<<jointNames[i]);
      }
      catch(std::out_of_range& e) {
        ROS_WARN_STREAM("Failed to get joint "<<jointNames[i]<<" state, no message received yet");
      }
    }
  }

  void MZ25WebotsInterface::write(ros::Duration elapsed_time)
  {
    sendJointPositionCmd(getJointCmdValue(jointNames[0]),
                         getJointCmdValue(jointNames[1]),
                         getJointCmdValue(jointNames[2]),
                         getJointCmdValue(jointNames[3]),
                         getJointCmdValue(jointNames[4]),
                         getJointCmdValue(jointNames[5])
    );
  }

  double MZ25WebotsInterface::getJointCmdValue(const std::string& jointName)
  {
    for (int i = 0; i < jointNames.size(); i++) {
      if (jointNames[i] == jointName)
        return jointPositionCommand[i];
    }
    return 0;
  }

  void MZ25WebotsInterface::jointStatesCb(const sensor_msgs::JointState::ConstPtr& msg)
  {
    std::unique_lock<std::timed_mutex> lock(jointStatesMutex, std::chrono::seconds(10));
    if (!lock) {
      ROS_ERROR("Couldn't acquire mutex, couldn't set joint state");
      return;
    }
    for (int i = 0; i < jointNames.size(); i++)
      jointStates[jointNames[i]] = std::vector<double>({msg->position.at(i), msg->velocity.at(i), msg->effort.at(i)});
  }

  void MZ25WebotsInterface::sendJointPositionCmd(double j1, double j2, double j3, double j4, double j5, double j6)
  {
    sensor_msgs::JointState msg;
    msg.position = std::vector<double>({j1, j2, j3, j4, j5, j6});
    jointPositionCmdPublisher.publish(msg);
  }
}



