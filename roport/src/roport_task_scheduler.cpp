#include <roport/bt_service_node.h>
#include <roport/bt_action_node.h>
#include <roport/bt_generic_types.h>
#include <roport/rosout_logger.h>

// ROS
#include <ros/ros.h>

// Services (customized)
#include <roport/ExecuteGroupJointStates.h>
#include <roport/ExecuteGroupPose.h>
#include <roport/ExecuteGroupPlan.h>

#include <roport/ExecuteAllPoses.h>
#include <roport/ExecuteAllPlans.h>

// Actions (customized)


using namespace BT;

class ExecuteGroupAngularJointStates : public RosServiceNode<roport::ExecuteGroupJointStates>
{
public:
  ExecuteGroupAngularJointStates(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteGroupJointStates>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<std::string>("group_name"),
      InputPort<DoubleArray>("goal"),
      InputPort<double>("tolerance"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);
    DoubleArray goal{};
    getInput<DoubleArray>("goal", goal);
    request.goal = goal.degToROS();
    getInput<double>("tolerance", request.tolerance);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteGroupLinearJointStates : public RosServiceNode<roport::ExecuteGroupJointStates>
{
public:
  ExecuteGroupLinearJointStates(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteGroupJointStates>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<std::string>("group_name"),
      InputPort<DoubleArray>("goal"),
      InputPort<double>("tolerance"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);
    DoubleArray goal{};
    getInput<DoubleArray>("goal", goal);
    request.goal = goal.plainToROS();
    getInput<double>("tolerance", request.tolerance);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteGroupPose : public RosServiceNode<roport::ExecuteGroupPose>
{
public:
  ExecuteGroupPose(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteGroupPose>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<std::string>("group_name"),
      InputPort<int>("is_absolute"),
      InputPort<Pose>("goal"),
      InputPort<double>("tolerance"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);

    int is_absolute;
    getInput<int>("is_absolute", is_absolute);
    request.is_absolute = bool(is_absolute);

    Pose goal{};
    getInput<Pose>("goal", goal);
    request.goal = goal.toROS();

    getInput<double>("tolerance", request.tolerance);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteGroupPlan : public RosServiceNode<roport::ExecuteGroupPlan>
{
public:
  ExecuteGroupPlan(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteGroupPlan>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<std::string>("group_name"),
      InputPort<int>("is_absolute"),
      InputPort<PoseArray>("poses"),
      InputPort<double>("stamp"),
      InputPort<int>("allow_collision"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);

    int is_absolute;
    getInput<int>("is_absolute", is_absolute);
    request.is_absolute = bool(is_absolute);

    PoseArray poses{};
    getInput<PoseArray>("poses", poses);
    request.poses = poses.toROS();

    getInput<double>("stamp", request.stamp);

    int allow_collision;
    getInput<int>("allow_collision", allow_collision);
    request.allow_collision = bool(allow_collision);

    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteAllPoses : public RosServiceNode<roport::ExecuteAllPoses>
{
public:
  ExecuteAllPoses(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteAllPoses>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<StringArray>("group_names"),
      InputPort<int>("is_absolute"),
      InputPort<PoseArray>("goals"),
      InputPort<DoubleArray>("stamps"),
      InputPort<int>("allow_collision"),
    };
  }

  void onSendRequest(RequestType &request) override {
    StringArray group_names{};
    getInput<StringArray>("group_names", group_names);
    request.group_names = group_names.toROS();

    int is_absolute;
    getInput<int>("is_absolute", is_absolute);
    request.is_absolute = bool(is_absolute);

    PoseArray goals{};
    getInput<PoseArray>("goals", goals);
    request.goals = goals.toROS();

    DoubleArray stamps{};
    getInput<DoubleArray>("stamps", stamps);
    request.stamps = stamps.plainToROS();

    int allow_collision;
    getInput<int>("allow_collision", allow_collision);
    request.allow_collision = bool(allow_collision);

    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteAllPlans : public RosServiceNode<roport::ExecuteAllPlans>
{
public:
  ExecuteAllPlans(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteAllPlans>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<StringArray>("group_names"),
      InputPort<int>("is_absolute"),
      InputPort<PoseArrayArray>("all_poses"),
      InputPort<DoubleArray>("stamps"),
      InputPort<int>("allow_collision"),
    };
  }

  void onSendRequest(RequestType &request) override {
    StringArray group_names{};
    getInput<StringArray>("group_names", group_names);
    request.group_names = group_names.toROS();

    int is_absolute;
    getInput<int>("is_absolute", is_absolute);
    request.is_absolute = bool(is_absolute);

    PoseArrayArray all_poses{};
    getInput<PoseArrayArray>("all_poses", all_poses);
    request.all_poses = all_poses.toROS();

    DoubleArray stamps{};
    getInput<DoubleArray>("stamps", stamps);
    request.stamps = stamps.plainToROS();

    int allow_collision;
    getInput<int>("allow_collision", allow_collision);
    request.allow_collision = bool(allow_collision);

    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "roport_bt_port");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string tree_file;
  pnh.getParam("tree_file", tree_file);
  if (tree_file.empty()) {
    ROS_ERROR("RoPort: No valid tree file.");
    return -1;
  }

  // We use the BehaviorTreeFactory to register our custom nodes
  BT::BehaviorTreeFactory factory;

  RegisterRosService<ExecuteGroupAngularJointStates>(factory, "ExecuteGroupAngularJointStates", nh);
  RegisterRosService<ExecuteGroupLinearJointStates>(factory, "ExecuteGroupLinearJointStates", nh);

  RegisterRosService<ExecuteGroupPose>(factory, "ExecuteGroupPose", nh);
  RegisterRosService<ExecuteGroupPlan>(factory, "ExecuteGroupPlan", nh);

  RegisterRosService<ExecuteAllPoses>(factory, "ExecuteAllPoses", nh);
  RegisterRosService<ExecuteAllPlans>(factory, "ExecuteAllPlans", nh);

  auto tree = factory.createTreeFromFile(tree_file);

  RosoutLogger logger(tree.rootNode());
  printTreeRecursively(tree.rootNode());

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  while(ros::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
    ros::spinOnce();
    status = tree.tickRoot();
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }

  return 0;
}