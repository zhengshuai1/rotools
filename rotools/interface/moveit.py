# Note that this is just a example for interacting with MoveIt in ROS

from __future__ import print_function

import sys

try:
    import rospy
    import moveit_commander

    import geometry_msgs.msg as GeometryMsg
    import moveit_msgs.msg as MoveItMsg
    import std_msgs.msg as StdMsg
    import sensor_msgs.msg as SensorMsg
except ImportError:
    pass

from rotools.utility import common


class MoveGroupInterface(object):

    def __init__(self):
        super(MoveGroupInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)

        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        # kinematic model and the robot's current joint states
        self.commander = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the
        # surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        # to a planning group (group of joints).  In this tutorial the group is the primary
        # arm joints in the Panda robot, so we set the group's name to "panda_arm".
        # If you are using a different robot, change this value to the name of your robot
        # arm planning group.
        # This interface can be used to plan and execute motions:
        self.group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # Create a `DisplayTrajectory`_ ROS publisher which is used to display
        # trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            MoveItMsg.DisplayTrajectory,
                                                            queue_size=20)

        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.move_group.get_planning_frame()

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.move_group.get_end_effector_link()

        # We can get a list of all the groups in the robot:
        group_names = self.commander.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the robot:
        print(self.commander.get_current_state())

        # Misc variables
        self.attached_object_name = ''

    def get_active_joint_names(self):
        return self.move_group.get_active_joints()

    def get_joint_state(self):
        """Get joint states of the robot

        :return: List[float]
        """
        return self.move_group.get_current_joint_values()

    def get_current_pose(self):
        """Get the eef pose in ROS format.

        :return: pose_stamped
        """
        return self.move_group.get_current_pose()

    def go_to_joint_state(self, joint_goal):
        """Set the joint states as desired.

        :param joint_goal: joint state list
        :return:
        """
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return common.all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, pose_goal):
        """Set the eef pose as desired.

        :param pose_goal: geometry_msgs.msg.Pose()
        :return: if in goal pose
        """
        self.move_group.set_pose_target(pose_goal)

        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return common.all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, waypoints):
        """Given way points in a list of geometry msg pose, plan a path
        go through all way points.

        :param waypoints: List[Pose]
        :param

        example
        # wpose = self.move_group.get_current_pose().pose
        # wpose.position.z -= 0.1  # First move up (z)
        # wpose.position.y += 0.2  # and sideways (y)
        # waypoints.append(copy.deepcopy(wpose))
        #
        # wpose.position.x += 0.1  # Second move forward/backwards in (x)
        # waypoints.append(copy.deepcopy(wpose))
        #
        # wpose.position.y -= 0.1  # Third move sideways (y)
        # waypoints.append(copy.deepcopy(wpose))
        """

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def display_trajectory(self, plan):
        # You can ask RViz to visualize a plan (aka trajectory) for you. But the
        # group.plan() method does this automatically so this is not that useful
        # here (it just displays the same trajectory again):
        # A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        # We populate the trajectory_start with our current robot state to copy over
        # any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = MoveItMsg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.commander.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)

    def wait_for_state_update(self, name_is_known=False, obj_is_attached=False, timeout=4):
        # If the Python node dies before publishing a collision object update message, the message
        # could get lost and the box will not appear. To ensure that the updates are
        # made, we wait until we see the changes reflected in the
        # ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        # For the purpose of this tutorial, we call this function after adding,
        # removing, attaching or detaching an object in the planning scene. We then wait
        # until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([self.attached_object_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = self.attached_object_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (obj_is_attached == is_attached) and (name_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.attached_object_name
        scene = self.scene

        # First, we will create a box in the planning scene at the location of the left finger:
        box_pose = GeometryMsg.PoseStamped()
        box_pose.header.frame_id = "panda_leftfinger"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.07  # slightly above the end effector
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.attached_object_name = box_name
        return self.wait_for_state_update(name_is_known=True, timeout=timeout)

    def add_sphere(self, base_link, position, radius, timeout=4):
        self.attached_object_name = 'sphere'

        sphere_pose = GeometryMsg.PoseStamped()
        sphere_pose.header.frame_id = base_link
        sphere_pose.pose.orientation.w = 1.0
        sphere_pose.pose.position.x = position[0]
        sphere_pose.pose.position.y = position[1]
        sphere_pose.pose.position.z = position[2]
        self.scene.add_sphere("sphere", sphere_pose, radius)

        return self.wait_for_state_update(name_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.attached_object_name
        robot = self.commander
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        # Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        # robot be able to touch them without the planning scene reporting the contact as a
        # collision. By adding link names to the ``touch_links`` array, we are telling the
        # planning scene to ignore collisions between those links and the box. For the Panda
        # robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        # you should change this value to the name of your end effector group name.
        grasping_group = 'hand'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(obj_is_attached=True, name_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.attached_object_name
        scene = self.scene
        eef_link = self.eef_link

        # We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(name_is_known=True, obj_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.attached_object_name
        scene = self.scene
        # We can remove the box from the world.
        scene.remove_world_object(box_name)

        # **Note:** The object must be detached before we can remove it from the world
        # We wait for the planning scene to update.
        return self.wait_for_state_update(obj_is_attached=False, name_is_known=False, timeout=timeout)

    def remove_obj(self, timeout=4):
        self.scene.remove_world_object(self.attached_object_name)
        return self.wait_for_state_update(obj_is_attached=False, name_is_known=False, timeout=timeout)


class WalkerMoveGroupInterface(object):

    def __init__(self, group_name, home_pose_name):
        super(WalkerMoveGroupInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)

        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        # kinematic model and the robot's current joint states
        self.commander = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the
        # surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = group_name
        self.home_pose_name = home_pose_name
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.move_group.get_planning_frame()
        print('planning frame:', self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.move_group.get_end_effector_link()

        # We can get a list of all the groups in the robot:
        group_names = self.commander.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the robot:
        print(self.commander.get_current_state())

    def get_active_joint_names(self):
        return self.move_group.get_active_joints()

    def get_joint_state(self):
        """Get joint states of the robot

        :return: List[float]
        """
        return self.move_group.get_current_joint_values()

    def get_current_pose(self):
        """Get the eef pose in ROS format.

        :return: pose_stamped
        """
        return self.move_group.get_current_pose()

    def set_start_state(self, joint_state):
        """Specifically set the start state

        :param joint_state: sensor_msgs JointState, the header, name, and
        position must be given.
        """

        moveit_robot_state = MoveItMsg.RobotState()
        moveit_robot_state.joint_state = joint_state
        return self.move_group.set_start_state(moveit_robot_state)

    def go_to_joint_state(self, joint_goal):
        """Set the joint states as desired.

        :param joint_goal: joint state list
        :return:
        """
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return common.all_close(joint_goal, current_joints, 0.01)

    def go_home(self):
        """Reset to home pose."""
        self.move_group.set_named_target(self.home_pose_name)
        self.move_group.go()

    def go_to_pose_goal(self, pose_goal):
        """Set the eef pose as desired.

        :param pose_goal: geometry_msgs.msg.Pose()
        :return: if in goal pose
        """
        self.move_group.set_pose_target(pose_goal)

        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return common.all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, waypoints):
        """Given way points in a list of geometry msg pose, plan a path
        go through all way points.

        :param waypoints: List[Pose]
        :param

        example
        # wpose = self.move_group.get_current_pose().pose
        # wpose.position.z -= 0.1  # First move up (z)
        # wpose.position.y += 0.2  # and sideways (y)
        # waypoints.append(copy.deepcopy(wpose))
        #
        # wpose.position.x += 0.1  # Second move forward/backwards in (x)
        # waypoints.append(copy.deepcopy(wpose))
        #
        # wpose.position.y -= 0.1  # Third move sideways (y)
        # waypoints.append(copy.deepcopy(wpose))
        """

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)

    def go_though_joint_state(self, joint_state):
        """Go through a series of joint states."""
        self.move_group.set_pose_targets()
