from __future__ import print_function

import sys
import numpy as np

try:
    import rospy
    import moveit_commander

    import geometry_msgs.msg as GeometryMsg
    import moveit_msgs.msg as MoveItMsg
    import control_msgs.msg as ControlMsg
    import trajectory_msgs.msg as TrajectoryMsg
    import std_msgs.msg as StdMsg
    import sensor_msgs.msg as SensorMsg
except ImportError:
    pass

from rotools.utility import common


class MoveGroupInterface(object):

    def __init__(
            self,
            group_names,
            ref_frames=None,
            ee_links=None,
            verbose=False
    ):
        super(MoveGroupInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)

        self.commander = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        assert isinstance(self.group_names, list) and len(self.group_names) >= 1
        # We can get a list of all the groups in the robot:
        all_group_names = self.commander.get_group_names()
        for name in self.group_names:
            assert name in all_group_names, 'Group name {} is not exist'.format(name)

        self.group_names = group_names
        self.group_num = len(self.group_names)

        self.move_groups = []
        for name in self.group_names:
            self.move_groups.append(moveit_commander.MoveGroupCommander(name))

        # Create a `DisplayTrajectory`_ ROS publisher which is used to display
        # trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            MoveItMsg.DisplayTrajectory,
                                                            queue_size=20)

        if not ref_frames:
            self.ref_frames = []
            for group in self.move_groups:
                self.ref_frames.append(group.get_planning_frame())
        else:
            self.ref_frames = ref_frames

        if not ee_links:
            self.ee_links = []
            for group in self.move_groups:
                self.ee_links.append(group.get_end_effector_link())
        else:
            self.ee_links = ee_links

        # Sometimes for debugging it is useful to print the entire state of the robot:
        if verbose:
            print(self.commander.get_current_state())

    def _get_group_id(self, group_name):
        for i, name in enumerate(self.group_names):
            if name == group_name:
                return i
        return None

    def get_active_joint_names_of_all_groups(self):
        ret = []
        for group in self.move_groups:
            ret.append(group.get_active_joints())
        return ret

    def get_active_joint_names_of_group(self, group_name):
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        return self.move_groups[group_id].get_active_joints()

    def get_joint_states_of_all_groups(self):
        """Get joint states of all move groups

        :return: List[List[float]]
        """
        ret = []
        for group in self.move_groups:
            ret.append(group.get_current_joint_values())
        return ret

    def get_joint_states_of_group(self, group_name):
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        return self.move_groups[group_id].get_current_joint_values()

    def get_current_poses_of_all_groups(self):
        """Get the eef pose in ROS format.

        :return: List[pose_stamped]
        """
        ret = []
        for group in self.move_groups:
            ret.append(group.get_current_pose())
        return ret

    def get_current_poses_of_group(self, group_name):
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        return self.move_groups[group_id].get_current_pose()

    def group_go_to_joint_states(self, group_name, goal, tolerance=0.01):
        """Set the joint states as desired.

        :param group_name: Controlled group name
        :param goal: joint state list
        :param tolerance:
        :return:
        """
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        group = self.move_groups[group_id]

        group.go(goal, wait=True)
        group.stop()

        # For testing:
        current_joints = group.get_current_joint_values()
        return common.all_close(goal, current_joints, tolerance)

    def all_go_to_joint_states(self, goals):
        assert len(goals) == self.group_num
        for i, goal in enumerate(goals):
            if i == self.group_num - 1:
                self.move_groups[i].go(goal, wait=True)
            else:
                self.move_groups[i].go(goal, wait=False)

        for group in self.move_groups:
            group.stop()

    def _group_go_to_pose_goal(self, group_name, goal, tolerance=0.01):
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        group = self.move_groups[group_id]

        group.set_pose_target(goal)
        try:
            group.go(wait=True)
        except self.commander.MoveItCommanderException:
            group.stop()
            group.clear_pose_targets()
            return False

        group.stop()
        group.clear_pose_targets()

        current_pose = group.get_current_pose().pose
        return common.all_close(goal, current_pose, tolerance)

    def _to_absolute_pose(self, group_name, relative_pose):
        current_pose = self.get_current_poses_of_group(group_name).pose
        current_pose_mat = common.sd_pose(current_pose)
        relative_pose_mat = common.sd_pose(relative_pose)
        absolute_pose_mat = np.dot(relative_pose_mat, current_pose_mat)  # left product
        return common.to_ros_pose(absolute_pose_mat)

    def group_go_to_absolute_pose_goal(self, group_name, goal, tolerance=0.01):
        """Move group to the desired pose wrt the ref_frame

        :param group_name: Controlled group name
        :param goal: geometry_msgs.msg.Pose or PoseStamped
        :param tolerance:
        :return: if goal reached
        """
        if isinstance(goal, GeometryMsg.PoseStamped):
            goal_pose = goal.pose
        elif isinstance(goal, GeometryMsg.Pose):
            goal_pose = goal
        else:
            raise NotImplementedError

        return self._group_go_to_pose_goal(group_name, goal_pose, tolerance)

    def group_go_to_relative_pose_goal(self, group_name, goal, tolerance=0.01):
        if isinstance(goal, GeometryMsg.PoseStamped):
            goal_pose = goal.pose
        elif isinstance(goal, GeometryMsg.Pose):
            goal_pose = goal
        else:
            raise NotImplementedError

        abs_goal = self._to_absolute_pose(group_name, goal_pose)
        return self._group_go_to_pose_goal(group_name, abs_goal, tolerance)

    def _all_go_to_pose_goal(self, goals):
        for i, goal in enumerate(goals):
            self.move_groups[i].set_pose_target(goal)
            if i == self.group_num - 1:
                self.move_groups[i].go(wait=True)
            else:
                self.move_groups[i].go(wait=False)

        for group in self.move_groups:
            group.stop()
            group.clear_pose_targets()

    def all_go_to_absolute_pose_goal(self, goals):
        assert len(goals) == self.group_num
        self._all_go_to_pose_goal(goals)

    def all_go_to_relative_pose_goal(self, goals):
        assert len(goals) == self.group_num
        abs_goals = []
        for name, goal in zip(self.group_names, goals):
            abs_goals.append(self._to_absolute_pose(name, goal))

        self._all_go_to_pose_goal(abs_goals)

    @staticmethod
    def _build_plan(plan, stamps):
        for i, point in enumerate(plan.joint_trajectory.points):
            if i == 0:
                assert point.time_from_start == 0
                continue
            point.time_from_start = stamps[i - 1]
        return plan

    def build_absolute_path_for_group(self, group_name, poses, stamps=None):
        """Given way points in a list of geometry msg pose, plan a path
        go through all way points.

        :param group_name
        :param poses: List[Pose]
        :param stamps: time from start
        """
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        group = self.move_groups[group_id]
        plan, fraction = group.compute_cartesian_path(poses, 0.01, 0.0)

        if stamps:
            assert len(poses) == len(stamps)
            plan = self._build_plan(plan, stamps)

        # curr_state = self.commander.get_current_state()
        # group.retime_trajectory(curr_state, plan, )
        return plan

    def build_relative_path_for_group(self, group_name, poses, stamps=None):
        abs_poses = self._to_absolute_pose(group_name, poses)
        return self.build_absolute_path_for_group(group_name, abs_poses, stamps)

    def group_execute_plan(self, group_name, plan):
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        self.move_groups[group_id].execute(plan, wait=True)

    def build_absolute_paths_for_all(self, all_poses, all_stamps=None):
        all_plans = []
        if all_stamps:
            assert len(all_poses) == len(all_stamps)
        else:
            all_stamps = [None] * len(all_poses)

        for i, poses in enumerate(all_poses):
            group_name = self.group_names[i]
            plan = self.build_absolute_path_for_group(group_name, poses, all_stamps[i])
            all_plans.append(plan)

        return all_plans

    def build_relative_paths_for_all(self, all_poses, all_stamps=None):
        all_abs_poses = []
        for i, poses in enumerate(all_poses):
            group_name = self.group_names[i]
            abs_poses = self._to_absolute_pose(group_name, poses)
            all_abs_poses.append(abs_poses)

        return self.build_absolute_paths_for_all(all_abs_poses, all_stamps)

    def all_execute_plan(self, plans):
        assert len(plans) == self.group_num
        for i, plan in enumerate(plans):
            group = self.move_groups[i]
            if i == len(plans) - 1:
                group.execute(plan, wait=True)
            else:
                group.execute(plan, wait=False)

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
