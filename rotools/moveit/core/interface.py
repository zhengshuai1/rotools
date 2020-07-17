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
            robot_description,
            ns,
            group_names,
            ref_frames=None,
            ee_links=None,
            verbose=False
    ):
        super(MoveGroupInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        print(robot_description, ns, group_names)

        self.commander = moveit_commander.RobotCommander(robot_description, ns)
        # self.scene = moveit_commander.PlanningSceneInterface(ns=ns)

        assert isinstance(group_names, list)
        self.group_names = group_names
        self.group_num = len(self.group_names)
        assert self.group_num >= 1

        # We can get a list of all the groups in the robot:
        all_group_names = self.commander.get_group_names()
        for name in self.group_names:
            assert name in all_group_names, 'Group name {} is not exist'.format(name)

        self.move_groups = []
        for name in self.group_names:
            self.move_groups.append(moveit_commander.MoveGroupCommander(name))

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

    def get_current_pose_of_group(self, group_name):
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        return self.move_groups[group_id].get_current_pose().pose

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

        current_pose = common.regularize_pose(group.get_current_pose().pose)
        return common.all_close(goal, current_pose, tolerance)

    def _to_absolute_pose(self, group_name, relative_pose):
        current_pose = self.get_current_pose_of_group(group_name)
        current_pose_mat = common.sd_pose(current_pose)
        relative_pose_mat = common.sd_pose(relative_pose)
        absolute_pose_mat = np.dot(current_pose_mat, relative_pose_mat)  # T_b1 * T_12 = T_b2
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
    def _update_plan_time_stamps(plan, stamp):
        points_num = len(plan.joint_trajectory.points)
        t = stamp / float(points_num)
        for i in range(points_num):
            plan.joint_trajectory.points[i].time_from_start = rospy.Duration.from_sec(t * (i + 1))
        return plan

    def build_absolute_path_for_group(self, group_name, poses, stamp=None, eef_step=0.01, avoid_collisions=True):
        """Given way points in a list of geometry_msgs.Pose, plan a path
        go through all way points.

        :param group_name: Group name for building plan
        :param poses: List[Pose]
        :param stamp: Last time stamp from start
        :param eef_step:
        :param avoid_collisions:
        """
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        group = self.move_groups[group_id]
        plan, fraction = group.compute_cartesian_path(poses, eef_step, jump_threshold=0.0,
                                                      avoid_collisions=avoid_collisions)

        if stamp:
            plan = self._update_plan_time_stamps(plan, stamp)

        # curr_state = self.commander.get_current_state()
        # group.retime_trajectory(curr_state, plan, )
        return plan

    def build_relative_path_for_group(self, group_name, poses, stamp=None, eef_step=0.01, avoid_collisions=True):
        abs_poses = []
        for pose in poses:
            abs_poses.append(self._to_absolute_pose(group_name, pose))
        return self.build_absolute_path_for_group(group_name, abs_poses, stamp, eef_step, avoid_collisions)

    def execute_plan_for_group(self, group_name, plan):
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        return self.move_groups[group_id].execute(plan, wait=True)

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

    def execute_plans_for_all(self, plans):
        assert len(plans) == self.group_num
        for i, plan in enumerate(plans):
            group = self.move_groups[i]
            if i == len(plans) - 1:
                group.execute(plan, wait=True)
            else:
                group.execute(plan, wait=False)
