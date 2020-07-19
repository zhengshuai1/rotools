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
    ):
        super(MoveGroupInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        print(robot_description, ns, group_names)

        self.commander = moveit_commander.RobotCommander(robot_description, ns)
        # self.scene = moveit_commander.PlanningSceneInterface(ns=ns)

        assert isinstance(group_names, list), 'group_names should be list, but got {}'.format(type(group_names))
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
            assert len(ref_frames) == self.group_num
            self.ref_frames = ref_frames
            for i, group in enumerate(self.move_groups):
                group.set_pose_reference_frame(ref_frames[i])

        if not ee_links:
            self.ee_links = []
            for group in self.move_groups:
                self.ee_links.append(group.get_end_effector_link())
        else:
            assert len(ee_links) == self.group_num
            self.ee_links = ee_links
            for i, group in enumerate(self.move_groups):
                group.set_end_effector_link(self.ee_links[i])

        # Sometimes for debugging it is useful to print the entire state of the robot:
        print(self.commander.get_current_state())

    def get_all_group_names(self):
        return self.commander.get_group_names()

    def get_active_group_names(self):
        return self.group_names

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

        :return: List[PoseStamped]
        """
        ret = []
        for group in self.move_groups:
            ret.append(group.get_current_pose())
        return ret

    def get_current_pose_of_group(self, group_name):
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        return self.move_groups[group_id].get_current_pose().pose

    def get_frame_of_group(self, group_name):
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        return self.ee_links[group_id], self.ref_frames[group_id]

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
        current_joints = group.get_current_joint_values()
        return common.all_close(goal, current_joints, tolerance)

    def all_go_to_joint_states(self, goals):
        assert len(goals) == self.group_num
        for i, goal in enumerate(goals):
            if i == self.group_num - 1:
                ok = self.move_groups[i].go(goal, wait=True)
            else:
                ok = self.move_groups[i].go(goal, wait=False)
            if not ok:
                return False
        for group in self.move_groups:
            group.stop()
        return True

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

    def _to_absolute_pose(self, group_name, relative_pose, init_pose=None):
        """Convert a pose wrt eef to base_link.

        :param group_name: String Planning group name
        :param relative_pose: Pose or List[float]
        :param init_pose: Initial pose of the robot before moving relatively
        :return: Pose
        """
        if not init_pose:
            current_pose = self.get_current_pose_of_group(group_name)
        else:
            current_pose = init_pose
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
            raise NotImplementedError('Goal of type {} not defined'.format(type(goal)))

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
        plans = []
        for i, goal in enumerate(goals):
            self.move_groups[i].set_pose_target(goal)
            plan = self.move_groups[i].plan()
            plans.append(plan)

            # if i == self.group_num - 1:
            #     ok = self.move_groups[i].go(wait=False)
            # else:
            #     ok = self.move_groups[i].go(wait=False)
            # print(i, ok)
            # if not ok:
            #     return False
        for plan, group in zip(plans, self.move_groups):
            group.execute(plan, wait=False)
            # group.stop()
            # group.clear_pose_targets()
        return True

    def all_go_to_absolute_pose_goal(self, goals):
        if isinstance(goals, GeometryMsg.PoseArray):
            goals = goals.poses
        assert len(goals) == self.group_num
        return self._all_go_to_pose_goal(goals)

    def all_go_to_relative_pose_goal(self, goals):
        if isinstance(goals, GeometryMsg.PoseArray):
            goals = goals.poses
        assert len(goals) == self.group_num
        abs_goals = []
        for name, goal in zip(self.group_names, goals):
            abs_goals.append(self._to_absolute_pose(name, goal))
        return self._all_go_to_pose_goal(abs_goals)

    def _update_plan_time_stamps(self, group, plan, stamp):
        original_stamp = plan.joint_trajectory.points[-1].time_from_start.to_sec()
        # points_num = len(plan.joint_trajectory.points)
        velocity_scale = original_stamp / stamp
        acceleration_scale = velocity_scale
        print('velocity_scale ', velocity_scale)
        curr_state = self.commander.get_current_state()
        updated_plan = group.retime_trajectory(curr_state, plan, velocity_scale, acceleration_scale)
        # for i in range(points_num):
        #     plan.joint_trajectory.points[i].time_from_start = rospy.Duration.from_sec(t * (i + 1))
        return updated_plan

    def build_absolute_path_for_group(self, group_name, poses, stamp=None, avoid_collisions=True):
        """Given way points in a list of geometry_msgs.Pose, plan a path
        go through all way points.

        :param group_name: Group name for building plan
        :param poses: geometry_msgs.PoseArray or List[Pose]
        :param stamp: Last time stamp from start
        :param avoid_collisions:
        """
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        group = self.move_groups[group_id]
        if isinstance(poses, GeometryMsg.PoseArray):
            poses = poses.poses
        plan, fraction = group.compute_cartesian_path(poses, eef_step=0.01, jump_threshold=0,
                                                      avoid_collisions=avoid_collisions)
        # move_group_interface.h  L754
        if fraction < 0:
            rospy.logerr('RoPort: Path planning failed.')
        # if stamp:
        #     plan = self._update_plan_time_stamps(group, plan, stamp)
        return plan

    def build_relative_path_for_group(self, group_name, poses, stamp=None, avoid_collisions=True):
        """Build a path composed by relative poses for the planning group.

        :param group_name: String Planning group name
        :param poses: PoseArray or List[Pose] Each pose is relevant to the last one
        :param stamp: Double Time stamp for the last pose
        :param avoid_collisions:
        :return:
        """
        if isinstance(poses, GeometryMsg.PoseArray):
            poses = poses.poses
        init_pose = None
        abs_poses = []
        for rel_pose in poses:
            abs_pose = self._to_absolute_pose(group_name, rel_pose, init_pose)
            abs_poses.append(abs_pose)
            init_pose = abs_pose
        return self.build_absolute_path_for_group(group_name, abs_poses, stamp, avoid_collisions)

    def execute_plan_for_group(self, group_name, plan, wait=True):
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        return self.move_groups[group_id].execute(plan, wait)

    def build_absolute_paths_for_all(self, all_poses, all_stamps=None, avoid_collisions=True):
        """

        :param all_poses: List[List[Pose]]
        :param all_stamps:
        :param avoid_collisions:
        :return:
        """
        all_plans = []
        if all_stamps:
            assert len(all_poses) == len(all_stamps)
        else:
            all_stamps = [None] * len(all_poses)
        for i, poses in enumerate(all_poses):
            group_name = self.group_names[i]
            plan = self.build_absolute_path_for_group(group_name, poses, all_stamps[i], avoid_collisions)
            all_plans.append(plan)
        return all_plans

    def build_relative_paths_for_all(self, all_poses, all_stamps=None, avoid_collisions=True):
        all_abs_poses = []
        for i, rel_poses_of_group in enumerate(all_poses):
            group_name = self.group_names[i]
            init_pose = None
            abs_poses_of_group = []
            for rel_pose in rel_poses_of_group:
                abs_pose = self._to_absolute_pose(group_name, rel_pose, init_pose)
                abs_poses_of_group.append(abs_pose)
                init_pose = abs_pose
            all_abs_poses.append(abs_poses_of_group)
        return self.build_absolute_paths_for_all(all_abs_poses, all_stamps, avoid_collisions)

    def execute_plans_for_all(self, plans):
        assert len(plans) == self.group_num
        for i, plan in enumerate(plans):
            group = self.move_groups[i]
            if i == len(plans) - 1:
                ok = group.execute(plan, wait=True)
            else:
                ok = group.execute(plan, wait=False)
            if not ok:
                return False
        return True

    def all_go_to_pose_goals(self, goals, is_absolute, stamps=None, avoid_collision=True):
        if isinstance(goals, GeometryMsg.PoseArray):
            goals = goals.poses
        assert len(goals) == self.group_num
        for i, goal in enumerate(goals):
            group_name = self.group_names[i]
            try:
                stamp = stamps[i] if stamps else None
            except IndexError:
                stamp = None
            if is_absolute:
                print(goal)
                plan = self.build_absolute_path_for_group(group_name, [goal], stamp, avoid_collision)
            else:
                plan = self.build_relative_path_for_group(group_name, [goal], stamp, avoid_collision)
            if i == self.group_num - 1:
                ok = self.execute_plan_for_group(group_name, plan, wait=True)
            else:
                ok = self.execute_plan_for_group(group_name, plan, wait=False)
            if not ok:
                return False
        return True
