#!/usr/bin/env python
from __future__ import print_function

import rospy

from roport.srv import *

# make sure setting > export PYTHONPATH="${PYTHONPATH}:~/rotools" in bashrc
import rotools.moveit.core.interface as interface


class MoveItServer(object):
    """An example RoPort server that uses the RoTools MoveIt interface to provide some
    handy services for controlling robot.
    """

    def __init__(self, kwargs):

        super(MoveItServer, self).__init__()

        self.interface = interface.MoveGroupInterface(**kwargs)

        self._srv_get_names = rospy.Service('get_all_group_names', GetAllNames, self.get_all_names_handle)
        self._srv_get_act_names = rospy.Service('get_active_group_names', GetAllNames, self.get_active_names_handle)

        self._srv_get_pose = rospy.Service('get_group_pose', GetGroupPose, self.get_pose_handle)
        self._srv_get_js = rospy.Service('get_group_joint_states', GetGroupJointStates, self.get_js_handle)

        self._srv_group_pose = rospy.Service('execute_group_pose', ExecuteGroupPose, self.group_pose_handle)
        self._srv_group_js = rospy.Service('execute_group_joint_states', ExecuteGroupJointStates, self.group_js_handle)
        self._srv_group_home = rospy.Service('execute_group_home', ExecuteGroupJointStates, self.group_home_handle)
        self._srv_group_plan = rospy.Service('execute_group_plan', ExecuteGroupPlan, self.group_plan_handle)

        self._srv_al_poses = rospy.Service('execute_all_poses', ExecuteAllPoses, self.all_poses_handle)
        # self._ajs_server = rospy.Service('execute_all_joint_states', AllJointStates, self.all_joint_states_handle)
        self._srv_all_plans = rospy.Service('execute_all_plans', ExecuteAllPlans, self.all_plans_handle)

    def get_all_names_handle(self, req):
        resp = GetAllNamesResponse()
        try:
            names = self.interface.get_all_group_names()
            resp.result_status = resp.SUCCEEDED
            resp.group_names = names
        except IndexError:
            resp.result_status = resp.FAILED
        return resp

    def get_active_names_handle(self, req):
        resp = GetAllNamesResponse()
        try:
            names = self.interface.get_active_group_names()
            resp.result_status = resp.SUCCEEDED
            resp.group_names = names
        except IndexError:
            resp.result_status = resp.FAILED
        return resp

    def get_pose_handle(self, req):
        resp = GetGroupPoseResponse()
        try:
            resp.pose = self.interface.get_current_pose_of_group(req.group_name)
            resp.ee_link, resp.ref_link = self.interface.get_frame_of_group(req.group_name)
            resp.result_status = resp.SUCCEEDED
        except IndexError:
            resp.result_status = resp.FAILED
        return resp

    def get_js_handle(self, req):
        resp = GetGroupJointStatesResponse()
        try:
            joint_names = self.interface.get_active_joint_names_of_group(req.group_name)
            joint_states = self.interface.get_joint_states_of_group(req.group_name)
            resp.joint_names = joint_names
            resp.joint_states = joint_states
            resp.result_status = resp.SUCCEEDED
        except IndexError:
            resp.result_status = resp.FAILED
        return resp

    def group_pose_handle(self, req):
        if not req.tolerance:
            req.tolerance = 0.01
        if req.is_absolute:
            ok = self.interface.group_go_to_absolute_pose_goal(req.group_name, req.goal, req.tolerance)
        else:
            ok = self.interface.group_go_to_relative_pose_goal(req.group_name, req.goal, req.tolerance)
        resp = ExecuteGroupPoseResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def group_js_handle(self, req):
        if not req.tolerance:
            req.tolerance = 0.01
        ok = self.interface.group_go_to_joint_states(req.group_name, req.goal, req.tolerance)
        resp = ExecuteGroupJointStatesResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def group_home_handle(self, req):
        if not req.tolerance:
            req.tolerance = 0.01

        goal = [0.] * len(self.interface.get_active_joint_names_of_group(req.group_name))
        ok = self.interface.group_go_to_joint_states(req.group_name, goal, req.tolerance)
        resp = ExecuteGroupJointStatesResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def group_plan_handle(self, req):
        if req.is_absolute:
            plan = self.interface.build_absolute_path_for_group(req.group_name, req.poses, req.stamp,
                                                                not req.allow_collision)
        else:
            plan = self.interface.build_relative_path_for_group(req.group_name, req.poses, req.stamp,
                                                                not req.allow_collision)

        ok = self.interface.execute_plan_for_group(req.group_name, plan)
        resp = ExecuteGroupPlanResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def all_poses_handle(self, req):
        if req.is_absolute:
            ok = self.interface.all_go_to_absolute_pose_goal(req.goals)
        else:
            ok = self.interface.all_go_to_relative_pose_goal(req.goals)
        # ok = self.interface.all_go_to_pose_goals(req.goals, req.is_absolute, req.stamps, not req.allow_collision)
        resp = ExecuteAllPosesResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def all_plans_handle(self, req):
        if req.is_absolute:
            ok = self.interface.build_absolute_paths_for_all(req.all_poses, req.stamps, not req.allow_collision)
        else:
            ok = self.interface.build_relative_paths_for_all(req.all_poses, req.stamps, not req.allow_collision)
        resp = ExecuteAllPlansResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp
