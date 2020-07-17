#!/usr/bin/env python
from __future__ import print_function

import rospy

from roport.srv import *

# make sure setting > export PYTHONPATH="${PYTHONPATH}:~/rotools" in bashrc
from rotools.moveit.core.server import MoveItServer


class ROSMoveItServer(MoveItServer):
    """An example RoPort server that uses the RoTools MoveIt interface to provide some
    handy services for controlling robot. This example is based on franka_ros and panda_moveit_config
    """

    def __init__(self, kwargs):

        super(ROSMoveItServer, self).__init__(kwargs)

        self._srv_get_pose = rospy.Service('get_group_pose', GetGroupPose, self.get_pose_handle)
        self._srv_group_pose = rospy.Service('execute_group_pose', ExecuteGroupPose, self.group_pose_handle)

        self._srv_get_js = rospy.Service('get_group_js', GetGroupJointStates, self.get_js_handle)
        self._srv_group_js = rospy.Service('execute_group_joint_states', ExecuteGroupJointStates, self.group_js_handle)
        self._srv_group_home = rospy.Service('execute_group_home', ExecuteGroupJointStates, self.group_home_handle)

        # self._ap_server = rospy.Service('execute_all_poses', AllPoses, self.all_poses_handle)
        # self._ajs_server = rospy.Service('execute_all_joint_states', AllJointStates, self.all_joint_states_handle)
        self._srv_group_plan = rospy.Service('execute_group_plan', ExecuteGroupPlan, self.group_plan_handle)
        # self._apl_server = rospy.Service('execute_all_plans', AllPlans, self.all_plans_handle)

    def get_pose_handle(self, req):
        resp = GetGroupPoseResponse()
        try:
            pose = self.interface.get_current_pose_of_group(req.group_name)
            resp.pose = pose
            resp.result_status = resp.SUCCEEDED
        except IndexError:
            resp.result_status = resp.FAILED
        return resp

    def get_js_handle(self, req):
        resp = GetGroupJointStatesResponse()
        try:
            joint_states = self.interface.get_joint_states_of_group(req.group_name)
            resp.joint_states = joint_states
            resp.result_status = resp.SUCCEEDED
        except IndexError:
            resp.result_status = resp.FAILED
        return resp

    def group_pose_handle(self, req):
        if not req.tolerance:
            req.tolerance = 0.01
        if req.is_relative:
            ok = self.interface.group_go_to_relative_pose_goal(req.group_name, req.goal, req.tolerance)
        else:
            ok = self.interface.group_go_to_absolute_pose_goal(req.group_name, req.goal, req.tolerance)
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
        if not req.eef_step:
            req.eef_step = 0.01

        if req.is_relative:
            plan = self.interface.build_relative_path_for_group(req.group_name, req.poses, req.stamp,
                                                                req.eef_step, req.avoid_collisions)
        else:
            plan = self.interface.build_absolute_path_for_group(req.group_name, req.poses, req.stamp,
                                                                req.eef_step, req.avoid_collisions)

        ok = self.interface.execute_plan_for_group(req.group_name, plan)
        resp = ExecuteGroupPlanResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp


if __name__ == "__main__":
    try:
        rospy.init_node('RoPort_moveit_server')
        # You only need to modify the config to apply this to new robots
        config = {
            'robot_description': 'robot_description',
            'ns': '',
            'group_names': ['panda_arm'],
        }
        server = ROSMoveItServer(config)
        rospy.logwarn("Server ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
