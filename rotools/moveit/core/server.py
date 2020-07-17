#!/usr/bin/env python
from __future__ import print_function

import rospy

# make sure setting > export PYTHONPATH="${PYTHONPATH}:~/rotools" in bashrc
import rotools.moveit.core.interface as interface


class MoveItServer(object):

    def __init__(
            self,
            kwargs,
    ):
        super(MoveItServer, self).__init__()
        self.interface = interface.MoveGroupInterface(**kwargs)

    def group_pose_handle(self, req):
        raise NotImplementedError

    def group_js_handle(self, req):
        raise NotImplementedError

    def group_home_handle(self, req):
        raise NotImplementedError

    def all_poses_handle(self, req):
        pass

    @staticmethod
    def all_joint_states_handle(msg):
        if msg.is_relative:
            pass
        else:
            pass

    def group_plan_handle(self, req):
        raise NotImplementedError

    @staticmethod
    def all_plans_handle(msg):
        if msg.is_relative:
            pass
        else:
            pass

