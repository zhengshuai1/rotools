#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from __future__ import print_function

import unittest

import rotools.interface.webots as webots_interface


class Test(unittest.TestCase):

    def test_ur_interface(self):
        """Before carrying this test, make sure the controller of the robot is
        chosen as <extern>, and the Webots simulation scene is running with a
        UR arm in scene.

        """
        ur_interface = webots_interface.WebotsInterfaceUR()
        js = ur_interface.get_joint_state()
        print('ur init joint state \n', js)

        q = [0, -0.8, 0.8, -1.57, 0.8, 0]
        ur_interface.set_joint_max_velocity(1)
        ur_interface.go_to_joint_state(q)

        js = ur_interface.get_joint_state()
        print('ur current joint state \n', js)


if __name__ == '__main__':
    unittest.main()
