#!/usr/bin/env python
from __future__ import print_function

import rospy

from rotools.moveit.core.server import MoveItServer


if __name__ == "__main__":
    try:
        rospy.init_node('roport_moveit_server')
        # You only need to modify the config to apply this to new robots
        config = {
            'robot_description': 'robot_description',
            'ns': '',
            'group_names': ['panda_arm', 'hand'],
        }
        server = MoveItServer(config)
        rospy.logwarn("RoPort: MoveIt server ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
