#!/usr/bin/env python
from __future__ import print_function

import rospy

from rotools.moveit.core.server import MoveItServer


if __name__ == "__main__":
    try:
        rospy.init_node('roport_moveit_server')
        # You only need to modify the config to apply this to new robots
        group_names = rospy.get_param('~group_names')
        ee_links = rospy.get_param('~ee_links') if rospy.has_param('~ee_links') else None
        ref_frames = rospy.get_param('~ref_frames') if rospy.has_param('~ref_frames') else None
        config = {
            'robot_description': 'robot_description',
            'ns': '',
            'group_names': group_names,
            'ee_links': ee_links,
            'ref_frames': ref_frames,
        }
        rospy.logwarn("RoPort: Configs: \n{}".format(config))
        server = MoveItServer(config)
        rospy.logwarn("RoPort: MoveIt server ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
