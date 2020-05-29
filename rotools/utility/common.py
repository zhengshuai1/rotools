import numpy as np
import rospy

import geometry_msgs.msg as GeometryMsg
import moveit_msgs.msg as MoveItMsg
import trajectory_msgs.msg as TrajectoryMsg

from moveit_commander.conversions import pose_to_list

from rotools.utility import transform


def all_close(goal, actual, tolerance):
    """Test if a list of values are within a tolerance of their counterparts in another list.

    :param: goal A list of floats, a Pose or a PoseStamped
    :param: actual     A list of floats, a Pose or a PoseStamped
    :param: tolerance  A float
    :returns: bool
    """
    if type(goal) is list:
        if not np.allclose(goal, actual, atol=tolerance):
            return False

    elif type(goal) is GeometryMsg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is GeometryMsg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


def sd_joint_state():
    pass


def sd_pose(pose):
    """Standardize the input pose to a 4x4 transformation matrix.

    :param pose:
    :return: transformation matrix
    """
    if isinstance(pose, np.ndarray):
        if pose.ndim == 1 and pose.size == 7:
            t = pose[:3]
            q = pose[3:]
            tm = transform.translation_matrix(t)
            rm = transform.quaternion_matrix(q)
            return np.dot(rm, tm)
        elif pose.ndim == 1 and pose.size == 6:
            t = pose[:3]
            rpy = pose[3:]
            tm = transform.translation_matrix(t)
            rm = transform.euler_matrix(rpy[0], rpy[1], rpy[2])
            return np.dot(rm, tm)
        elif pose.shape == (4, 4):
            return pose
        else:
            raise NotImplementedError
    elif isinstance(pose, list):
        return sd_pose(np.array(pose))
    elif isinstance(pose, GeometryMsg.Pose):
        p = pose.position
        o = pose.orientation
        return sd_pose(np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w]))
    elif isinstance(pose, GeometryMsg.PoseStamped):
        p = pose.pose.position
        o = pose.pose.orientation
        return sd_pose(np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w]))
    else:
        raise NotImplementedError


def ros_plan(t, p, v=None, a=None):
    """Convert a plan to ros MoveItMsg.RobotTrajectory msg.
    Note that the msg contains no joint name or header, which need
    to be added explicitly.

    :param t: timestamp of shape N
    :param p: way point positions of shape [dim, N]
    :param v: way point velocities of shape [dim, N]
    :param a: way point accelerations of shape [dim, N], if both p and v are given, a could be 0
    :return: MoveItMsg.RobotTrajectory
    """
    msg = MoveItMsg.RobotTrajectory()
    way_point_num = t.size
    dim = p.shape[0]
    zero_list = np.zeros(dim).tolist()

    for w in range(way_point_num):
        if w == 0:
            continue
        wpt = TrajectoryMsg.JointTrajectoryPoint()

        wpt.positions = list(p[:, w])
        wpt.velocities = zero_list if v is None else list(v[:, w])
        wpt.accelerations = zero_list if a is None else list(a[:, w])
        wpt.time_from_start = rospy.Duration.from_sec(t[w])
        msg.joint_trajectory.points.append(wpt)
    return msg
