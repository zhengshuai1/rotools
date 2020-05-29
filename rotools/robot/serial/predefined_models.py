"""Predefined robot models."""
import numpy as np  # type: ignore


def kuka_lbr_iiwa_7():  # pragma: no cover
    """Get KUKA LBR iiwa 7 MDH model."""
    return np.array(
        [
            [0, 0, 0, 340],
            [-np.pi / 2, 0, 0, 0],
            [np.pi / 2, 0, 0, 400],
            [np.pi / 2, 0, 0, 0],
            [-np.pi / 2, 0, 0, 400],
            [-np.pi / 2, 0, 0, 0],
            [np.pi / 2, 0, 0, 126],
        ]
    )


def mecademic_meca500():  # pragma: no cover
    """Get Meca500 MDH model."""
    return np.array(
        [
            [0, 0, 0, 135],
            [-np.pi / 2, 0, -np.pi / 2, 0],
            [0, 135, 0, 0],
            [-np.pi / 2, 38, 0, 120],
            [np.pi / 2, 0, 0, 0],
            [-np.pi / 2, 0, np.pi, 72],
        ]
    )


def puma560():  # pragma: no cover
    """Get PUMA560 MDH model."""
    return np.array(
        [
            [0, 0, 0, 0],
            [-np.pi / 2, 0, 0, 0],
            [0, 612.7, 0, 0],
            [0, 571.6, 0, 163.9],
            [-np.pi / 2, 0, 0, 115.7],
            [np.pi / 2, 0, np.pi, 92.2],
        ]
    )


def ur10():  # pragma: no cover
    """Get UR10 MDH model."""
    return np.array(
        [
            [0, 0, 0, 0.1807],
            [np.pi / 2, 0, np.pi, 0],
            [0, 0.6127, 0, 0],
            [0, 0.57155, 0, 0.17415],
            [-np.pi / 2, 0, 0, 0.11985],
            [np.pi / 2, 0, np.pi, 0.11655],
        ]
    )


def franka_panda():
    """MDH parameters in alpha, a, theta, d order, with units be meter or radius

    note:
    alpha is z axis rotation around x axis
    a is distance along +x axis
    theta is x axis rotation around z axis
    d is distance along +z axis

    Refer: https://raw.githubusercontent.com/petercorke/robotics-toolbox-matlab/master/models/mdl_panda.m
    """
    return np.array(
        [
            [0, 0, 0, 0.333],  # theta in [-2.8973 2.8973] 0 -> 1
            [-np.pi/2, 0, 0, 0],  # theta in [-1.7628 1.7628] 1 -> 2
            [np.pi/2, 0, 0, 0.316],  # theta in [-2.8973 2.8973] 2 -> 3
            [np.pi/2, 0.0825, 0, 0],  # theta in [-3.0718 -0.0698], note the ref value of theta is 0, 3 -> 4
            [-np.pi/2, -0.0825, 0, 0.384],  # theta in [-2.8973 2.8973] 4 -> 5
            [np.pi/2, 0, 0, 0],  # theta in [-0.0175 3.7525] 5 -> 6
            [np.pi/2, 0.088, 0, 0.107],  # theta in [-2.8973 2.8973], 0.107 is distance between link 6 and 8
        ]
    )
