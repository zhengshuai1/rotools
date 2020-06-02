"""Predefined robot models."""
import numpy as np  # type: ignore


def kuka_lbr_iiwa7():  # pragma: no cover
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


def ur10e_mdh_model():
    """Get UR10e MDH model."""

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


def ur10e_poe_model():
    """Get UR10e product of exponential model."""

    M = np.array([
        [-1, 0, 0, 1.18425],  # L1+L2
        [0, 0, 1, 0.2907],  # W1+W2
        [0, 1, 0, 0.06085],  # H1-H2
        [0, 0, 0, 1],
    ])

    screw_axes = np.array([
        [0, 0, 1, 0, 0, 0],
        [0, 1, 0, -0.1807, 0, 0],  # -H1
        [0, 1, 0, -0.1807, 0, 0.6127],  # -H1 L1
        [0, 1, 0, -0.1807, 0, 1.18425],  # -H1 L1+L2
        [0, 0, -1, -0.17415, 1.18425, 0],  # -W1 L1+L2
        [0, 1, 0, -0.06085, 0, 1.18425],  # H2-H1 L1+L2
    ])

    return M, screw_axes


def ur10_poe_model():
    """Get UR10 product of exponential model."""

    M = np.array([
        [-1, 0, 0, 1.1843],  # L1+L2
        [0, 0, 1, 0.256141],  # W1+W2
        [0, 1, 0, 0.0116],  # H1-H2
        [0, 0, 0, 1],
    ])

    screw_axes = np.array([
        [0, 0, 1, 0, 0, 0],
        [0, 1, 0, -0.1273, 0, 0],  # -H1
        [0, 1, 0, -0.1273, 0, 0.612],  # -H1 L1
        [0, 1, 0, -0.1273, 0, 1.1843],  # -H1 L1+L2
        [0, 0, -1, -0.163941, 1.1843, 0],  # -W1 L1+L2
        [0, 1, 0, -0.0116, 0, 1.1843],  # H2-H1 L1+L2
    ])

    return M, screw_axes


def ur5_poe_model():
    """Get ur5 product of exponential model."""

    M = np.array([
        [-1, 0, 0, 0.81725],  # L1+L2
        [0, 0, 1, 0.19145],  # W1+W2
        [0, 1, 0, -0.005491],  # H1-H2
        [0, 0, 0, 1],
    ])

    screw_axes = np.array([
        [0, 0, 1, 0, 0, 0],
        [0, 1, 0, -0.089159, 0, 0],
        [0, 1, 0, -0.089159, 0, 0.425],
        [0, 1, 0, -0.089159, 0, 0.81725],
        [0, 0, -1, -0.10915, 0.81725, 0],
        [0, 1, 0, 0.005491, 0, 0.81725],
    ]).T

    return M, screw_axes


def ur5e_poe_model():
    """Get ur5e product of exponential model."""

    M = np.array([
        [-1, 0, 0, 0.8172],
        [0, 0, 1, 0.2329],
        [0, 1, 0, 0.0628],
        [0, 0, 0, 1],
    ])

    screw_axes = np.array([
        [0, 0, 1, 0, 0, 0],
        [0, 1, 0, -0.1625, 0, 0],
        [0, 1, 0, -0.1625, 0, 0.425],
        [0, 1, 0, -0.1625, 0, 0.8172],
        [0, 0, -1, -0.1333, 0.8172, 0],
        [0, 1, 0, -0.0628, 0, 0.8172],
    ]).T

    return M, screw_axes


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
            [-np.pi / 2, 0, 0, 0],  # theta in [-1.7628 1.7628] 1 -> 2
            [np.pi / 2, 0, 0, 0.316],  # theta in [-2.8973 2.8973] 2 -> 3
            [np.pi / 2, 0.0825, 0, 0],  # theta in [-3.0718 -0.0698], note the ref value of theta is 0, 3 -> 4
            [-np.pi / 2, -0.0825, 0, 0.384],  # theta in [-2.8973 2.8973] 4 -> 5
            [np.pi / 2, 0, 0, 0],  # theta in [-0.0175 3.7525] 5 -> 6
            [np.pi / 2, 0.088, 0, 0.107],  # theta in [-2.8973 2.8973], 0.107 is distance between link 6 and 8
        ]
    )
