#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from __future__ import print_function

import unittest
import math
import numpy as np

import rotools.utility.transform as transform


class Test(unittest.TestCase):

    def test_transform(self):
        """
        """
        R = transform.rotation_matrix(20 * np.pi / 180, (1, 0, 0))
        print(np.array2string(R, separator=', '))

        sin_20 = math.sin(20. / 180 * np.pi)  # 20. makes sure the result is a float
        cos_20 = math.cos(20. / 180 * np.pi)

        homogeneous_matrix = np.array([
            [0., 0., 1., 0.084415],
            [1., 0., 0., 0.],
            [0., 1., 0, 0.098503+0.093313],
            [0., 0., 0., 1.]
        ], dtype=float)
        q = transform.quaternion_from_matrix(homogeneous_matrix)
        print(q)

    def test_2d_rotation(self):
        R = transform.rotation_matrix(-20 * np.pi / 180, (0, 0, 1))
        v0_t = np.dot(R, np.array([1, 2, 0, 1]).T)
        v1_t = np.dot(R, np.array([1, -2, 0, 1]).T)
        v2_t = np.dot(R, np.array([-1, -2, 0, 1]).T)
        v3_t = np.dot(R, np.array([-1, 2, 0, 1]).T)
        print(v0_t, v1_t, v2_t, v3_t)


if __name__ == '__main__':
    unittest.main()
