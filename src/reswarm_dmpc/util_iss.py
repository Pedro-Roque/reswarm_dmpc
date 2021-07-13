"""
Set of utility functions used across the package.
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import math
import numpy as np


def r_mat_np(q):
    """
    Generate rotation matrix from unit quaternion

    :param q: unit quaternion
    :type q: ca.MX
    :return: rotation matrix, SO(3)
    :rtype: ca.MX
    """

    Rmat = np.zeros((3, 3))

    # Extract states
    qx = q[0]
    qy = q[1]
    qz = q[2]
    qw = q[3]

    Rmat[0, 0] = 1 - 2 * qy**2 - 2 * qz**2
    Rmat[0, 1] = 2 * qx * qy - 2 * qz * qw
    Rmat[0, 2] = 2 * qx * qz + 2 * qy * qw

    Rmat[1, 0] = 2 * qx * qy + 2 * qz * qw
    Rmat[1, 1] = 1 - 2 * qx**2 - 2 * qz**2
    Rmat[1, 2] = 2 * qy * qz - 2 * qx * qw

    Rmat[2, 0] = 2 * qx * qz - 2 * qy * qw
    Rmat[2, 1] = 2 * qy * qz + 2 * qx * qw
    Rmat[2, 2] = 1 - 2 * qx**2 - 2 * qy**2

    return Rmat


class Vec(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    @staticmethod
    def dot(v1, v2):
        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z

    @staticmethod
    def cross(v1, v2):
        x = v1.y * v2.z - v1.z * v2.y
        y = v1.x * v2.x - v1.x * v2.z
        z = v1.z * v2.y - v1.y * v2.x
        return Vec(x, y, z)

    @staticmethod
    def normalize(v):
        return v / v.norm()

    def norm(self):
        return math.sqrt(Vec.dot(self, self))

    def __add__(self, v):
        return Vec(self.x + v.x, self.y + v.y, self.z + v.z)

    def __neg__(self, v):
        return Vec(-self.x, -self.y, -self.z)

    def __sub__(self, v):
        return self + (-v)

    def __mul__(self, v):
        if isinstance(v, Vec):
            return Vec(self.x * v.x, self.y * v.y, self.z * v.z)
        else:
            return Vec(self.x * v, self.y * v, self.z * v)

    def __rmul__(self, v):
        return self.__mul__(v)

    def __div__(self, v):
        if isinstance(v, Vec):
            return Vec(self.x / v.x, self.y / v.y, self.z / v.z)
        else:
            return Vec(self.x / v, self.y / v, self.z / v)

    def __str__(self):
        return '[ %.4f, %.4f, %.4f ]' % (self.x, self.y, self.z)
