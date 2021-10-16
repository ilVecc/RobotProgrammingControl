#!/usr/bin/env python

from __future__ import print_function

import numpy as np
import quaternion

L0 = 0.4
L1 = 0.3
L2 = 0.3
L4 = 0.5

th1_min = -2.5
th1_max = +2.5
th2_min = -2.0
th2_max = +2.0
th3_min = -3.0
th3_max = +3.0
d4_min = 0.0
d4_max = 0.45


class DH(object):

    def __init__(self):
        super(DH, self).__init__()
        self.frames = []
        self.q_names = []
        self.limits = []
    
    def add(self, d, th, a, al, limits=()):
        # matrices
        if isinstance(d, str):
            h = lambda d: self._homogeneous(d, th, a, al)
        elif isinstance(th, str):
            h = lambda th: self._homogeneous(d, th, a, al)
        else:
            h = lambda: self._homogeneous(d, th, a, al)
        self.frames.append(h)

        # joint names
        if isinstance(d, str):
            self.q_names.append(d)
        elif isinstance(th, str):
            self.q_names.append(th)
        
        if len(limits) > 2:
            raise RuntimeError("Joint limits can be none, single value or min-max values")
        if len(limits) == 1:
            l = abs(limits[0])
            limits = (-l, l)
        self.limits.append(limits)

    @staticmethod
    def _homogeneous(d, th, a, al):
        return np.array([[np.cos(th), -np.sin(th)*np.cos(al),  np.sin(th)*np.sin(al), a*np.cos(th)],
                         [np.sin(th),  np.cos(th)*np.cos(al), -np.cos(th)*np.sin(al), a*np.sin(th)],
                         [         0,             np.sin(al),             np.cos(al),            d],
                         [         0,                      0,                      0,            1]])
    
    def fk(self, q):
        assert len(q) == len(self.q_names), "Must provide %d joint values".format(len(self.q_names))
        H = np.eye(4)
        i = 0
        for h in self.frames:
            if h.func_code.co_argcount != 0:
                # check limits
                if self.limits[i] != ():
                    m, M = self.limits[i]
                    assert m <= q[i] and q[i] <= M, "%s does not comply with the limits (%f, %f)".format(self.q_names[i], m, M)
                H_i = h(q[i])
                i += 1
            else:
                H_i = h()
            H = np.matmul(H, H_i)
        return H


def h_to_pose(h):
    return quaternion.from_rotation_matrix(h[0:3, 0:3]), h[0:3,3]


def ik(x, y, z, th):

    d4 = L0 - z
    assert d4_min <= d4 <= d4_max, "d4 does not comply with the limits"

    c2 = ((x**2 + y**2) - (L1**2 + L2**2)) / (2*L1*L2)
    s2 = +np.sqrt(1 - c2**2)  # choosing + configuration
    th2 = np.arctan2(s2, c2)
    assert th2_min <= th2 <= th2_max, "th2 does not comply with the limits"

    c1 = ((L1 + L2*np.cos(th2))*x + L2*np.sin(th2)*y) / (x**2 + y**2)
    s1 = ((L1 + L2*np.cos(th2))*y - L2*np.sin(th2)*x) / (x**2 + y**2)
    th1 = np.arctan2(s1, c1)
    assert th1_min <= th1 <= th1_max, "th1 does not comply with the limits"

    th3 = th - th1 - th2
    assert th3_min <= th3 <= th3_max, "th3 does not comply with the limits"

    return th1, th2, th3, d4


def make_robot():
    dh = DH()
    dh.add(  L0,     0,  0,     0)
    dh.add(   0, "th1", L1,     0, limits=(th1_min, th1_max))
    dh.add(   0, "th2", L2,     0, limits=(th2_min, th2_max))
    dh.add(   0, "th3",  0, np.pi, limits=(th3_min, th3_max))
    dh.add("d4",     0,  0,     0, limits=( d4_min,  d4_max))
    dh.add(   0,     0,  0,     0)
    return dh


if __name__ == "__main__":

    """
    The robot has elbow singularity in most of the workspace due to the th3 joint, which allows extra control on the ee pose.

    The workspace is the classic SCARA workspace.
    """

    dh = make_robot()

    js = [0.5, 1.2, -0.3, 0.4]
    h = dh.fk(js)
    q, t = h_to_pose(h)
    print("Pose of q =", js)
    print(" q =", q)
    print(" t =", t)

    print()

    th = np.arctan2(h[0,1], h[0,0])
    x, y, z = h[0:3,3]
    th1, th2, th3, d4 = ik(x, y, z, th)
    print("th1 =", th1)
    print("th2 =", th2)
    print("th3 =", th3)
    print(" d4 =", d4)
