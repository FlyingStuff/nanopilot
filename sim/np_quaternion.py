import numpy as np


def rotation(axis, angle):
    axis_n = axis / np.linalg.norm(axis)
    qv = np.sin(angle/2) * axis_n
    return np.array([np.cos(angle/2), qv[0], qv[1], qv[2]])


def rotate_vect(v, q):
    qv = np.array([0, v[0], v[1], v[2]])
    qv_rot = mult(mult(q, qv), conj(q))
    return qv_rot[1:]


def rotation_matrix(q):
    return np.array([
        [
            q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2,
            -2*q[0]*q[3] + 2*q[1]*q[2],
            2*q[0]*q[2] + 2*q[1]*q[3]
        ],
        [
            2*q[0]*q[3] + 2*q[1]*q[2],
            q[0]**2 - q[1]**2 + q[2]**2 - q[3]**2,
            -2*q[0]*q[1] + 2*q[2]*q[3]
        ],
        [
            -2*q[0]*q[2] + 2*q[1]*q[3],
            2*q[0]*q[1] + 2*q[2]*q[3],
            q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2
        ]
    ])


def conj(q):
    qc = np.array([q[0], -q[1], -q[2], -q[3]])
    return qc


def mult(q1, q2):
    q = np.zeros(4)
    q[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]
    q[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]
    q[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]
    q[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
    return q
