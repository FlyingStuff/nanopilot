import sympy as sp


def rotate_vect(v, q):
    qv = sp.Matrix([0]).col_join(v)
    qv_rot = mult(mult(q, qv), conj(q))
    qv_rot.row_del(0)
    return qv_rot


def conj(q):
    qc = sp.Matrix([q[0], -q[1], -q[2], -q[3]])
    return qc


def mult(q1, q2):
    q = q1.copy()
    q[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]
    q[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]
    q[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]
    q[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
    return q
