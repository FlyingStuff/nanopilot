"""
Extended Kalman Filter derivations for a simple attitude estimation model

 estimates:
 - attitude (quaternion)

 based on:
 - gyro
 - accelerometer ("up" sensor)

 model:
 - direct gyro "control" input for state propagation
 - no acceleration -> only gravity which is used to compensate for gyro drift (measurement input)
"""

import logging
import quaternion
import c_code_gen
import sympy as sp
import argparse


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', '--loglevel', default='WARNING')
    args = parser.parse_args()
    logging.basicConfig(level=args.loglevel.upper())


def display(name, val):
    logging.debug("{} = {}".format(name, val))


delta_t = sp.symbols("Delta_t", real=True)

q0, q1, q2, q3 = sp.symbols("q0 q1 q2 q3", real=True)
attitude = sp.Matrix([[q0], [q1], [q2], [q3]])

gbx, gby, gbz = sp.symbols("gbx, gby, gbz", real=True)
gyro_bias = sp.Matrix([[gbx], [gby], [gbz]])

# x = attitude.col_join(gyro_bias)
x = attitude
display("x", x)

# gyro
gx, gy, gz = sp.symbols("gx gy gz", real=True)
gyro = sp.Matrix([[gx], [gy], [gz]])
u = gyro

# rate = gyro - gyro_bias
rate = gyro
display("rate", rate)

rate_quat = rate.row_insert(0, sp.Matrix([0]))

attitude_dot = 1/2*quaternion.mult(attitude, rate_quat) # quaternion kinemamtics
# gyro_bias_dot = sp.zeros(3, 1) # constant gyro bias model
gyro_bias_dot = - gyro_bias * 0.01

# x_dot = attitude_dot.col_join(gyro_bias_dot)
x_dot = attitude_dot
x_dot = sp.simplify(x_dot)

display("x_dot", x_dot)

f = x_dot * delta_t + x
F = f.jacobian(x)

display("f", f)
display("F", F)


expected_meas = sp.Matrix([0, 0, 9.81])
h = quaternion.rotate_vect(expected_meas, quaternion.conj(attitude))
H = h.jacobian(x)

display("h", h)
display("H", H)


if __name__ == "__main__":
    c_code = 'const int STATE_DIM = {};\n'.format(len(x))
    c_code += 'const int CONTROL_DIM = {};\n'.format(len(u))
    c_code += 'const int MEASURE_DIM = {};\n'.format(len(h))
    c_code += '\n\n'

    c_code += c_code_gen.generate_c_func('f', f, [('x', x), ('u', u), ('delta_t', delta_t)])
    c_code += c_code_gen.generate_c_func('F', F, [('x', x), ('u', u), ('delta_t', delta_t)])
    c_code += c_code_gen.generate_c_func('h', h, [('x', x)])
    c_code += c_code_gen.generate_c_func('H', H, [('x', x)])

    c_code_gen.write_file('ekf_gyro_acc.h', c_code)
