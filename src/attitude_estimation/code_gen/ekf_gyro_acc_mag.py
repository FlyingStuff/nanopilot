"""
Extended Kalman Filter derivations for a simple attitude estimation model

 estimates:
 - attitude (quaternion)

 based on:
 - gyro
 - accelerometer ("up" sensor)
 - magnetometer ("north" sensor)

 model:
 - direct gyro "control" input for state propagation
 - no acceleration -> only gravity which is used to compensate for gyro drift (measurement input)
 - magnetometer projected on horizontal plane points north (direction x+)
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

x = attitude.col_join(gyro_bias)
x = attitude
display("x", x)

# gyro
gx, gy, gz = sp.symbols("gx gy gz", real=True)
gyro = sp.Matrix([[gx], [gy], [gz]])
u = gyro

rate = gyro - gyro_bias
rate = gyro
display("rate", rate)

rate_quat = rate.row_insert(0, sp.Matrix([0]))

attitude_dot = 1/2*quaternion.mult(attitude, rate_quat) # quaternion kinemamtics
gyro_bias_dot = sp.zeros(3, 1) # constant gyro bias model
gyro_bias_dot = - gyro_bias * 0.1

x_dot = attitude_dot.col_join(gyro_bias_dot)
x_dot = attitude_dot
x_dot = sp.simplify(x_dot)

display("x_dot", x_dot)

f = x_dot * delta_t + x
F = f.jacobian(x)

display("f", f)
display("F", F)


# acc measurement

expected_meas = sp.Matrix([0, 0, 9.81])
h_acc = quaternion.rotate_vect(expected_meas, quaternion.conj(attitude))
H_acc = h_acc.jacobian(x)

display("h_acc", h_acc)
display("H_acc", H_acc)


# mag measurement

north = sp.Matrix([1, 0, 0])
r0, r1, r2, r3 = sp.symbols("r0 r1 r2 r3", real=True)
ref = sp.Matrix([[r0], [r1], [r2], [r3]])
north_vect_in_ref = quaternion.rotate_vect(north, quaternion.mult(ref, quaternion.conj(attitude)))
h_mag = sp.Matrix([sp.atan2(north_vect_in_ref[1], north_vect_in_ref[0])])
H_mag = h_mag.jacobian(x)
ref_subs = list(zip([r0, r1, r2, r3], [q0, q1, q2, q3])) + [(q0**2 + q1**2 + q2**2 + q3**2, 1)]

h_mag = h_mag.subs(ref_subs)
H_mag = H_mag.subs(ref_subs)
display("h_mag", h_mag)
display("H_mag", H_mag)

z0, z1, z2 = sp.symbols("z0 z1 z2", real=True)
z = sp.Matrix([[z0], [z1], [z2]])
z_inertial = quaternion.rotate_vect(z, attitude)
measurement_transform = sp.Matrix([sp.atan2(z_inertial[1], z_inertial[0])])
display('measurement_transform', measurement_transform)


if __name__ == "__main__":
    c_code = 'const int STATE_DIM = {};\n'.format(len(x))
    c_code += 'const int CONTROL_DIM = {};\n'.format(len(u))
    c_code += 'const int MEASURE_DIM = {};\n'.format(len(h_mag))
    c_code += '\n\n'

    c_code += c_code_gen.generate_c_func('f', f, [('x', x), ('u', u), ('delta_t', delta_t)])
    c_code += c_code_gen.generate_c_func('F', F, [('x', x), ('u', u), ('delta_t', delta_t)])
    c_code += c_code_gen.generate_c_func('h', h_mag, [('x', x)])
    c_code += c_code_gen.generate_c_func('H', H_mag, [('x', x)])
    c_code += c_code_gen.generate_c_func('meas_transf', measurement_transform, [('attitude', attitude), ('z', z)])
    c_code += c_code_gen.generate_c_func('z_inertial', z_inertial, [('attitude', attitude), ('z', z)])

    c_code_gen.write_file('ekf_gyro_mag.h', c_code)
