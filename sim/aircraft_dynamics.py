from scipy.integrate import odeint
import numpy as np
import math
from math import sin, cos, exp, copysign, pi
from numpy.linalg import inv
import scipy.constants
import np_quaternion as quaternion
from functools import reduce
import operator


def passive_yaw_pitch_roll_sequence_q(yaw, pitch, roll):
    q_z = quaternion.rotation(np.array([0, 0, 1]), -yaw)
    q_y = quaternion.rotation(np.array([0, 1, 0]), -pitch)
    q_x = quaternion.rotation(np.array([1, 0, 0]), -roll)
    return quaternion.mult(q_x, quaternion.mult(q_y, q_z))


def passive_yaw_pitch_roll_sequence_R(yaw, pitch, roll):
    q = passive_yaw_pitch_roll_sequence_q(yaw, pitch, roll)
    return quaternion.rotation_matrix(q)


class DynamicsModel:

    def time_step(self, delta_t, inputs):
        def odeint_fn(state_vector, t, self, inputs):
            state = self.unpack_state_vector(state_vector)
            derivative = self.cont_time_model(state, t, inputs)
            return self.pack_state_vector(derivative)

        t = [0, delta_t]
        x = self.pack_state_vector(self.state)
        res = odeint(odeint_fn, x, t, (self, inputs))
        self.state = self.unpack_state_vector(res[1])
        return self.state

    def pack_state_vector(self, state):
        state_vector = np.array([])
        for name, shape in self.state_vector_composition:
            l = reduce(operator.mul, shape, 1)
            state_vector = np.concatenate((state_vector, state[name].reshape(l)))
        return state_vector

    def unpack_state_vector(self, state_vector):
        state = {}
        for name, shape in self.state_vector_composition:
            l = reduce(operator.mul, shape, 1)
            state[name] = state_vector[:l].reshape(shape)
            state_vector = state_vector[l:]
        return state

    def get_zero_state_vector(self):
        n = sum([reduce(operator.mul, shape, 1) for name, shape in self.state_vector_composition])
        return np.zeros(n)


class RigidBody:
    states = (
        ('pos_I', (3, 1)),  # position in inertial NED frame
        ('vel_I', (3, 1)),  # vel_I: velocity in inertial NED frame
        ('q_b_to_I', (4, )),  # attitude, transforms a vector from body to intertial frame
        ('omega_b', (3, 1)),  # angular rate of body with respect to inerital in body frame
    )

    @staticmethod
    def d_dt_pos_I(state):
        return state['vel_I']

    @staticmethod
    def d_dt_vel_I(state, mass, force_b):
        R_b_to_I = quaternion.rotation_matrix(state['q_b_to_I'])
        force_I = R_b_to_I.dot(force_b)
        return force_I / mass

    @staticmethod
    def d_dt_q_b_to_I(state):
        w = state['omega_b']
        wq = np.array([0, w[0], w[1], w[2]])
        return quaternion.mult(state['q_b_to_I'], 1/2*wq)

    @staticmethod
    def d_dt_omega_b(state, inertia_b, torque_b):
        w = state['omega_b']
        Ldot = torque_b - np.cross(w, inertia_b.dot(w), axis=0)
        return inv(inertia_b).dot(Ldot)


def d_dt_pos_servo_actuator_rate_limit(pos, setpt, param):
    if (setpt > pos and pos < param['max']):
        return np.array(param['rate'])
    if (setpt < pos and pos > param['min']):
        return np.array(-param['rate'])
    return np.array(0)


class Airplane(DynamicsModel):

    state_vector_composition = (
        RigidBody.states +
        (
            ('d_a', (1,)),
            ('d_e', (1,)),
            ('d_r', (1,)),
        )
    )

    @staticmethod
    def airspeed_b(q_b_to_I, groundspeed_I, windspeed_I):
        airspeed_I = groundspeed_I - windspeed_I
        R_b_to_I = quaternion.rotation_matrix(q_b_to_I)
        R_I_to_b = R_b_to_I.T
        airspeed_b = np.dot(R_I_to_b, airspeed_I)
        return airspeed_b

    @staticmethod
    def airspeed_aoa_aos(airspeed_b):
        airspeed_b_x, airspeed_b_y, airspeed_b_z = list(airspeed_b.reshape(3))
        airspeed = np.linalg.norm(airspeed_b)
        aoa = np.arctan2(airspeed_b_z, airspeed_b_x)
        aos = np.arctan2(airspeed_b_y, np.sqrt(airspeed_b_x**2 + airspeed_b_z**2))
        if airspeed_b_x <= 0:
            aos = aoa = float('nan')
        return airspeed, aoa, aos

    @staticmethod
    def C_L(param, aoa, q_normalized, d_e):
        flat_plate_lift = 2*copysign(1, aoa)*sin(aoa)**2*cos(aoa)
        linear_lift = param['C_L_0'] + param['C_L_alpha']*aoa
        aoa_max = param['aoa_max']
        M = param['aoa_max_M']
        sigmoid_n = 1 + exp(-M*(aoa-aoa_max)) + exp(M*(aoa+aoa_max))
        sigmoid_d = (1 + exp(-M*(aoa-aoa_max))) * (1 + exp(M*(aoa+aoa_max)))
        sigmoid = sigmoid_n / sigmoid_d
        C_L_of_alpha = (1 - sigmoid) * linear_lift + sigmoid * flat_plate_lift
        return C_L_of_alpha + param['C_L_q']*q_normalized + param['C_L_d_e']*d_e

    @staticmethod
    def C_D(param, aoa, q_normalized, d_e):
        lift = param['C_L_0'] + param['C_L_alpha']*aoa
        AR = param['b']**2/param['S']
        C_D_of_alpha = param['C_D_p'] + lift**2/(pi*param['e_C_D']*AR)
        return C_D_of_alpha + param['C_D_q']*q_normalized + param['C_D_d_e']*d_e

    @staticmethod
    def C_Y(param, aos, p_normalized, r_normalized, d_a, d_r):
        return (param['C_Y_0'] + param['C_Y_beta']*aos
                + param['C_Y_p']*p_normalized + param['C_Y_r']*r_normalized
                + param['C_Y_d_a']*d_a + param['C_Y_d_r']*d_r)

    @staticmethod
    def aerodynamic_force_b(param, rho, airspeed_b, omega_b, ctrl_surf):
        V_a, aoa, aos = Airplane.airspeed_aoa_aos(airspeed_b)
        p, q, r = list(omega_b.reshape(3))
        p_normalized = p*param['b']/2*V_a
        q_normalized = q*param['c']/2*V_a
        r_normalized = r*param['b']/2*V_a
        C_L = Airplane.C_L(param, aoa, q_normalized, ctrl_surf['d_e'])
        C_D = Airplane.C_D(param, aoa, q_normalized, ctrl_surf['d_e'])
        C_Y = Airplane.C_Y(param, aos, p_normalized, r_normalized, ctrl_surf['d_a'], ctrl_surf['d_r'])
        F_L = 1/2*rho*V_a**2*param['S']*C_L
        F_D = 1/2*rho*V_a**2*param['S']*C_D
        F_Y = 1/2*rho*V_a**2*param['S']*C_Y
        force_b = np.array([[- cos(aoa)*F_D + sin(aoa)*F_L],
                            [F_Y],
                            [-sin(aoa)*F_D - cos(aoa)*F_L]])
        return force_b

    @staticmethod
    def C_m(param, aoa, q_normalized, d_e):
        return (param['C_m_0'] + param['C_m_alpha']*aoa
                + param['C_m_q']*q_normalized + param['C_m_d_e']*d_e)

    @staticmethod
    def C_l(param, aos, p_normalized, r_normalized, d_a, d_r):
        return (param['C_l_0'] + param['C_l_beta']*aos
                + param['C_l_p']*p_normalized + param['C_l_r']*r_normalized
                + param['C_l_d_a']*d_a + param['C_l_d_r']*d_r)

    @staticmethod
    def C_n(param, aos, p_normalized, r_normalized, d_a, d_r):
        return (param['C_n_0'] + param['C_n_beta']*aos
                + param['C_n_p']*p_normalized + param['C_n_r']*r_normalized
                + param['C_n_d_a']*d_a + param['C_n_d_r']*d_r)

    @staticmethod
    def aerodynamic_torque_b(param, rho, airspeed_b, omega_b, ctrl_surf):
        V_a, aoa, aos = Airplane.airspeed_aoa_aos(airspeed_b)
        p, q, r = list(omega_b.reshape(3))
        p_normalized = p*param['b']/2*V_a
        q_normalized = q*param['c']/2*V_a
        r_normalized = r*param['b']/2*V_a
        C_m = Airplane.C_m(param, aoa, q_normalized, ctrl_surf['d_e'])
        C_l = Airplane.C_l(param, aos, p_normalized, r_normalized, ctrl_surf['d_a'], ctrl_surf['d_r'])
        C_n = Airplane.C_n(param, aos, p_normalized, r_normalized, ctrl_surf['d_a'], ctrl_surf['d_r'])
        l = 1/2*rho*param['S']*param['b']*C_l
        m = 1/2*rho*param['S']*param['c']*C_m
        n = 1/2*rho*param['S']*param['b']*C_n
        torque_b = np.array([[l], [m], [n]])
        return torque_b

    def aerodynamics_from_state(self, state, inputs):
        airspeed_b = Airplane.airspeed_b(state['q_b_to_I'], state['vel_I'], inputs['wind_I'])
        rho = 1.225 # todo
        ctrl_surf = {k: state[k][0] for k in ['d_e', 'd_a', 'd_r']}
        force_b = Airplane.aerodynamic_force_b(self.param['aero'], rho, airspeed_b,
                                               state['omega_b'], ctrl_surf)
        torque_b = Airplane.aerodynamic_torque_b(self.param['aero'], rho, airspeed_b,
                                                 state['omega_b'], ctrl_surf)
        return force_b, torque_b

    def __init__(self, param, state_init={}):
        self.state = self.unpack_state_vector(self.get_zero_state_vector())
        self.state['q_b_to_I'] = np.array([1, 0, 0, 0])
        self.state.update(state_init)

        self.param = param

    def cont_time_model(self, state, t, inputs):
        d_dt = {}

        gravity_I = np.array([[0], [0], [scipy.constants.g]])
        R_b_to_I = quaternion.rotation_matrix(state['q_b_to_I'])
        R_I_to_b = R_b_to_I.T
        gravity_b = np.dot(R_I_to_b, gravity_I)

        aero_force_b, aero_torque_b = self.aerodynamics_from_state(state, inputs)
        force_b = aero_force_b + gravity_b  # + thrust
        torque_b = aero_torque_b  # + thrust
        d_dt['pos_I'] = RigidBody.d_dt_pos_I(state)
        d_dt['vel_I'] = RigidBody.d_dt_vel_I(state, self.param['mass'], force_b)
        d_dt['q_b_to_I'] = RigidBody.d_dt_q_b_to_I(state)
        d_dt['omega_b'] = RigidBody.d_dt_omega_b(state, self.param['I'], torque_b)

        for a in ['d_a', 'd_e', 'd_r']:
            d_dt[a] = d_dt_pos_servo_actuator_rate_limit(state[a], inputs[a], self.param['ctrl_surf'][a])

        return d_dt







import unittest
from math import radians


class AirplaneTest(unittest.TestCase):

    def test_airspeed_b_at_zero_attitude(self):
        q_b_to_I_zero = np.array([1, 0, 0, 0])
        groundspeed_I = np.array([[100, 200, 300]]).T
        windspeed_I = np.array([[1, 2, 3]]).T
        airspeed_b = Airplane.airspeed_b(q_b_to_I_zero, groundspeed_I, windspeed_I)
        expected_airspeed_b = np.array([[99, 198, 297]]).T
        np.testing.assert_array_almost_equal(expected_airspeed_b, airspeed_b)

    def test_airspeed_b_at_90deg_nose_up(self):
        q_I_to_b_90deg_pitch = passive_yaw_pitch_roll_sequence_q(0, radians(90), 0)
        q_b_to_I_90deg_pitch = quaternion.conj(q_I_to_b_90deg_pitch)
        groundspeed_I = np.array([[100, 0, 0]]).T
        windspeed_I = np.array([[1, 0, 0]]).T
        airspeed_b = Airplane.airspeed_b(q_b_to_I_90deg_pitch, groundspeed_I, windspeed_I)
        expected_airspeed_b = np.array([[0, 0, 99]]).T
        np.testing.assert_array_almost_equal(expected_airspeed_b, airspeed_b)

    def test_zero_aoa_aos(self):
        airspeed_b = np.array([[10, 0, 0]]).T
        s, aoa, aos = Airplane.airspeed_aoa_aos(airspeed_b)
        self.assertEqual(10, s)
        self.assertEqual(0, aoa)
        self.assertEqual(0, aos)

    def test_positive_45deg_aoa(self):
        airspeed_b = np.array([[10, 0, 10]]).T
        s, aoa, aos = Airplane.airspeed_aoa_aos(airspeed_b)
        self.assertAlmostEqual(np.sqrt(10**2+10**2), s)
        self.assertAlmostEqual(radians(45), aoa)
        self.assertAlmostEqual(0, aos)

    def test_positive_45deg_aos(self):
        airspeed_b = np.array([[10, 10, 0]]).T
        s, aoa, aos = Airplane.airspeed_aoa_aos(airspeed_b)
        self.assertAlmostEqual(np.sqrt(10**2+10**2), s)
        self.assertAlmostEqual(0, aoa)
        self.assertAlmostEqual(radians(45), aos)

    def test_negative_45deg_aoa(self):
        airspeed_b = np.array([[10, 0, -10]]).T
        s, aoa, aos = Airplane.airspeed_aoa_aos(airspeed_b)
        self.assertAlmostEqual(np.sqrt(10**2+10**2), s)
        self.assertAlmostEqual(radians(-45), aoa)
        self.assertAlmostEqual(0, aos)

    def test_negative_45deg_aos(self):
        airspeed_b = np.array([[10, -10, 0]]).T
        s, aoa, aos = Airplane.airspeed_aoa_aos(airspeed_b)
        self.assertAlmostEqual(np.sqrt(10**2+10**2), s)
        self.assertAlmostEqual(0, aoa)
        self.assertAlmostEqual(radians(-45), aos)

    def test_reverse_aoa_aos_is_nan(self):
        airspeed_b = np.array([[-1, 10, 10]]).T
        s, aoa, aos = Airplane.airspeed_aoa_aos(airspeed_b)
        self.assertEqual(np.linalg.norm(airspeed_b), s)
        self.assertTrue(math.isnan(aoa))
        self.assertTrue(math.isnan(aoa))

    def test_C_L_linear(self):
        aoa = 0.1
        q_normalized = 0.2
        d_e = -0.1
        param = {'C_L_0': 1, 'C_L_alpha': 1, 'aoa_max': 1, 'aoa_max_M': 10,
                 'C_L_q': 2, 'C_L_d_e': 1}
        cl = Airplane.C_L(param, aoa, q_normalized, d_e)
        exp_cl = (param['C_L_0'] + aoa*param['C_L_alpha']
                  + q_normalized * param['C_L_q'] + d_e * param['C_L_d_e'])
        self.assertAlmostEqual(exp_cl, cl, 3)

    def test_C_D(self):
        aoa = 0.1
        q_normalized = 0.2
        d_e = -0.1
        param = {'C_L_0': 1, 'C_L_alpha': 1, 'C_D_p': 0.2, 'C_D_q': 0.1,
                 'C_D_d_e': 0.3, 'b': 2, 'S': 0.2, 'e_C_D': 0.9}
        cl = param['C_L_0'] + param['C_L_alpha']*aoa
        AR = param['b']**2/param['S']
        cd = Airplane.C_D(param, aoa, q_normalized, d_e)
        exp_cd = (param['C_D_p'] + cl**2/(pi * AR * param['e_C_D'])
                  + q_normalized * param['C_D_q'] + d_e * param['C_D_d_e'])
        self.assertAlmostEqual(exp_cd, cd)

    def test_C_Y(self):
        aos = -0.1
        p_normalized = 0.5
        r_normalized = 0.1
        d_a = 0.2
        d_r = 0.3
        param = {'C_Y_0': 0.11, 'C_Y_beta': 0.12, 'C_Y_p': 0.13, 'C_Y_r': 0.14,
                 'C_Y_d_a': 0.15, 'C_Y_d_r': 0.16}
        cy = Airplane.C_Y(param, aos, p_normalized, r_normalized, d_a, d_r)
        exp_cy = (param['C_Y_0'] + param['C_Y_beta']*aos
                  + param['C_Y_p']*p_normalized + param['C_Y_r']*r_normalized
                  + param['C_Y_d_a']*d_a + param['C_Y_d_r']*d_r)
        self.assertEqual(exp_cy, cy)

    def test_aerodynamic_force_b(self):
        param = {'C_L_0': 0.1, 'C_L_alpha': 0.1, 'aoa_max': 1,
                 'aoa_max_M': 100, 'C_L_q': 0.1, 'C_L_d_e': 0.1,
                 'C_L_alpha': 0.1, 'C_D_p': 0.1, 'e_C_D': 0.1, 'C_D_q': 0.1,
                 'C_D_d_e': 0.1, 'C_Y_0': 0., 'C_Y_beta': 0.1, 'C_Y_p': 0.1,
                 'C_Y_r': 0.1, 'C_Y_d_a': 0.1, 'C_Y_d_r': 0.1,
                 'b': 2, 'S': 1, 'c': 0.5}
        rho = 1
        V_a = 100
        airspeed_b = np.array([[V_a], [0], [0]])
        omega_b = np.array([[0], [0], [0]])
        ctrl_surf = {'d_e': 0, 'd_a': 0, 'd_r': 0}
        f = Airplane.aerodynamic_force_b(param, rho, airspeed_b, omega_b, ctrl_surf)
        C_L = param['C_L_0']
        AR = param['b']**2/param['S']
        C_D = param['C_D_p'] + C_L**2 / (pi * param['e_C_D'] * AR)
        exp_f = np.array([[-C_D * 1/2 * rho*param['S']*V_a**2],
                          [0],
                          [-C_L * 1/2 * rho*param['S']*V_a**2]])
        np.testing.assert_array_almost_equal(exp_f, f, 4)


class test_state_vector_packing(unittest.TestCase):

    def test_pack(self):
        m = DynamicsModel()
        m.state_vector_composition = (
            ('x', (3, 1)),
            ('y', (3, 1)),
            ('M', (3, 3)),
        )
        x = np.array([[1], [2], [3]])
        y = np.array([[4], [5], [6]])
        M = np.array([[10, 11, 12],
                      [13, 14, 15],
                      [16, 17, 18]])
        exp = np.array([1, 2, 3, 4, 5, 6, 10, 11, 12, 13, 14, 15, 16, 17, 18])
        packed = m.pack_state_vector({'x': x, 'y': y, 'M': M})
        np.testing.assert_array_equal(exp, packed)

    def test_pack_quaternion(self):
        m = DynamicsModel()
        m.state_vector_composition = (
            ('q', (4,)),
        )
        q = np.array([1, 0, 0, 0])
        exp = np.array([1, 0, 0, 0])
        packed = m.pack_state_vector({'q': q})
        np.testing.assert_array_equal(exp, packed)

    def test_unpack(self):
        m = DynamicsModel()
        m.state_vector_composition = (
            ('x', (3, 1)),
            ('y', (3, 1)),
            ('M', (3, 3)),
        )
        packed = np.array([1, 2, 3, 4, 5, 6, 10, 11, 12, 13, 14, 15, 16, 17, 18])
        unpacked = m.unpack_state_vector(packed)
        np.testing.assert_array_equal(unpacked['x'], np.array([[1], [2], [3]]))
        np.testing.assert_array_equal(unpacked['y'], np.array([[4], [5], [6]]))
        np.testing.assert_array_equal(unpacked['M'], np.array([[10, 11, 12],
                                                               [13, 14, 15],
                                                               [16, 17, 18]]))

    def test_unpack_quaternion(self):
        m = DynamicsModel()
        m.state_vector_composition = (
            ('q', (4,)),
        )
        q = np.array([1, 0, 0, 0])
        packed = np.array([1, 0, 0, 0])
        unpacked = m.unpack_state_vector(packed)
        np.testing.assert_array_equal(unpacked['q'], np.array([1, 0, 0, 0]))











param = {
    'aero': {
        # C_L
        'C_L_0': 0.1,
        'C_L_alpha': 1,
        'aoa_max': radians(30),
        'aoa_max_M': 10,
        'C_L_q': 0,
        'C_L_d_e': 0,
        # C_D
        'C_D_p': 0.1,
        'e_C_D': 0.9,
        'C_D_q': 0.2,
        'C_D_d_e': 0.1,
        # C_Y
        'C_Y_0': 0,
        'C_Y_beta': 1,
        'C_Y_p': 1,
        'C_Y_r': 1,
        'C_Y_d_a': 1,
        'C_Y_d_r': 1,
        # C_m
        'C_m_0': 1,
        'C_m_alpha': -20,
        'C_m_q': -0.01,
        'C_m_d_e': 1,
        # C_l
        'C_l_0': 0,
        'C_l_beta': 0,
        'C_l_p': -1,
        'C_l_r': -1,
        'C_l_d_a': 1,
        'C_l_d_r': 1,
        # C_n
        'C_n_0': 0,
        'C_n_beta': -1,
        'C_n_p': -1,
        'C_n_r': -1,
        'C_n_d_a': 1,
        'C_n_d_r': 1,
        'S': 1,  # wing area
        'c': 0.2,  # mean aerodynamic cord
        'b': 5,  # wing span
    },
    'mass': 1,
    'I': np.array([[0.2, 0, 0],
                   [0, 0.2, 0],
                   [0, 0, 0.2]]),
    'ctrl_surf': {
        'd_e': {'min': -1, 'max': 1, 'rate': 2},
        'd_a': {'min': -1, 'max': 1, 'rate': 2},
        'd_r': {'min': -1, 'max': 1, 'rate': 2},
    }
}
init = {
    'vel_I': np.array([[15], [0], [0]])
}
# a = Airplane(param, init)
# inputs = {'d_e': 0, 'd_a': 0, 'd_r': 0, 'wind_I': np.array([[0], [0], [0]])}
# for i in range(100):
#     print(a.state)
#     a.time_step(0.01, inputs)
