from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import casadi as ca
import numpy as np
import numpy.matlib as nmp
from reswarm_dmpc.util import *


class Astrobee(object):
    def __init__(self,
                 iface='casadi',
                 mass=7,
                 inertia=np.diag([0.1083, 0.1083, 0.1083]),
                 h=0.01,
                 **kwargs):
        """
        Astrobee Robot, NMPC tester class.

        :param mass: mass of the Astrobee
        :type mass: float
        :param inertia: inertia tensor of the Astrobee
        :type inertia: np.diag
        :param h: sampling time of the discrete system, defaults to 0.01
        :type h: float, optional
        """

        # Model
        self.solver = iface
        self.nonlinear_model = self.astrobee_dynamics
        self.model = None
        self.n = 13
        self.m = 6
        self.dt = h

        # Model prperties
        self.mass = mass
        self.inertia = inertia

        # Set single agent properties
        self.set_casadi_options()
        self.set_dynamics()

        # Check additional role parameters:
        if 'role' in kwargs:
            # Check role validity
            assert kwargs['role'] in ['leader', 'follower', 'local_leader'],\
                   "Wrong role. Can be 'leader', 'follower' or 'local_leader'."
            self.role = kwargs['role']

            # Check existence of vmax parameter
            assert 'vmax' in kwargs, "Missing 'vmax' for agent role"
            assert type(kwargs['vmax']) is float, "Wrong data-type for 'vmax'.\
                                                   Should be float."
            self.neighbours_vmax = kwargs['vmax']

            # Check existence of number of neighbours paramter
            assert 'num_neighbours' in kwargs, "Missing number of neighbors"
            assert type(kwargs['num_neighbours']) is int, "Wrong data type for\
                   'num_neighbors'. Should be int."
            assert kwargs['num_neighbours'] > 0, "Invalid number of numbers.\
                   Should be bigger than 0."
            self.num_neighbors = kwargs['num_neighbours']

            # Define neighbors dynamics depending on the role
            if kwargs['role'] == 'leader':
                # prepare leader dynamics
                self.set_follower_dynamics()
            elif kwargs['role'] == 'follower':
                # prepare follower dynamics
                self.set_local_leader_dynamics()
            elif kwargs['role'] == 'local_leader':
                # prepare dynamics for leader and follower
                self.set_local_leader_dynamics()
                self.set_follower_dynamics()

            # Get formation geometry. Formation geometry is defined as the
            # relative position of all the neighbors in the local frame of
            # the Astrobee. The geometry is passed as a np.ndarray with
            # shape (3,n), with 'n' being the number neighbours, starting
            # on the local leader (if it exists) and finishing on the bottom
            # right agent. The formation is assumed to be a binary tree-shape.
            # Leader is always the first agent (1).
            # Ex:         Star-shape
            #
            #                 (1)              Line-Shape
            #                  |
            #                  O               (1)--O--(2)
            #                 / \
            #               (2) (3)
            #
            assert 'fg' in kwargs, "Missing formation geometry param 'fg'."
            assert np.shape(kwargs['fg'])[0] == 3 and \
                   np.shape(kwargs['fg'])[1] == self.num_neighbors, \
                   "Wrong formation geometry shape. Should be 3 x #neighbours"
            self.fg = kwargs['fg']

    def set_casadi_options(self):
        """
        Helper function to set casadi options.
        """
        self.fun_options = {
            "jit": False,
            "jit_options": {"flags": ["-O2"]}
        }

    def set_dynamics(self):
        """
        Helper function to populate Astrobee's dynamics.
        """
        if self.solver == 'acados':
            self.x = ca.vertcat(ca.MX.sym('x', self.n, 1))
            self.u = ca.vertcat(ca.MX.sym('u', self.m, 1))
            self.xdot = ca.vertcat(ca.MX.sym('xdot', self.n, 1))
            self.name = "astrobee"

            self.f_expl = self.astrobee_dynamics(self.x, self.u)
            self.f_impl = self.xdot - self.f_expl
            self.z = []
            self.disc_dyn_expr = self.rk4_integrator(self.astrobee_dynamics)
        else:
            self.model = self.rk4_integrator(self.astrobee_dynamics)

        return

    def set_follower_dynamics(self):
        """
        Helper function to set local follower dynamics in the formation.
        Applies the prediction model in  @TODO(Pedro-Roque): ref to paper.
        """

        v = ca.MX.sym('v', 3, 1)
        rpos0 = ca.MX.sym('rpos', 3, 1)
        rposD = ca.MX.sym('rposD', 3, 1)
        next_follower_rpos = self.follower_dynamics(v, rpos0, rpos0)
        self.next_follower_rpos = ca.Function('FollowerRPos',
                                              [v, rpos0, rposD],
                                              [next_follower_rpos],
                                              self.fun_options)

    def set_local_leader_dynamics(self):
        """
        Helper function to set local leader dynamics in the formation.
        Applies the prediction model in  @TODO(Pedro-Roque): ref to paper.
        """

        v = ca.MX.sym('v', 3, 1)
        q = ca.MX.sym('q', 4, 1)
        rpos0 = ca.MX.sym('rpos', 3, 1)
        vL = ca.MX.sym('vL', 3, 1)
        next_leader_rpos = self.local_leader_dynamics(q, v, rpos0, vL)
        self.next_local_leader_rpos = ca.Function('LeaderRPos',
                                                  [v, q, rpos0, vL],
                                                  [next_leader_rpos],
                                                  self.fun_options)

    def follower_dynamics(self, v, rpos, rposD):
        """
        Predict follower position based on a relative position 'rpos'
        measurement and desired relative position.

        :param v: agent velocity
        :type v: ca.MX, np.ndarray
        :param rpos: relative position vector, 3x1
        :type rpos: ca.MX , np.ndarray
        :param rposD: desired relative position of follower
        :type rposD: ca.MX , np.ndarray
        """
        vmax = self.neighbours_vmax
        alpha = 1  # Exponential decay
        eps = 0.0001  # Epsilon for zero singularity

        e_rpos = rposD - rpos
        e_vf = vmax*(1.0 - ca.exp(-alpha*ca.norm_2(e_rpos)))
        next_rpos = rpos + ((e_rpos)/(ca.norm_2(e_rpos)+eps))*(e_vf - ca.norm_2(v))

        return next_rpos

    def local_leader_dynamics(self, q, v, rpos, vL):
        """
        Predict local leader position based on a relative position 'rpos'
        measurement and the local leader velocity 'vL'.

        :param v: agent velocity
        :type v: ca.MX, np.ndarray
        :param rpos: relative position vector, 3x1
        :type rpos: ca.MX , np.ndarray
        :param vL: local leader velocity
        :type vL: ca.MX , np.ndarray
        """
        v_err = vL - v
        next_rpos = rpos + ca.mtimes(r_mat(q).T, v_err)

        return next_rpos

    def astrobee_dynamics(self, x, u):
        """
        Pendulum nonlinear dynamics.

        :param x: state
        :type x: ca.MX
        :param u: control input
        :type u: ca.MX
        :return: state time derivative
        :rtype: ca.MX
        """

        # State extraction
        p = x[0:3]
        v = x[3:6]
        q = x[6:10]  # /ca.norm_2(x[6:10])
        w = x[10:]

        # 3D Force
        f = u[0:3]

        # 3D Torque
        tau = u[3:]

        # Model
        pdot = v
        vdot = ca.mtimes(r_mat(q), f)/self.mass
        qdot = ca.mtimes(self.xi_mat(q), w)/2
        wdot = ca.mtimes(ca.inv(self.inertia), tau + ca.mtimes(skew(w),
                         ca.mtimes(self.inertia, w)))

        dxdt = [pdot, vdot, qdot, wdot]

        return ca.vertcat(*dxdt)

    def test_dynamics(self, state, input):
        """
        Helper function for a simple dynamics test.
        """
        if self.solver != 'acados':
            next_state = self.model(state, input)  # state after self.dt
            next_state[6:10] = next_state[6:10]/ca.norm_2(next_state[6:10])
            return np.asarray(next_state)

    def rk4_integrator(self, dynamics):
        """
        Runge-Kutta 4th Order discretization.

        :param x: state
        :type x: ca.MX
        :param u: control input
        :type u: ca.MX
        :return: state at next step
        :rtype: ca.MX
        """
        x0 = ca.MX.sym('x0', self.n, 1)
        u = ca.MX.sym('u', self.m, 1)

        x = x0

        k1 = dynamics(x, u)
        k2 = dynamics(x + self.dt / 2 * k1, u)
        k3 = dynamics(x + self.dt / 2 * k2, u)
        k4 = dynamics(x + self.dt * k3, u)
        xdot = x0 + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        # Normalize quaternion: TODO(Pedro-Roque): check best way to propagate
        rk4 = ca.Function('RK4', [x0, u], [xdot], self.fun_options)

        return rk4

    def xi_mat(self, q):
        """
        Generate the matrix for quaternion dynamics Xi,
        from Trawney's Quaternion tutorial.

        :param q: unit quaternion
        :type q: ca.MX
        :return: Xi matrix
        :rtype: ca.MX
        """
        Xi = ca.MX(4, 3)

        # Extract states
        qx = q[0]
        qy = q[1]
        qz = q[2]
        qw = q[3]

        # Generate Xi matrix
        Xi[0, 0] = qw
        Xi[0, 1] = -qz
        Xi[0, 2] = qy

        Xi[1, 0] = qz
        Xi[1, 1] = qw
        Xi[1, 2] = -qx

        Xi[2, 0] = -qy
        Xi[2, 1] = qx
        Xi[2, 2] = qw

        Xi[3, 0] = -qx
        Xi[3, 1] = -qy
        Xi[3, 2] = -qz

        return Xi

    def omega_mat(self, w):
        """
        Get quaternion Omega matrix. Ref:
        "Indirect Kalman Filter for 3D Attitude Estimation"
        Nikolas Trawny and Stergios I. Roumeliotis

        :param w: angular velocity vector
        :type w: ca.MX or ca.DM
        :return: omega matrix 4x4
        :rtype: ca.MX
        """
        omega = ca.MX.zeros(4, 4)

        w_x = w[0]
        w_y = w[1]
        w_z = w[2]

        omega[3, 1] = w_z
        omega[3, 2] = -w_y
        omega[3, 3] = w_x

        omega[0, 0] = -w_z
        omega[0, 2] = w_x
        omega[0, 3] = w_y

        omega[1, 0] = w_y
        omega[1, 1] = -w_x
        omega[1, 3] = w_z

        omega[2, 0] = -w_x
        omega[2, 1] = -w_y
        omega[2, 2] = -w_z

        return omega
