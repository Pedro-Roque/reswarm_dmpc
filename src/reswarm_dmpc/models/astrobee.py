from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import casadi as ca
import numpy as np
import numpy.matlib as nmp
from reswarm_dmpc.util import *


class Astrobee(object):
    def __init__(self,
                 iface='acados',
                 mass=7,
                 inertia=np.diag([0.1083, 0.1083, 0.1083]),
                 h=0.01):
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

        # Tracking
        self.trajectory_type = "Sinusoidal"

        # Model prperties
        self.mass = mass
        self.inertia = inertia

        # Barrier properties
        self.eps_p = 0.1
        self.eps_v = 0.05
        self.hp_m = 1.0
        self.hp_exp = 1.0

        self.eps_q = 0.5
        self.eps_w = 0.1
        self.hq_m = 1.0
        self.hq_exp = 1.0

        self.set_casadi_options()
        self.set_dynamics()
        self.set_barrier_functions()
        self.test_dynamics()
        self.test_barrier_functions()

        print("Astrobee class initialized")

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

    def test_dynamics(self):
        """
        Helper function for a simple dynamics test.
        """
        if self.solver != 'acados':
            x0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0])
            u0 = np.array([0.1, 0.1, 0.1, 0, 0, 0.1])
            xt_n = self.model(x0, u0)  # state after self.dt seconds
            print(xt_n)

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

    def get_trajectory(self, x0, t0, npoints):
        """
        Generate trajectory to be followed.

        :param x0: starting position
        :type x0: ca.DM
        :param t0: starting time
        :type t0: float
        :param npoints: number of trajectory points
        :type npoints: int
        :return: trajectory with shape (Nx, npoints)
        :rtype: np.array
        """

        if self.trajectory_type == "Sinusoidal":
            # Trajectory params
            f = 0.1
            A = 0.1

            # Trajectory reference: oscillate in Y with vy
            t = np.linspace(t0, t0+(npoints-1)*self.dt, npoints)
            vx = A*np.cos(2*np.pi*f*t)
            vy = A*np.sin(2*np.pi*f*t)
            vz = 0.05*np.ones(npoints)

            # Once we have a velocity profile, we can create the
            # position references
            x = np.array([x0[0]])
            y = np.array([x0[1]])
            z = np.array([x0[2]])

            for i in range(npoints-1):
                x = np.append(x, x[-1]+vx[i]*self.dt)
                y = np.append(y, y[-1]+vy[i]*self.dt)
                z = np.append(z, z[-1]+vz[i]*self.dt)
            # Create trajectory matrix
            x_sp = np.array([x])
            x_sp = np.append(x_sp, [y], axis=0)
            x_sp = np.append(x_sp, [z], axis=0)
            x_sp = np.append(x_sp, [vx], axis=0)
            x_sp = np.append(x_sp, [vy], axis=0)
            x_sp = np.append(x_sp, [vz], axis=0)
            x_sp = np.append(x_sp, np.zeros((3, npoints)), axis=0)
            x_sp = np.append(x_sp, np.ones((1, npoints)), axis=0)
            x_sp = np.append(x_sp, np.zeros((3, npoints)), axis=0)

        return x_sp

    def set_trajectory_type(self, trj_type):
        """
        Set trajectory type to be followed

        :param trj_type: trajectory format
        :type trj_type: string
        """

        self.trajectory_type = trj_type

    def set_barrier_functions(self, hp=None, hpdt=None,
                              hq=None, hqdt=None):
        """
        Helper method to set the desired barrier functions.

        :param hp: position barrier, defaults to None
        :type hp: ca.MX, optional
        :param hpdt: time-derivative of hp, defaults to None
        :type hpdt: ca.MX, optional
        :param hq: attitude barrier, defaults to None
        :type hq: ca.MX, optional
        :param hqdt: time-derivative of hq, defaults to None
        :type hqdt: ca.MX, optional
        """

        if hp is not None and hpdt is not None:
            self.hp = hp
            self.hpdt = hpdt
        else:
            # Paper Translation barrier
            u = ca.MX.sym("u", 6, 1)

            p = ca.MX.sym("p", 3, 1)
            pr = ca.MX.sym("pr", 3, 1)

            v = ca.MX.sym("v", 3, 1)
            vr = ca.MX.sym("vr", 3, 1)

            dot_vr = ca.MX.sym("dotvr", 3, 1)

            hp = self.eps_p - ca.norm_2(p-pr)**2 \
                + self.eps_v - ca.norm_2(v-vr)**2

            hpdt = -2*ca.mtimes((p-pr).T, v) + 2*ca.mtimes((p-pr).T, vr) \
                - 2*ca.mtimes((v-vr).T, u[0:3]/self.mass) \
                + 2*ca.mtimes((v-vr).T, dot_vr)

            self.hp = ca.Function('hp', [p, pr, v, vr],
                                        [self.hp_m*hp**self.hp_exp],
                                  self.fun_options)
            self.hpdt = ca.Function('hpdt', [p, pr, u, v, vr, dot_vr], [hpdt],
                                    self.fun_options)

        if hq is not None and hqdt is not None:
            self.hq = hq
            self.hqdt = hqdt
        else:
            # Paper Attitude Barrier
            q = ca.MX.sym("q", 4, 1)
            qr = ca.MX.sym("qr", 4, 1)

            w = ca.MX.sym("w", 3, 1)
            wr = ca.MX.sym("wr", 3, 1)

            dot_wr = ca.MX.sym("dotwr", 3, 1)

            hq = self.eps_q - (1 - ca.mtimes(q.T, qr)**2) \
                + self.eps_w - ca.norm_2(w-wr)**2

            hqdt = ca.mtimes(ca.mtimes(qr.T, self.xi_mat(q)), w) \
                + ca.mtimes(ca.mtimes(q.T, self.xi_mat(qr)), wr) \
                - 2*ca.mtimes((w - wr).T,
                              ca.mtimes(ca.inv(self.inertia), u[3:])) \
                + 2*ca.mtimes((w-wr).T, dot_wr)

            self.hq = ca.Function('hp', [q, qr, w, wr], [self.hq_m*hq**self.hq_exp],
                                      self.fun_options)
            self.hqdt = ca.Function('hpdt', [q, qr, u, w, wr, dot_wr], [hqdt],
                                    self.fun_options)

        vT = 0.0  # TODO(Pedro-Roque): set v(T) alpha function
        self.vT = 0.0

    def get_barrier_value(self, x_t, x_r, u_t):

        p = x_t[0:3]
        v = x_t[3:6]
        q = x_t[6:10]
        w = x_t[10:]

        pr = x_r[0:3]
        vr = x_r[3:6]
        # dot_vr = ca.MX.zeros(3, 1)
        dot_vr = np.zeros((3, 1))
        qr = x_r[6:10]
        wr = x_r[10:]
        # dot_wr = ca.MX.zeros(3, 1)
        dot_wr = np.zeros((3, 1))

        u = u_t

        # hp_ineq >= 0
        hp_ineq = self.hpdt(p, pr, u, v, vr, dot_vr) \
            + self.hp(p, pr, v, vr)

        # hq_ineq >= 0
        hq_ineq = self.hqdt(q, qr, u, w, wr, dot_wr) \
            + self.hq(q, qr, w, wr)

        return hp_ineq, hq_ineq

    def test_barrier_functions(self):
        """
        Unit test for barrier certificates.
        """

        # Set variables:
        p = np.zeros((3, 1))
        pr = np.zeros((3, 1))
        v = np.zeros((3, 1))
        vr = np.zeros((3, 1))
        dot_vr = np.zeros((3, 1))

        q = np.array([[0, 0, 0, 1]]).reshape(4, 1)
        qr = np.array([[0, 0, 0, 1]]).reshape(4, 1)
        w = np.zeros((3, 1))
        wr = np.zeros((3, 1))
        dot_wr = np.zeros((3, 1))

        u = np.zeros((6, 1))

        # Position barrier:
        hp_output = self.hpdt(p, pr, u, v, vr, dot_vr) + self.hp(p, pr, v, vr)
        hq_output = self.hqdt(q, qr, u, w, wr, dot_wr) + self.hq(q, qr, w, wr)
        print("# --- Zeroing Control Barrier Functions Test --- #")
        print("Test at desired reference:")
        print("Pos Barrier: ", hp_output, "\nAtt Barrier: ", hq_output)
        p = np.ones((3, 1))*2
        hp_output = self.hpdt(p, pr, u, v, vr, dot_vr) + self.hp(p, pr, v, vr)
        hq_output = self.hqdt(q, qr, u, w, wr, dot_wr) + self.hq(q, qr, w, wr)
        print("Test position far from desired reference:")
        print("Pos Barrier: ", hp_output, "\nAtt Barrier: ", hq_output)
        p = np.zeros((3, 1))
        q = np.array([[0, 1, 0, 0]]).reshape(4, 1)
        hp_output = self.hpdt(p, pr, u, v, vr, dot_vr) + self.hp(p, pr, v, vr)
        hq_output = self.hqdt(q, qr, u, w, wr, dot_wr) + self.hq(q, qr, w, wr)
        print("Test attitude far from desired reference:")
        print("Pos Barrier: ", hp_output, "\nAtt Barrier: ", hq_output)
