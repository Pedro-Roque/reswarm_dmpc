from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import casadi as ca
import numpy as np
import numpy.matlib as nmp
from reswarm_dmpc.util import *


class Astrobee(object):
    def __init__(self,
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
        self.solver = 'acados'
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

        self.set_dynamics()
        self.test_dynamics()

        print("Astrobee class initialized")

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
        q = x[6:10]
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

        # Normalize quaternion: TODO(Pedro-Roque): check how to normalize
        # xdot[6:10] = xdot[6:10]/ca.norm_2(xdot[6:10])
        fun_options = {
            "jit": False,
            "jit_options": {"flags": ["-O2"]}
        }
        rk4 = ca.Function('RK4', [x0, u], [xdot], fun_options)

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
            vy = A*np.sin(2*np.pi*f*t)
            vx = 0.05*np.ones(npoints)

            # Once we have a velocity profile, we can create the
            # position references
            x = np.array([x0[0]])
            y = np.array([x0[1]])
            for i in range(npoints-1):
                x = np.append(x, x[-1]+vx[i]*self.dt)
                y = np.append(y, y[-1]+vx[i]*self.dt)

            # Create trajectory matrix
            x_sp = np.array([x])
            x_sp = np.append(x_sp, [y], axis=0)
            x_sp = np.append(x_sp, np.zeros((1, npoints)), axis=0)
            x_sp = np.append(x_sp, [vx], axis=0)
            x_sp = np.append(x_sp, [vy], axis=0)
            x_sp = np.append(x_sp, np.zeros((4, npoints)), axis=0)
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
