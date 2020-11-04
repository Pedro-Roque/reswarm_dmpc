from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import casadi as ca
import numpy as np
from filterpy.kalman import KalmanFilter


class Astrobee(object):
    def __init__(self, mass, inertia, h=0.01):
        """
        Pendulum model class.

        Describes the movement of a pendulum with mass 'm' attached to a cart
        with mass 'M'. All methods should return casadi.MX or casadi.DM
        variable types.

        :param h: sampling time, defaults to 0.1
        :type h: float, optional
        """

        # Model
        self.nonlinear_model = self.astrobee_dynamics
        self.model = None
        self.n = 13
        self.m = 6
        self.dt = h

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

        self.model = self.rk4_integrator(self.astrobee_dynamics)
        return

    def test_dynamics(self):
        """
        Helper function for a simple dynamics test.
        """
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
        vdot = ca.mtimes(self.r_mat(q), f)/self.mass
        qdot = ca.mtimes(self.xi_mat(q), w)/2
        wdot = ca.mtimes(ca.inv(self.inertia), tau + ca.mtimes(self.skew(w),
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
        print(x.shape)
        k1 = dynamics(x, u)
        print("Step 1 done")
        k2 = dynamics(x + self.dt / 2 * k1, u)
        k3 = dynamics(x + self.dt / 2 * k2, u)
        k4 = dynamics(x + self.dt * k3, u)
        xdot = x0 + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        # Normalize quaternion
        xdot[6:10] = xdot[6:10]/ca.norm_2(xdot[6:10])
        rk4 = ca.Function('RK4', [x0, u], [xdot])

        return rk4

    def r_mat(self, q):
        """
        Generate rotation matrix from unit quaternion

        :param q: unit quaternion
        :type q: ca.MX
        :return: rotation matrix, SO(3)
        :rtype: ca.MX
        """

        Rmat = ca.MX(3, 3)

        # Extract states
        qx = q[0]
        qy = q[1]
        qz = q[2]
        qw = q[3]

        Rmat[0, 0] = 1 - 2*qy**2 - 2*qz**2
        Rmat[0, 1] = 2*qx*qy - 2*qz*qw
        Rmat[0, 2] = 2*qx*qz + 2*qy*qw

        Rmat[1, 0] = 2*qx*qy + 2*qz*qw
        Rmat[1, 1] = 1 - 2*qx**2 - 2*qz**2
        Rmat[1, 2] = 2*qy*qz - 2*qx*qw

        Rmat[2, 0] = 2*qx*qz - 2*qy*qw
        Rmat[2, 1] = 2*qy*qz + 2*qx*qw
        Rmat[2, 2] = 1 - 2*qx**2 - 2*qy**2

        return Rmat

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

    def skew(self, v):
        """
        Returns the skew matrix of a vector v

        :param v: vector
        :type v: ca.MX
        :return: skew matrix of v
        :rtype: ca.MX
        """

        sk = ca.MX.zeros(3, 3)

        # Extract vector components
        x = v[0]
        y = v[1]
        z = v[2]

        sk[0, 1] = -z
        sk[1, 0] = z
        sk[0, 2] = y
        sk[2, 0] = -y
        sk[1, 2] = -x
        sk[2, 1] = x

        return sk
