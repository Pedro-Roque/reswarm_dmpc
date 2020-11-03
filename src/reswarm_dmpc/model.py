from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import casadi as ca
import numpy as np
from filterpy.kalman import KalmanFilter


class Astrobee(object):
    def __init__(self, h=0.1):
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
        self.model = self.rk4_astrobee_dynamics

        self.set_integrators()
        self.set_discrete_time_system()
        self.set_augmented_discrete_system()

        print("Astrobee class initialized")
        print(self)                         # You can comment this line

    def astrobee_dynamics(self, x, u):
        """
        Pendulum nonlinear dynamics.

        :param x: state
        :type x: casadi.DM or casadi.MX
        :param u: control input
        :type u: casadi.DM or casadi.MX
        :return: state time derivative
        :rtype: casadi.DM or casadi.MX, depending on inputs
        """

        dxdt = [pdot, vdot, qdot, wdot]

        return ca.vertcat(*dxdt)

    def rk4_astrobee_dynamics(self, x, u):
        """
        Runge-Kutta 4th Order discretization.

        :param x: state
        :type x: ca.MX or ca.DM
        :param u: control input
        :type u: ca.MX or ca.DM
        :return: state at next step
        :rtype: ca.MX or ca.DM
        """
        k1 = self.nonlinear_model(x, u)
        k2 = self.nonlinear_model(x + self.dt / 2 * k1, u)
        k3 = self.nonlinear_model(x + self.dt / 2 * k2, u)
        k4 = self.nonlinear_model(x + self.dt * k3, u)
        xdot = x + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return xdot
