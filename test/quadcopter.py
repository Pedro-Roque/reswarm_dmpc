from __future__ import division
import sys
from acados_template import AcadosModel
import glog as log
import casadi as ca
import sys
import numpy as np
import math as ma

from utils import *
import itertools
import os 


class Quadcopter(object):
    def __init__(self, dynamic_type = "Euler", h=0.05, g = 9.81, Ix = 0.03, Iy = 0.03, Iz = 0.06, m = 1.5):
        
        if dynamic_type is None :
            log.warning("System type set to Euler description")
            dynamic_type = "Euler"
        else:
            log.check(dynamic_type == "Euler" or dynamic_type == "Quaternion", "The dynamic type has to be either Euler or Quaternion")
        
        self.g = g
        self.dt = h
        self.dynamic_type = dynamic_type

        # Quadcopter Parameters
        self.m = m
        self.Ix = Ix
        self.Iy = Iy
        self.Iz = Iz

        self.model_nl = self.quad_nl_dynamics if self.dynamic_type == "Quaternion" else self.quad_nl_euler_dynamics
        
        self.Integrator_nl = None
        self.rk4_intergrator = None

        self.Nx = 13 if self.dynamic_type == "Quaternion" else 12
        self.Nu = 4

        self.W = np.zeros((1,12))

        self.set_integrators()
        
        print("Quadcopter class initialized.\n")
        print("Dynamic: {}.\nDiscretization step {}.\nInertia Ixx {}, Iyy {}, Izz {}.\nMasse {} and gravity {}.\n".format(dynamic_type,h,Ix,Iy,Iz,m,g))

    def set_integrators(self):
        """
        Generate continuous time high-precision integrators.
        """
        
        # Set CasADi variables
        x = ca.MX.sym('x', self.Nx)
        u = ca.MX.sym('u', self.Nu)

        # Integration method - integrator options an be adjusted
        options = {"abstol" : 1e-5, "reltol" : 1e-9, "max_num_steps": 100, 
                   "tf" : self.dt}
   
        # Create nonlinear dynamics integrator
        dae = {'x': x, 'ode': self.model_nl(x,u), 'p':ca.vertcat(u)}
        self.Integrator_nl = ca.integrator('integrator', 'cvodes', dae, options)
        
        # Create Runge-Kutta integrator
        self.rk4_integrator =  self._rk4_integrator(self.Nx, self.Nu)

    def quad_nl_euler_dynamics(self, x, u, *_):
        """
        Pendulum nonlinear dynamics based on euler angle description.

        :param x: state
        :type x: casadi.DM or casadi.MX
        :param u: control input
        :type u: casadi.DM or casadi.MX
        :return: state time derivative
        :rtype: casadi.DM or casadi.MX, depending on inputs
        """
            
        x1 = x[X] # x 
        x2 = x[Y] # y 
        x3 = x[Z] # z

        x4 = x[VX] # Vx
        x5 = x[VY] # Vy
        x6 = x[VZ] # Vz

        x7 = x[WX] # Wroll
        x8 = x[WY] # Wpitch
        x9 = x[WZ] # Wyaw
            
        x10 = x[Roll] # Roll
        x11 = x[Pitch] # Pitch
        x12 = x[Yaw] # Yaw

        u1 = u[0]
        u2 = u[1]
        u3 = u[2]
        u4 = u[3]

        Ix = self.Ix
        Iy = self.Iy
        Iz = self.Iz

        xdot1  = x4
        xdot2  = x5
        xdot3  = x6

        xdot4  = (ca.cos(x10)*ca.sin(x11)*ca.cos(x12) + ca.sin(x10)*ca.sin(x12))*u1/self.m
        xdot5  = (ca.cos(x10)*ca.sin(x11)*ca.sin(x12) - ca.sin(x10)*ca.cos(x12))*u1/self.m
        xdot6  = (ca.cos(x10)*ca.cos(x11))*u1/self.m - self.g

        xdot7 = ((Iy-Iz)*x8*x9 +u2)/Ix
        xdot8 = ((Iz-Ix)*x9*x7 +u3)/Iy
        xdot9 = ((Ix-Iy)*x7*x8 +u4)/Iz

        xdot10  = x7 + x8 * ca.sin(x10) * ca.tan(x11) + x9 * ca.cos(x10) * ca.tan(x11)
        xdot11  = x8 * ca.cos(x10)  - x9 * ca.sin(x10)
        xdot12  = x8 * ca.sin(x10)/ca.cos(x11) + x9 * ca.cos(x10)/ca.cos(x11)

        dxdt = [xdot1,
                xdot2,
                xdot3,
                xdot4,
                xdot5,
                xdot6,
                xdot7,
                xdot8,
                xdot9,
                xdot10,
                xdot11,
                xdot12,
                ]

        return ca.vertcat(*dxdt)

    def quad_nl_dynamics(self, x, u):
        """
        Pendulum nonlinear dynamics based on quaternions description.

        :param x: state
        :type x: casadi.DM or casadi.MX
        :param u: control input
        :type u: casadi.DM or casadi.MX
        :return: state time derivative
        :rtype: casadi.DM or casadi.MX, depending on inputs
        """

        x0 = x[X]
        x1 = x[Y]
        x2 = x[Z]

        vx = x[VX]
        vy = x[VY]
        vz = x[VZ]

        w1 = x[WX]
        w2 = x[WY]
        w3 = x[WZ]

        qx = x[QX]
        qy = x[QY]
        qz = x[QZ]
        qw = x[QW]
        
        t = u[0]
        m1 = u[1]
        m2 = u[2]
        m3 = u[3]
        
        xdot = vx
        ydot = vy
        zdot = vz

        vxdot =  t/self.m * 2*(qx*qz + qw*qy)
        vydot =  t/self.m * 2*(qy*qz - qw*qx)
        vzdot =  -self.g + t/self.m * ((qw**2 - qx**2 - qy**2 + qz**2))

        w1dot = ((self.Iy-self.Iz)*w2*w3 +m1)/self.Ix
        w2dot = ((self.Iz-self.Ix)*w3*w1 +m2)/self.Iy
        w3dot = ((self.Ix-self.Iy)*w1*w2 +m3)/self.Iz

        qxdot = 1/2 * ( qw*w1 - qz*w2 + qy*w3)
        qydot = 1/2 * ( qz*w1 + qw*w2 - qx*w3)
        qzdot = 1/2 * ( -qy*w1 + qx*w2 + qw*w3)
        qwdot = 1/2 * ( -qx*w1 - qy*w2 - qz*w3)

        dxdt = [ xdot,
                 ydot,
                 zdot,
                 vxdot,
                 vydot,
                 vzdot,
                 w1dot,
                 w2dot,
                 w3dot,
                 qxdot,
                 qydot,
                 qzdot,
                 qwdot
                 ]

        return ca.vertcat(*dxdt)

    def discrete_nl_dynamics_rk4(self, x0, u, noise = False):
        """
        Perform a time step in continuous quaternion dynamic based on RGK4 integrator.

        :param x0: state
        :type x0: ca.MX
        :param u: control input
        :type u: ca.Mx
        :param noise: add noise on computed dot(x0)
        :type noise: Boolean
        :return: dot(x0), time derivative
        :rtype: ca.DM
        """
        log.check_eq(x0.shape[0], self.Nx, "Wrong state dimension in RGK4 integrator.")
        log.check_eq(u.shape[0], self.Nu, "Wrong input dimension in RGK4 integrator.")

        out = self.rk4_integrator(x=x0, u=ca.vertcat(u))
        
        if not noise:
            return out["xDot"]
        else:
            return self._add_noise(out["xDot"])

    def discrete_nl_dynamics(self, x0, u, noise = False):
        """
        Perform a time step in continuous dynamic based on casadi integrator.

        :param x0: state
        :type x0: ca.MX
        :param u: control input
        :type u: ca.Mx
        :param noise: add noise on computed dot(x0)
        :type noise: Boolean
        :return: dot(x0), time derivative
        :rtype: ca.DM
        """
        log.check_eq(x0.shape[0], self.Nx, "Wrong state dimension in RGK4 quaternion integrator.")
        log.check_eq(u.shape[0], self.Nu, "Wrong input dimension in RGK4 euler integrator.")

        out = self.Integrator_nl(x=x0, u=ca.vertcat(u))

        if not noise:
            return out["xDot"]
        else:
            return self._add_noise(out["xDot"])

    def trajectory_to_reference(self, x_ref, xd_ref, sys_type):
        """
        Generate concateneted reference from flat representation generated trajectory.

        :param x_ref: Flat representation reference [x, y, z, yaw]
        :type x_ref: ca.MX
        :param xd_ref: Flat representation derivative reference [x', y', z', yaw']
        :type xd_ref: ca.MX
        :param controller: Controller
        :type controller: Python Class        
        :return: Reference state R^(Horizon+1)*Nx
        :rtype: np.array
        """
        
        if self.dynamic_type == "Quaternion":
            x_out = [x_ref[idx][0:3] + xd_ref[idx][0:3] + [0, 0, xd_ref[idx][3]] + euler_to_quaternion(x_ref[idx][3], x_ref[idx][4] ,x_ref[idx][5]) for idx in range(len(x_ref))]
        elif self.dynamic_type == "Euler":
            x_out = [x_ref[idx][0:3] + xd_ref[idx][0:3] + [0, 0, xd_ref[idx][3]] + [x_ref[idx][3], x_ref[idx][4] ,x_ref[idx][5]] for idx in range(len(x_ref))]
            # x_out = [x_ref[idx][0:3] + [0,0,0] + [0, 0, xd_ref[idx][3]] + [x_ref[idx][3], x_ref[idx][4] ,x_ref[idx][5]] for idx in range(len(x_ref))]

        return np.array(list(itertools.chain.from_iterable(x_out)))

    def set_noise(self, w):
        """
        Set noise param

        :param w: noise vector
        :type w: list, np.array
        """
        self.W = np.array(w)

    def get_noise(self):
        """
        Get noise param

        :return: noise vector
        :rtype: list or np.array
        """
        return self.W
    
    def get_dynamic_fct(self):
        return self.rk4_integrator
    
    def get_dimensions(self):
        """
        Get state dimension and input dimension

        :return: Dimensions Nx, Nu
        :rtype: int, int
        """
        return self.Nx, self.Nu

    def get_dynamic(self):
        """
        Get dynamic description type, Euler or Quaternion

        :return: dynamic description type, Euler or Quaternion
        :rtype: str
        """
        return self.dynamic_type
    
    def _rk4_integrator(self , Nx, Nu):
        """
        Definition of Runge-Kutta 4th Order discretization function.

        :param dynamics: Target dynamic to integrate
        :type dynamics: function/instancemethod
        :param Nx: State dimension
        :type Nx: int
        :param Nu: Input dimension
        :type Nu: int        
        :return: RGK4 integrator
        :rtype: ca.Function
        """
        x = ca.MX.sym('x', Nx, 1)
        u = ca.MX.sym('u', Nu, 1)

        dynamics = self.model_nl

        k1 = dynamics(x, u)
        k2 = dynamics(x + self.dt / 2 * k1, u)
        k3 = dynamics(x + self.dt / 2 * k2, u)
        k4 = dynamics(x + self.dt * k3, u)
        xDot = x + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        rk4 = ca.Function('RK4', [x, u], [xDot],['x','u'],['xDot'])
        
        return rk4

    def _add_noise(self, x):
        """
        Add Gaussian noise input state.

        :param x: state vector
        :type x: ca.DM
        :return: Noisy state vector
        :rtype: ca.DM
        """
        log.check_eq(x.shape[0], self.W.shape[0], "State dimension different than provided noise vector dimension.")

        for idx in range(self.W.shape[0]):
            w = np.random.normal(0, self.W[idx], size=1)
            x[idx] = x[idx] + w

        return x

    def export_acados_model(self):
        """
        Export acados model based non-linear dynamic.

        :return: Acados model
        :rtype: acados model
        """
        model_name = 'quadcopter_ode'

        x = ca.MX.sym('x', self.Nx)
        u = ca.MX.sym('u', self.Nu)
        p = []
        xDot = ca.MX.sym('xDot', self.Nx)

        f_expl = self.model_nl(x, u)
        f_impl = xDot - f_expl

        model = AcadosModel()

        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = x
        model.xdot = xDot
        model.u = u
        model.p = p
        model.name = model_name

        return model

    def export_acados_model_with_discrete_rk4(self):
        """
        Export acados model based RK4 approximation of non-linear dynamic.

        :return: Acados model
        :rtype: acados model
        """
        model = self.export_acados_model()

        model.name = "quadcopter_rk4"
        x = model.x
        u = model.u
        nx = x.size()[0]
        dT = self.dt

        ode = ca.Function('ode', [x, u], [model.f_expl_expr])
        # set up RK4
        k1 = ode(x,       u)
        k2 = ode(x+dT/2*k1,u)
        k3 = ode(x+dT/2*k2,u)
        k4 = ode(x+dT*k3,  u)
        xf = x + dT/6 * (k1 + 2*k2 + 2*k3 + k4)

        model.disc_dyn_expr = xf
        print("built RK4 for pendulum model with dT = ", dT)

        return model

        
