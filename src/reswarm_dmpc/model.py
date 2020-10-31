from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import casadi as ca
import numpy as np
from filterpy.kalman import KalmanFilter

class SimplePendulum(object):
    def __init__(self, h=0.1):
        """
        Pendulum model class. 
        
        Describes the movement of a pendulum with mass 'm' attached to a cart
        with mass 'M'. All methods should return casadi.MX or casadi.DM variable 
        types.

        :param h: sampling time, defaults to 0.1
        :type h: float, optional
        """

        # Model, gravity and sampling time parameters
        self.model = self.pendulum_linear_dynamics
        self.g = 9.81
        self.dt = h

        # System reference (x_d) and disturbance (w)
        self.x_d = 10
        self.w = 0.0

        # Pendulum Parameters
        self.m = 0.2
        self.I = 0.006
        self.l = 0.3
        self.bp = 0.012

        # Aggregated terms
        self.a0 = -(self.m*self.g*self.l)/(self.I+self.m*self.l**2)
        self.a1 = self.bp/(self.I+self.m*self.l**2)
        self.b0 = (self.m*self.l)/(self.I+self.m*self.l**2)
        
        # Linearize system around vertical equilibrium with no input
        self.x_eq = [0,0]
        self.u_eq = 0
        self.Integrator = None

        self.set_integrators()
        self.set_discrete_time_system()

        print("Pendulum class initialized")
        print(self)                         # You can comment this line

    def __str__(self):
        return """                                                                  
                ,@@,                                                            
              @@@@@@@@                                                          
              @@@ m @@                                                         
               .@@@@&                                                          
                     *                   .                                      
                      (#       theta     .                                   
                       ,* *              .                                   
                         */              .                                      
                          (*             .                                      
                            *,           .                                      
                             #(          .                                      
           Y               l   *         .                                      
           ^                    ##       .                                      
           |                     .*      .                                      
           |                       /(    .                                      
           |                        /*   .                                      
           +-------> X                *, .                                      
                                       %/.                                                     
            -----------------------------------------------------------      """

    def set_integrators(self):
        """
        Generate continuous time high-precision integrators.
        """
        
        # Set CasADi variables
        x = ca.MX.sym('x', 2)
        u = ca.MX.sym('u', 1)

        # Integration method - integrator options an be adjusted
        options = {"abstol" : 1e-5, "reltol" : 1e-9, "max_num_steps": 100, 
                   "tf" : self.dt}

        # Create linear dynamics integrator
        dae = {'x': x, 'ode': self.model(x,u), 'p':ca.vertcat(u)}
        self.Integrator = ca.integrator('integrator', 'cvodes', dae, options)

        # Create nonlinear dynamics integrator
        dae = {'x': x, 'ode': self.set_pendulum_nl_dynamics(x,u), 'p':ca.vertcat(u)}
        self.Integrator_nl = ca.integrator('integrator', 'cvodes', dae, options)

    def set_discrete_time_system(self):
        """
        Set discrete-time system matrices from linear continuous dynamics.
        """
        
        # Check for integrator definition
        if self.Integrator is None:
            print("Integrator not defined. Set integrators first.")
            exit()

        # Set CasADi variables
        x = ca.MX.sym('x', 2)
        u = ca.MX.sym('u', 1)
    
        # Jacobian of exact discretization
        self.Ad = ca.Function('jac_x_Ad', [x, u], [ca.jacobian(
                            self.Integrator(x0=x, p=u)['xf'], x)])
        self.Bd = ca.Function('jac_u_Bd', [x, u], [ca.jacobian(
                            self.Integrator(x0=x, p=u)['xf'], u)])


    def pendulum_linear_dynamics(self, x, u):  
        """ 
        Pendulum continuous-time linearized dynamics.

        :param x: state
        :type x: MX variable, 4x1
        :param u: control input
        :type u: MX variable, 1x1
        :return: dot(x)
        :rtype: MX variable, 4x1
        """

        Ac = ca.MX.zeros(2,2)
        Bc = ca.MX.zeros(2,1)

        ### Build Ac matrix
        # First Row
        Ac[0,1] = 1

        # Second row
        Ac[1,0] = -self.a0
        Ac[1,1] = -self.a1

        ### Build Bc matrix
        # Second Row
        Bc[1,0] = self.b0

        ### Store matrices as class variables
        self.Ac = Ac
        self.Bc = Bc 

        return ca.mtimes(Ac, x) + ca.mtimes(Bc, u  )

    def set_pendulum_nl_dynamics(self, x, u):
        """Pendulum nonlinear dynamics

        :param x: state, 2x1
        :type x: ca.MX
        :param u: control input, 1x1
        :type u: ca.MX
        :return: state time derivative, 2x1
        :rtype: ca.MX
        """

        f1 = -self.a0*ca.sin(x[0]) - self.a1*x[1] + self.b0*ca.cos(x[0])*u
        dxdt = [ x[1], f1 ]

        return ca.vertcat(*dxdt)

    def set_reference(self, ref):
        """
        Simple method to set the new system reference.

        :param ref: desired reference [m]
        :type ref: float or casadi.DM 1x1
        """
        self.x_d = ref
        
    def get_discrete_system_matrices_at_eq(self):
        """
        Evaluate the discretized matrices at the equilibrium point

        :return: A,B,C matrices for equilibrium point
        :rtype: casadi.DM 
        """
        A_eq = self.Ad(self.x_eq, self.u_eq)
        B_eq = self.Bd(self.x_eq, self.u_eq)
        
        # Populate a full observation matrix
        C_eq = ca.DM.zeros(1,2)
        C_eq[0,0] = 1
        C_eq[0,1] = 1

        return A_eq, B_eq, C_eq


    def discrete_integration(self, x0, u):
        """
        Perform a time step iteration in continuous dynamics.

        :param x0: initial state
        :type x0: 4x1 ( list [a, b, c, d] , ca.MX )
        :param u: control input
        :type u: scalar, 1x1
        :return: dot(x), time derivative
        :rtype: 4x1, ca.DM
        """
        out = self.Integrator(x0=x0, p=u)
        return out["xf"]

    def discrete_nl_dynamics(self, x0, u):
        """Discrete time nonlinear integrator

        :param x0: starting state
        :type x0: ca.DM
        :param u: control input
        :type u: ca.DM
        :return: state at the final sampling interval time
        :rtype: ca.DM
        """
        out = self.Integrator_nl(x0=x0, p=u)
        return out["xf"]

    def discrete_time_dynamics(self,x0,u):
        """ 
        Performs a discrete time iteration step.

        :param x0: initial state
        :type x0: 4x1 ( list [a, b, c, d] , ca.MX )
        :param u: control input
        :type u: scalar, 1x1
        :return: next discrete time state
        :rtype: 4x1, ca.DM
        """
        
        return ca.mtimes(self.Ad(self.x_eq, self.u_eq),  x0) + \
                ca.mtimes(self.Bd(self.x_eq, self.u_eq), u)

    def enable_disturbance(self, w=0.01):
        """
        Enable system disturbance as a wind force.

        :param w: disturbance magnitude, defaults to 0.1
        :type w: float, optional
        """

        # Activate disturbance
        self.w = w

        # Re-generate integrators for dynamics with disturbance
        self.set_integrators()

    pass


class Pendulum(object):
    def __init__(self, h=0.1):
        """
        Pendulum model class. 
        
        Describes the movement of a pendulum with mass 'm' attached to a cart
        with mass 'M'. All methods should return casadi.MX or casadi.DM variable 
        types.

        :param h: sampling time, defaults to 0.1
        :type h: float, optional
        """

        # Model, gravity and sampling time parameters
        self.model = self.pendulum_linear_dynamics
        self.model_nl = self.pendulum_nonlinear_dynamics
        self.model_ag = self.pendulum_augmented_dynamics
        self.g = 9.81
        self.dt = h

        # System reference (x_d) and disturbance (w)
        self.x_d = 10
        self.w = 0.0

        # Pendulum Parameters
        self.m = 0.2
        self.M = 0.5
        self.I = 0.006
        self.l = 0.3
        self.bc = 0.1
        self.bp = 0.012

        # Linearize system around vertical equilibrium with no input
        self.x_eq = [0,0,0,0]
        self.u_eq = 0
        self.Integrator_lin = None
        self.Integrator = None
        self.Integrator_ag = None
        self.Ad_i = None

        self.set_integrators()
        self.set_discrete_time_system()
        self.set_augmented_discrete_system()

        print("Pendulum class initialized")
        print(self)                         # You can comment this line

    def __str__(self):
        return """                                                                  
                ,@@,                                                            
              @@@@@@@@                                                          
              @@@ m @@                                                         
               .@@@@&                                                          
                     *                   .                                      
                      (#       theta     .                                   
                       ,* *              .                                   
                         */              .                                      
                          (*             .                                      
                            *,           .                                      
                             #(          .                                      
           Y               l   *         .                                      
           ^                    ##       .                                      
           |                     .*      .                                      
           |                       /(    .                                      
           |                        /*   .                                      
           +-------> X                *, .                                      
                                       %/.                                      
                        ***************////***************                      
                        (*********************************      F              
                        (***  M  *************************---------->       
                        (*********************************                      
                            ,/**/                ,/*/*                          
                          ********#            (*******(                        
                          #*******(            %*******#                        
                            %***#                %***%                          
            -----------------------------------------------------------      """

    def set_integrators(self):
        """
        Generate continuous time high-precision integrators.
        """
        
        # Set CasADi variables
        x = ca.MX.sym('x', 4)
        u = ca.MX.sym('u', 1)
        w = ca.MX.sym('w', 1)

        # Integration method - integrator options an be adjusted
        options = {"abstol" : 1e-5, "reltol" : 1e-9, "max_num_steps": 100, 
                   "tf" : self.dt}

        # Create linear dynamics integrator
        dae = {'x': x, 'ode': self.model(x,u,w), 'p':ca.vertcat(u,w)}
        self.Integrator_lin = ca.integrator('integrator', 'cvodes', dae, options)
        
        # Create nonlinear dynamics integrator
        dae = {'x': x, 'ode': self.model_nl(x,u), 'p':ca.vertcat(u)}
        self.Integrator = ca.integrator('integrator', 'cvodes', dae, options)

        if self.Ad_i is not None:
            # Create augmented system dynamics integrator
            x_ag = ca.MX.sym('x', 5)
            dae = {'x': x_ag, 'ode': self.model_ag(x_ag,u), 'p':ca.vertcat(u)}
            self.Integrator_ag = ca.integrator('integrator', 'cvodes', dae, options)

    def set_discrete_time_system(self):
        """
        Set discrete-time system matrices from linear continuous dynamics.
        """
        
        # Check for integrator definition
        if self.Integrator_lin is None:
            print("Integrator_lin not defined. Set integrators first.")
            exit()

        # Set CasADi variables
        x = ca.MX.sym('x', 4)
        u = ca.MX.sym('u', 1)
        w = ca.MX.sym('w', 1)
    
        # Jacobian of exact discretization
        self.Ad = ca.Function('jac_x_Ad', [x, u, w], [ca.jacobian(
                            self.Integrator_lin(x0=x, p=ca.vertcat(u, w))['xf'], x)])
        self.Bd = ca.Function('jac_u_Bd', [x, u, w], [ca.jacobian(
                            self.Integrator_lin(x0=x, p=ca.vertcat(u, w))['xf'], u)])
        self.Bw = ca.Function('jac_u_Bd', [x, u, w], [ca.jacobian(
                            self.Integrator_lin(x0=x, p=ca.vertcat(u, w))['xf'], w)])
        

        # C matrix does not depend on the state
        # TODO: put this in a better place later!
        Cd_eq = ca.DM.zeros(1,4)
        Cd_eq[0,0] = 1
        Cd_eq[0,1] = 1
        Cd_eq[0,2] = 1
        Cd_eq[0,3] = 1

        self.Cd_eq = Cd_eq
    
    def set_augmented_discrete_system(self):
        """
        Pendulum dynamics with integral action.

        :param x: state
        :type x: casadi.DM
        :param u: control input
        :type u: casadi.DM
        """

        # Grab equilibrium dynamics
        Ad_eq = self.Ad(self.x_eq, self.u_eq, self.w)
        Bd_eq = self.Bd(self.x_eq, self.u_eq, self.w)

        # Instantiate augmented system
        self.Ad_i = ca.DM.zeros(5,5)
        self.Bd_i = ca.DM.zeros(5,1)
        self.Bw_i = ca.DM.zeros(5,1)
        self.Cd_i = ca.DM.zeros(1,5)
        self.R_i = ca.DM.zeros(5,1)

        # Populate matrices
        self.Ad_i[0:4,0:4] = Ad_eq
        self.Ad_i[4,0:4] = ca.mtimes(-self.dt, self.Cd_eq)
        self.Ad_i[4,4] = 1

        self.Bd_i[0:4,0] = Bd_eq

        self.Bw_i[0:4,0] = self.Bw(self.x_eq, self.u_eq, self.w)

        self.R_i[4,0] = self.dt

        self.Cd_i[0,0:4] = self.Cd_eq

    def pendulum_linear_dynamics(self, x, u, w):  
        """ 
        Pendulum continuous-time linearized dynamics.

        :param x: state
        :type x: MX variable, 4x1
        :param u: control input
        :type u: MX variable, 1x1
        :return: dot(x)
        :rtype: MX variable, 4x1
        """

        Ac = ca.MX.zeros(4,4)
        Bc = ca.MX.zeros(4,1)
        Bwc = ca.MX.zeros(4,1)

        v1 = (self.m + self.M)/(self.I*(self.m + self.M) + self.m*self.M*self.l**2)
        v2 = (self.I+self.m*self.l**2)/(self.I*(self.m + self.M) + self.m*self.M*self.l**2)

        ### Build Ac matrix
        # First Row
        Ac[0,1] = 1

        # Second row
        Ac[1,1] = -self.bc*v2
        Ac[1,2] = self.m**2 * self.l**2 * self.g * v2 / (self.I + self.m*(self.l**2))
        Ac[1,3] = -self.m * self.l * self.bp * v2 / (self.I + self.m*(self.l**2))

        # Third row
        Ac[2,3] = 1
        
        # Fourth row
        Ac[3,1] = -self.m * self.l * self.bc * v1 /(self.m + self.M)
        Ac[3,2] = self.m * self.g * self.l * v1
        Ac[3,3] = -self.bp * v1

        ### Build Bc matrix
        # Second Row
        Bc[1,0] = v2

        # Fourth row
        Bc[3,0] = self.m * self.l * v1 / (self.m + self.M)

        ### Build Bwc
        Bwc[1,0] = -self.I*v2/(self.I + self.m*self.l**2); 
        Bwc[3,0] = self.M*self.l*v1/(self.M+self.m);


        ### Store matrices as class variables
        self.Ac = Ac
        self.Bc = Bc
        self.Bwc = Bwc 

        return ca.mtimes(Ac, x) + ca.mtimes(Bc, u) + ca.mtimes(Bwc, w)

    def pendulum_linear_dynamics_with_disturbance(self, x, u):
        Ad_eq = self.Ad(self.x_eq, self.u_eq, self.w)
        Bd_eq = self.Bd(self.x_eq, self.u_eq, self.w)
        Bw_eq = self.Bw(self.x_eq, self.u_eq, self.w)

        return ca.mtimes(Ad_eq, x) + ca.mtimes(Bd_eq, u) + ca.mtimes(Bw_eq, self.w) 

    def pendulum_nonlinear_dynamics(self, x, u, *_):
        """
        Pendulum nonlinear dynamics.

        :param x: state
        :type x: casadi.DM or casadi.MX
        :param u: control input
        :type u: casadi.DM or casadi.MX
        :return: state time derivative
        :rtype: casadi.DM or casadi.MX, depending on inputs
        """
        x1 = x[0]
        x2 = x[1]
        x3 = x[2]
        x4 = x[3]
         
        # dot(x2)
        f1 = (1.0/ ( self.M + self.m - (self.m**2*self.l**2*ca.cos(x3)**2)/(self.I + self.m*self.l**2) ) ) * \
             (u + ( (self.m*self.l**2*ca.cos(x3)**2)/(self.I + self.m*self.l**2) - 1)*self.w  
              - self.bc*x2 
              - self.m*self.l*x4**2*ca.sin(x3) 
              + (self.m**2*self.l**2*self.g*ca.sin(x3)*ca.cos(x3))/(self.I + self.m*self.l**2) 
              - (self.m*self.l*self.bp*x4*ca.cos(x3))/(self.I + self.m*self.l**2) 
             )

        # dot(x4)
        f2 = (1.0/ ( self.I + self.m*self.l**2 - (self.m**2*self.l**2*ca.cos(x3)**2)/(self.m + self.M) ) ) * \
             (
                 u*(self.m*self.l*ca.cos(x3))/(self.m+self.M) 
                 + self.w*(self.M*self.l*ca.cos(x3))/(self.m+self.M) 
                 - self.bp*x4 
                 + self.m*self.l*self.g*ca.sin(x3) 
                 - (self.m**2*self.l**2*x4**2*ca.sin(x3)*ca.cos(x3))/(self.m + self.M) 
                 - (self.m*self.l*self.bc*x2*ca.cos(x3))/(self.m+self.M)
             )
        
        dxdt = [ x2, f1, x4, f2 ]

        return ca.vertcat(*dxdt)


    def pendulum_augmented_dynamics(self, x, u):
        """Augmented pendulum system dynamics

        :param x: state
        :type x: casadi.DM
        :param u: control input
        :type u: casadi.DM
        :return: next state
        :rtype: casadi.DM
        """

        return ca.mtimes(self.Ad_i, x) + ca.mtimes(self.Bd_i, u) + self.R_i * self.x_d + self.Bw_i * self.w

    def set_reference(self, ref):
        """
        Simple method to set the new system reference.

        :param ref: desired reference [m]
        :type ref: float or casadi.DM 1x1
        """
        self.x_d = ref

    def set_equilibrium_point(self, x_eq, u_eq):
        """
        Set a different equilibrium poin for the system.

        :param x_eq: state equilibrium
        :type x_eq: list with 4 floats, [a,b,c,d]
        :param u_eq: control input for equilibrium point
        :type u_eq: float
        """

        self.x_eq = x_eq
        self.u_eq = u_eq
        
    def get_discrete_system_matrices_at_eq(self):
        """
        Evaluate the discretized matrices at the equilibrium point

        :return: A,B,C matrices for equilibrium point
        :rtype: casadi.DM 
        """
        Ad_eq = self.Ad(self.x_eq, self.u_eq, self.w)
        Bd_eq = self.Bd(self.x_eq, self.u_eq, self.w)
        Bw_eq = self.Bw(self.x_eq, self.u_eq, self.w)

        return Ad_eq, Bd_eq, Bw_eq, self.Cd_eq

    def get_augmented_discrete_system(self):
        """
        Get discrete time augmented system with integral component.

        :return: System A, B, C matrices
        :rtype: casadi.DM
        """
        return self.Ad_i, self.Bd_i, self.Bw_i, self.Cd_i


    def continuous_time_linear_dynamics(self, x0, u):
        """
        Perform a time step iteration in continuous dynamics.

        :param x0: initial state
        :type x0: 4x1 ( list [a, b, c, d] , ca.MX )
        :param u: control input
        :type u: scalar, 1x1
        :return: dot(x), time derivative
        :rtype: 4x1, ca.DM
        """
        out = self.Integrator_lin(x0=x0, p=ca.vertcat(u, self.w))
        return out["xf"]

    def continuous_time_nonlinear_dynamics(self, x0, u):
        out = self.Integrator(x0=x0, p=ca.vertcat(u))
        return out["xf"]

    def discrete_time_dynamics(self,x0,u):
        """ 
        Performs a discrete time iteration step.

        :param x0: initial state
        :type x0: 4x1 ( list [a, b, c, d] , ca.MX )
        :param u: control input
        :type u: scalar, 1x1
        :return: next discrete time state
        :rtype: 4x1, ca.DM
        """

        return ca.mtimes(self.Ad(self.x_eq, self.u_eq, self.w), x0) + \
                ca.mtimes(self.Bd(self.x_eq, self.u_eq, self.w), u) + \
                ca.mtimes(self.Bw(self.x_eq, self.u_eq, self.w), self.w)

    def enable_disturbance(self, w=0.01):
        """
        Enable system disturbance as a wind force.

        :param w: disturbance magnitude, defaults to 0.1
        :type w: float, optional
        """

        # Activate disturbance
        self.w = w

        # Re-generate dynamics
        self.set_integrators()
        self.set_discrete_time_system()
        self.set_augmented_discrete_system()

    #===============================================#
    #            Kalman Filter modules              # 
    #===============================================#

    def set_kf_params(self, C, Q, R):
        """
        Set the Kalman Filter variables.

        :param C: observation matrix
        :type C: numpy.array
        :param Q: process noise
        :type Q: numpy.array
        :param R: measurement noise
        :type R: numpy.array
        """
        self.C_KF = C 
        self.Q_KF = Q
        self.R_KF = R

    def init_kf(self, x=np.array([[0,0,0,0]])):
        """
        Initialize the Kalman Filter estimator.
        """

        # Initialize filter object
        self.kf_estimator = KalmanFilter(dim_x=4, dim_z=np.size(self.C_KF,0), dim_u=2)

        Ad_eq = self.Ad(self.x_eq, self.u_eq, self.w)
        Bd_eq = self.Bd(self.x_eq, self.u_eq, self.w)
        Bw_eq = self.Bw(self.x_eq, self.u_eq, self.w)

        B = ca.DM.zeros(4,2)
        B[:,0] = Bd_eq
        B[:,1] = Bw_eq

        # Set filter parameters
        self.kf_estimator.F = np.asarray(Ad_eq)
        self.kf_estimator.B = np.asarray(B)

        self.kf_estimator.H = self.C_KF
        self.kf_estimator.Q = self.Q_KF
        self.kf_estimator.R = self.R_KF


        # Set initial estimation
        self.kf_estimator.x = x.T 