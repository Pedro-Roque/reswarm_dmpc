"""
Model Predictive Control - CasADi interface
Adapted from Helge-Andre Langaker work on GP-MPC
Customized by Pedro Roque for EL2700 Model Predictive Countrol Course at KTH
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import time
import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from acados_template import AcadosModel
from scipy.stats import norm
import scipy.linalg

from reswarm_dmpc.util import *


class MPC(object):

    def __init__(self, model, dynamics,
                 Q, R, P,
                 ulb, uub, xlb, xub,
                 horizon=10,
                 terminal_constraint=None):
        """
        MPC Controller Class for setpoint stabilization

        :param model: model class
        :type model: python class
        :param dynamics: system dynamics function
        :type dynamics: ca.Function
        :param horizon: prediction horizon [s], defaults to 10
        :type horizon: float, optional
        :param Q: state error weight matrix
        :type Q: np.diag
        :param P: terminal state weight matrix
        :type P: np.array
        :param R: control input weight matrix
        :type R: np.diag
        :param ulb: control lower bound, defaults to None
        :type ulb: np.array, optional
        :param uub: control upper bound, defaults to None
        :type uub: np.array, optional
        :param xlb: state lower bound, defaults to None
        :type xlb: np.array, optional
        :param xub: state upper bound, defaults to None
        :type xub: np.array, optional
        :param terminal_constraint: terminal constraint set, defaults to None
        :type terminal_constraint: np.array, optional
        """

        build_solver_time = -time.time()
        self.dt = model.dt
        self.Nx = model.n
        self.Nu = model.m
        self.Nt = int(horizon / self.dt)
        self.Tf = horizon

        # Initialize variables
        self.Q = Q
        self.P = P
        self.R = R

        # Initialize problem
        self.ocp = AcadosOcp()

        # Set model
        self.ocp.model = AcadosModel()
        self.ocp.model.f_expl_expr = model.f_expl
        self.ocp.model.f_impl_expr = model.f_impl
        self.ocp.model.disc_dyn_expr = model.disc_dyn_expr
        self.ocp.model.x = model.x
        self.ocp.model.u = model.u
        self.ocp.model.name = model.name

        # Set cost function
        self.ocp.cost.cost_type_0 = 'EXTERNAL'
        self.ocp.cost.cost_type = 'EXTERNAL'
        self.ocp.cost.cost_type_e = 'EXTERNAL'

        # Get Variables
        self.x_var = model.x
        self.u_var = model.u
        self.xr_var = ca.MX.sym('ref', self.Nx, 1)
        self.ocp.model.p = self.xr_var
        self.ocp.parameter_values = np.zeros(self.Nx)

        # Set cost functions
        self.ocp.model.cost_expr_ext_cost = self.running_cost(self.x_var,
                                                              self.xr_var,
                                                              self.Q,
                                                              self.u_var,
                                                              self.R)

        self.ocp.model.cost_expr_ext_cost_0 = self.running_cost(self.x_var,
                                                                self.xr_var,
                                                                self.Q,
                                                                self.u_var,
                                                                self.R)

        self.ocp.model.cost_expr_ext_cost_e = self.terminal_cost(self.x_var,
                                                                 self.xr_var,
                                                                 self.P)

        # Set constraints
        self.ocp.constraints.constr_type = 'BGH'

        self.ocp.constraints.lbu = ulb
        self.ocp.constraints.ubu = uub

        self.ocp.constraints.lbx = xlb
        self.ocp.constraints.ubx = xub

        # self.ocp.model.con_h_expr = # CASADI EXPR
        # self.ocp.constraints.lh = # Lower lower bound
        # self.ocp.constraints.uh = # Upper lower bound

        self.ocp.constraints.x0 = np.zeros(self.Nx)

        # If terminal_constraint is present, define terminal set
        if terminal_constraint is not None:
            self.ocp.constraints.ubx_e = terminal_constraint
            self.ocp.constraints.lbx_e = -terminal_constraint
            self.ocp.constraints.idxbx_e = np.array(range(self.Nx))

        self.ocp.constraints.idxbu = np.array(range(self.Nu))
        self.ocp.constraints.idxbx = np.array(range(self.Nx))

        # QP Solver before: FULL_CONDENSING_QPOASES
        self.ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        self.ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        self.ocp.solver_options.integrator_type = 'ERK'
        self.ocp.solver_options.nlp_solver_type = 'SQP_RTI'  # SQP_RTI
        self.ocp.solver_options.print_level = 5

        self.ocp.solver_options.qp_solver_cond_N = self.Nt
        self.ocp.dims.N = self.Nt

        self.ocp.solver_options.tf = self.Tf

        self.acados_ocp_solver = AcadosOcpSolver(self.ocp,
                                    json_file='acados_ocp_astrobee.json')

        build_solver_time += time.time()
        print('\n________________________________________')
        print('# Receding horizon length: %d ' % self.Nt)
        print('# Time to build mpc solver: %f sec' % build_solver_time)
        print('----------------------------------------')
        pass

    def set_options_dicts(self):
        pass

    def running_cost(self, x, xr, Q, u, R):
        """
        Running cost function.

        :param x: state
        :type x: ca.MX
        :param xr: desired state
        :type xr: ca.MX replaced by numpy array
        :param Q: weight matrix
        :type Q: numpy array
        :param u: control input
        :type u: ca.MX
        :param R: control weight matrix
        :type R: numpy array
        :return: cost
        :rtype: ca.MX expression
        """
        # Prepare variables
        p = x[0:3]
        v = x[3:6]
        q = x[6:10]
        w = x[10:]

        pr = xr[0:3]
        vr = xr[3:6]
        qr = xr[6:10]
        wr = xr[10:]

        # Calculate errors
        ep = p - pr
        ev = v - vr
        ew = w - wr
        eq = 0.5*inv_skew(ca.mtimes(r_mat(qr).T, r_mat(q))
                          - ca.mtimes(r_mat(q).T, r_mat(qr)))

        e_vec = ca.vertcat(*[ep, ev, eq, ew])

        # Calculate running cost
        ln = ca.mtimes(ca.mtimes(e_vec.T, Q), e_vec) \
            + ca.mtimes(ca.mtimes(u.T, R), u)

        return ln

    def terminal_cost(self, x, xr, P):
        """
        Terminal cost function.

        :param x: state
        :type x: ca.MX
        :param xr: desired state
        :type xr: ca.MX
        :param P: terminal cost weight matrix
        :type P: numpy array
        :return: cost value
        :rtype: ca.MX expression
        """
        # Prepare variables
        p = x[0:3]
        v = x[3:6]
        q = x[6:10]
        w = x[10:]

        pr = xr[0:3]
        vr = xr[3:6]
        qr = xr[6:10]
        wr = xr[10:]

        # Calculate errors
        ep = p - pr
        ev = v - vr
        ew = w - wr
        eq = 0.5*inv_skew(ca.mtimes(r_mat(qr).T, r_mat(q))
                          - ca.mtimes(r_mat(q).T, r_mat(qr)))

        e_vec = ca.vertcat(*[ep, ev, eq, ew])

        # Calculate running cost
        ln = ca.mtimes(ca.mtimes(e_vec.T, P), e_vec)

        return ln

    def solve_mpc(self, x0, u0=None):
        """
        Solve the MPC problem

        :param x0: state
        :type x0: ca.DM
        :param u0: initia guess for the control input, defaults to None
        :type u0: ca.DM, optional
        :return: predicted states and control inputs
        :rtype: ca.DM ca.DM vectors
        """

        # Initial state
        if u0 is None:
            u0 = np.zeros(self.Nu)
        if self.x_sp is None:
            self.x_sp = np.zeros(self.Nx)

        # Initialize variables
        self.optvar_x0 = np.full((1, self.Nx), x0.T)

        # Initial guess of the warm start variables
        self.optvar_init = self.opt_var(0)
        self.optvar_init['x', 0] = self.optvar_x0[0]

        param = ca.vertcat(x0, self.x_sp, u0)
        args = dict(x0=self.optvar_init,
                    lbx=self.optvar_lb,
                    ubx=self.optvar_ub,
                    lbg=self.con_lb,
                    ubg=self.con_ub,
                    p=param)

        # Solve NLP - TODO fix this
        self.solve_time = -time.time()
        sol = self.solver(**args)
        self.solve_time += time.time()
        # status = self.solver.stats()['return_status'] # IPOPT
        status = self.solver.stats()['success']  # SCPGEN
        optvar = self.opt_var(sol['x'])

        print('\nSolver status: ', status)
        print('MPC took %f seconds to solve.' % (self.solve_time))
        print('MPC cost: ', sol['f'])

        return optvar['x'], optvar['u']

    def mpc_controller(self, x0, _):
        """
        MPC interface.

        :param x0: initial state
        :type x0: ca.DM
        :param u0: initial guess for control input, defaults to None
        :type u0: ca.DM, optional
        :return: first control input
        :rtype: ca.DM
        """

        _, u_pred = self.solve_mpc(x0)

        return u_pred[0], self.x_sp

    def set_reference(self, x_sp):
        """
        Set MPC reference.

        :param x_sp: reference for the state
        :type x_sp: ca.DM
        """
        self.x_sp = x_sp

    def get_last_solve_time(self):
        """
        Helper function that returns the last solver time.
        """
        return self.solve_time
