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
import numpy.matlib as nmp
import casadi as ca
import casadi.tools as ctools

from reswarm_dmpc.util import *


class TMPC(object):

    def __init__(self, model, dynamics,
                 Q, P, R, solver_type='sqpmethod', horizon=10,
                 **kwargs):
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

        self.solve_time = 0.0
        self.solver_type = solver_type
        self.dt = model.dt
        self.Nx = model.n
        self.Nu = model.m
        self.model = model
        self.Nt = int(horizon / self.dt)
        self.dynamics = dynamics

        # Initialize variables
        self.Q = ca.MX(Q)
        self.P = ca.MX(P)
        self.R = ca.MX(R)

        self.x_sp = None

        # Initialize bound variables
        if "xub" in kwargs:
            self.xub = kwargs["xub"]
        if "xlb" in kwargs:
            self.xlb = kwargs["xlb"]
        if "uub" in kwargs:
            self.uub = kwargs["uub"]
        if "ulb" in kwargs:
            self.ulb = kwargs["ulb"]
        if "terminal_constraint" in kwargs:
            self.tc_ub = np.full((self.Nx,), kwargs["terminal_constraint"])
            self.tc_lb = np.full((self.Nx,), -kwargs["terminal_constraint"])

        # Check if we are in formation context
        self.formation = False
        if hasattr(self.model, 'role'):
            self.formation = True
            self.role = self.model.role
            self.num_neighbours = self.model.num_neighbours
            self.fg = self.model.fg

            # Dynamics
            self.follower_dynamics = self.model.next_follower_rpos
            self.leader_dynamics = self.model.next_local_leader_rpos

        # Set internal solver properties
        self.set_options_dicts()
        if "use_jit" in kwargs:
            self.set_jit()

        self.set_cost_functions()
        self.test_cost_functions(Q, R, P)
        self.create_solver()

    def create_solver(self):
        """
        Instantiate the solver object.
        """

        build_solver_time = -time.time()

        # Starting state parameters - add slack here
        x0 = ca.MX.sym('x0', self.Nx)
        x_ref = ca.MX.sym('x_ref', self.Nx * (self.Nt + 1),)
        u0 = ca.MX.sym('u0', self.Nu)
        param_s = ca.vertcat(x0, x_ref, u0)
        if self.formation:
            r0 = ca.MX.sym('r0', 3 * self.num_neighbours)
            param_s = ca.vertcat(param_s, r0)
            if self.role == 'follower' or self.role == 'local_leader':
                vL = ca.MX.sym('r0', 3)
                param_s = ca.vertcat(param_s, vL)

        # Create optimization variables
        opt_var = ctools.struct_symMX([(ctools.entry('u', shape=(self.Nu,), repeat=self.Nt),
                                        ctools.entry('x', shape=(self.Nx,), repeat=self.Nt + 1),
                                        )])
        if self.formation:
            opt_var = ctools.struct_symMX([(
                ctools.entry('u', shape=(self.Nu,), repeat=self.Nt),
                ctools.entry('x', shape=(self.Nx,), repeat=self.Nt + 1),
                ctools.entry('r', shape=(3 * self.num_neighbours,), repeat=self.Nt + 1),
            )])

        self.opt_var = opt_var
        self.num_var = opt_var.size

        # Decision variable boundries
        self.optvar_lb = opt_var(-np.inf)
        self.optvar_ub = opt_var(np.inf)

        # Set initial values
        obj = ca.MX(0)
        self.con_eq = []
        self.con_ineq = []
        self.con_ineq_lb = []
        self.con_ineq_ub = []
        self.con_eq.append(opt_var['x', 0] - x0)

        # Set initial formation variables
        if self.formation:
            self.con_eq.append(opt_var['r', 0] - r0)

        # Generate MPC Problem
        for t in range(self.Nt):
            # Get variables
            x_t = opt_var['x', t]
            x_r = x_ref[(t * 13):(t * 13 + 13)]
            u_t = opt_var['u', t]
            if self.formation:
                r_t = opt_var['r', t]

            # Dynamics constraint
            x_t_next = self.dynamics(x_t, u_t)
            self.con_eq.append(x_t_next - opt_var['x', t + 1])

            # Input constraints
            if self.uub is not None:
                self.set_upper_bound_constraint(u_t, self.uub)
            if self.ulb is not None:
                self.set_lower_bound_constraint(u_t, self.ulb)

            # State constraints
            if self.xub is not None:
                self.set_upper_bound_constraint(x_t, self.xub)
            if self.xlb is not None:
                self.set_lower_bound_constraint(x_t, self.xlb)

            if self.formation:
                # Set propagation rules for formation context
                if self.role == 'leader':
                    v = x_t[3:6]
                    r_next = ca.MX.sym('r_next')
                    for i in range(self.num_neighbours):
                        rF = r_t[(i * 3):(3 * i + 3)]
                        r_d = self.fg[:, i]
                        r_next_f = self.follower_dynamics(v, rF, r_d)
                        r_next = ca.vertcat(r_next, r_next_f)

                elif self.role == 'local_leader':
                    r_next = ca.MX.sym('r_next')
                    v = x_t[3:6]
                    q = x_t[6:10]
                    rL = r_t[0:3]
                    r_next_l = self.leader_dynamics(v, q, rL, vL)
                    r_next = ca.vertcat(r_next, r_next_l)
                    for i in range(1, self.num_neighbours - 1, 1):
                        rF = r_t[(i * 3):(3 * i + 3)]
                        r_d = self.fg[:, i]
                        r_next_f = self.follower_dynamics(v, rF, r_d)
                        r_next = ca.vertcat(r_next, r_next_f)

                else:  # Follower only needs to propagate leader
                    v = x_t[3:6]
                    q = x_t[6:10]
                    rL = r_t[0:3]
                    r_next = self.leader_dynamics(v, q, rL, vL)

                # Set dynamics constraint for neighbours
                self.con_eq.append(r_next - opt_var['r', t + 1])

            # Objective Function / Cost Function
            if self.formation:
                if self.role == 'leader':
                    obj += self.running_cost(x_t, x_r, self.Q, u_t, self.R)
                elif self.role == 'local_leader':
                    pass
            else:
                obj += self.running_cost(x_t, x_r, self.Q, u_t, self.R)

        # Terminal Cost
        obj += self.terminal_cost(opt_var['x', self.Nt],
                                  x_ref[self.Nt * 13:], self.P)

        # Terminal constraint
        if self.tc_ub is not None and self.tc_lb is not None:
            self.set_lower_bound_constraint(opt_var['x', self.Nt] - x_ref[self.Nt * 13:],
                                            self.tc_lb)
            self.set_upper_bound_constraint(opt_var['x', self.Nt] - x_ref[self.Nt * 13:],
                                            self.tc_ub)

        # Equality constraints bounds are 0 (they are equality constraints),
        # -> Refer to CasADi documentation
        num_eq_con = ca.vertcat(*self.con_eq).size1()
        num_ineq_con = ca.vertcat(*self.con_ineq).size1()
        con_eq_lb = np.zeros((num_eq_con, 1))
        con_eq_ub = np.zeros((num_eq_con, 1))

        # Set constraints
        con = ca.vertcat(*(self.con_eq + self.con_ineq))
        self.con_lb = ca.vertcat(con_eq_lb, *self.con_ineq_lb)
        self.con_ub = ca.vertcat(con_eq_ub, *self.con_ineq_ub)
        nlp = dict(x=opt_var, f=obj, g=con, p=param_s)

        # Instantiate solver
        if self.solver_type == "sqpmethod":
            self.solver = ca.nlpsol('mpc_solver', 'sqpmethod', nlp,
                                    self.sol_options_sqp)
        elif self.solver_type == "ipopt":
            self.solver = ca.nlpsol('mpc_solver', 'ipopt', nlp,
                                    self.sol_options_ipopt)
        else:
            raise ValueError("Wrong solver selected.")
        build_solver_time += time.time()
        print('\n________________________________________')
        print('# Receding horizon length: %d ' % self.Nt)
        print('# Time to build mpc solver: %f sec' % build_solver_time)
        print('# Number of variables: %d' % self.num_var)
        print('# Number of equality constraints: %d' % num_eq_con)
        print('# Number of inequality constraints: %d' % num_ineq_con)
        print('----------------------------------------')
        pass

    def set_lower_bound_constraint(self, var, value):
        """
        Set a lower bound constraint for the variable var.
        :param var: variable to lower bound
        :type var: ca.MX, ca.DM
        :param value: lower bound value
        :type value: np.ndarray
        """
        self.con_ineq.append(var)
        self.con_ineq_ub.append(np.full((var.shape[0],), ca.inf))
        self.con_ineq_lb.append(value)

    def set_upper_bound_constraint(self, var, value):
        """
        Set a upper bound constraint for the variable var.
        :param var: variable to upper bound
        :type var: ca.MX, ca.DM
        :param value: upper bound value
        :type value: np.ndarray
        """
        self.con_ineq.append(var)
        self.con_ineq_lb.append(np.full((var.shape[0],), -ca.inf))
        self.con_ineq_ub.append(value)

    def set_options_dicts(self):
        """
        Helper function to set the dictionaries for solver and function options
        """

        # Functions options
        self.fun_options = None

        # Options for NLP Solvers
        # -> SQP Method
        qp_opts = {
            'max_iter': 10,
            'error_on_fail': False,
            'print_header': False,
            'print_iter': False
        }
        self.sol_options_sqp = {
            'max_iter': 3,
            'qpsol': 'qrqp',
            'convexify_margin': 1e-5,
            'print_header': False,
            'print_time': False,
            'print_iteration': False,
            'qpsol_options': qp_opts
        }

        # Options for IPOPT Solver
        # -> IPOPT
        self.sol_options_ipopt = {
            'ipopt.print_level': 0,
            'ipopt.warm_start_bound_push': 1e-4,
            'ipopt.warm_start_bound_frac': 1e-4,
            'ipopt.warm_start_slack_bound_frac': 1e-4,
            'ipopt.warm_start_slack_bound_push': 1e-4,
            'ipopt.warm_start_mult_bound_push': 1e-4,
            'print_time': False,
            'verbose': False,
            'expand': True
        }

        return True

    def set_jit(self):
        """
        Helper function to update the dictionaries with jit attributed.
        """

        self.fun_options = {
            "jit": True,
            "jit_options": {'compiler': 'ccache gcc',
                            'flags': ["-O2", "-pipe"]},
            'compiler': 'shell',
            'jit_temp_suffix': True
        }

        self.sol_options_sqp.update({
            "jit": True,
            "jit_options": {'compiler': 'ccache gcc',
                            'flags': ["-O2", "-pipe"]},
            'compiler': 'shell',
            'jit_temp_suffix': False})

        self.sol_options_ipopt.update({
            "jit": True,
            "jit_options": {"flags": ["-O2"]}})

    def set_solver_dictionaries(self, nlp):
        """
        Helper function to create the solver dictionares to abstract the need
        for different options for each solver.

        :param nlp: NLP problem being solved.
        :type nlp: ca.nlpsol
        """

        self.solver_dict = {
            'sqpmethod': ca.nlpsol('mpc_solver', 'sqpmethod', nlp,
                                   self.sol_options_sqp),
            'ipopt': ca.nlpsol('mpc_solver', 'ipopt', nlp,
                               self.sol_options_ipopt)
        }

    def set_cost_functions(self):
        """
        Helper function to setup the cost functions.
        """

        # Create functions and function variables for calculating the cost
        Q = ca.MX.sym('Q', self.Nx - 1, self.Nx - 1)
        P = ca.MX.sym('P', self.Nx - 1, self.Nx - 1)
        R = ca.MX.sym('R', self.Nu, self.Nu)

        x = ca.MX.sym('x', self.Nx)
        xr = ca.MX.sym('xr', self.Nx)
        u = ca.MX.sym('u', self.Nu)

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
        eq = 0.5 * inv_skew(ca.mtimes(r_mat(qr).T, r_mat(q))
                            - ca.mtimes(r_mat(q).T, r_mat(qr)))

        if not self.formation:
            e_vec = ca.vertcat(*[ep, ev, eq, ew])

            # Calculate running cost
            ca.mtimes(ca.mtimes(e_vec.T, Q), e_vec)
            ln = ca.mtimes(ca.mtimes(e_vec.T, Q), e_vec) \
                + ca.mtimes(ca.mtimes(u.T, R), u)

            self.running_cost = ca.Function('ln', [x, xr, Q, u, R], [ln],
                                            self.fun_options)

            # Calculate terminal cost
            V = ca.mtimes(ca.mtimes(e_vec.T, P), e_vec)
            self.terminal_cost = ca.Function('V', [x, xr, P], [V],
                                             self.fun_options)
        else:
            if self.role == 'leader':
                # Set leader cost functions
                pass
            elif self.role == 'local_leader':
                # Set follower cost functions
                pass
            else:
                pass

        return

    def test_cost_functions(self, Q, R, P):
        """
        Helper function to test the cost functions.
        """
        x = ca.DM.zeros(13, 1)
        xr = ca.DM.zeros(13, 1)
        u = ca.DM.zeros(6, 1)
        x[10] = 1
        xr[10] = 1

        print("Running cost:", self.running_cost(x, xr, Q, u, R))

        print("Terminal cost:", self.terminal_cost(x, xr, P))

        return

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
            self.x_sp = nmp.repmat(np.array([[0, 0.5, 0,
                                              0, 0, 0,
                                              0, 0, 0, 1,
                                              0, 0, 0]]).T, self.Nt + 1, 1)
            print("Setpoint dimension:", np.size(self.x_sp))

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
        solve_time = -time.time()
        sol = self.solver(**args)
        solve_time += time.time()
        status = None
        if self.solver_type == "ipopt":
            status = self.solver.stats()['return_status']
        elif self.solver_type == "sqpmethod":
            status = self.solver.stats()['success']
        optvar = self.opt_var(sol['x'])
        self.solve_time = solve_time

        print ('Solver status: ', status)
        # print('MPC took %f seconds to solve.' % (solve_time))
        print('MPC cost: ', sol['f'])

        return optvar['x'], optvar['u']

    def mpc_controller(self, x0, t0):
        """
        MPC interface.

        :param x0: initial state
        :type x0: ca.DM
        :param u0: initial guess for control input, defaults to None
        :type u0: ca.DM, optional
        :return: first control input
        :rtype: ca.DM
        """
        # Generate trajectory from t0 and x0
        x_sp_vec = self.model.get_trajectory(x0, t0, self.Nt + 1)
        ref = x_sp_vec[:, 0]
        x_sp = x_sp_vec.reshape(self.Nx * (self.Nt + 1), order='F')
        self.set_reference(x_sp)

        x_pred, u_pred = self.solve_mpc(x0)

        return u_pred[0], ref, x_pred, x_sp_vec

    def set_reference(self, x_sp):
        """
        Set MPC reference.

        :param x_sp: reference for the state
        :type x_sp: ca.DM
        """
        self.x_sp = x_sp

    def get_last_solve_time(self):
        """
        Get time that took to solve the MPC problem.
        """
        return self.solve_time
