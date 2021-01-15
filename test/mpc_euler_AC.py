from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import time
import sys
import yaml
import os
import numpy as np
import scipy.linalg
from ctypes import *
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosModel
import casadi as ca 

from quadcopter import Quadcopter

class MPC_EULER(object):

    def __init__(self, dynamics, h = 0.05,
                 horizon=20, Q=None, P=None, R=None, E=None,PE=None,
                 ulb=None, uub=None, xlb=None, xub=None, terminal_constraint=None,
                 solver_opts=None,
                 x_d=[0]*12,
                ):

        build_solver_time = -time.time()

        self.dt = h
        self.Nx = dynamics.x.size()[0]
        self.Nu = dynamics.u.size()[0]
        self.Ny = self.Nx + self.Nu
        self.Ny_e = self.Nx
        self.Nt = int(horizon)
        self.Tf = int(horizon) * self.dt
        self.dynamics = dynamics

        ocp = AcadosOcp()
        ocp.model = dynamics

        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'

        # Cost function weights
        if P is None:
            P = np.eye(self.Nx) * 10
        if Q is None:
            Q = np.eye(self.Nx)
        if R is None:
            R = np.eye(self.Nu) * 0.01

        unscale = self.Nt / self.Tf
        ocp.cost.W = scipy.linalg.block_diag(Q, R)
        ocp.cost.W_e = P

        ocp.cost.Vx = np.zeros((self.Ny, self.Nx))
        ocp.cost.Vx[:self.Nx,:self.Nx] = np.eye(self.Nx)

        ocp.cost.Vu = np.zeros((self.Ny, self.Nu))
        ocp.cost.Vu[self.Nx:,:] = np.eye(self.Nu)

        ocp.cost.Vx_e = np.eye(self.Nx)
        
        ocp.cost.yref  = np.zeros((self.Ny, ))
        ocp.cost.yref_e = np.zeros((self.Ny_e, ))
        
        if xub is None:
            xub = np.full((self.Nx), np.inf)
        if xlb is None:
            xlb = np.full((self.Nx), -np.inf)
        if uub is None:
            uub = np.full((self.Nu), np.inf)
        if ulb is None:
            ulb = np.full((self.Nu), -np.inf)

        self.lbx = xlb
        self.ubx = xub
        self.lbu = ulb
        self.ubu = uub
        self.terminal_constraint = terminal_constraint

        ocp.constraints.constr_type = 'BGH'

        ocp.constraints.lbu = self.lbu
        ocp.constraints.ubu = self.ubu

        ocp.constraints.lbx = self.lbx
        ocp.constraints.ubx = self.ubx

        ocp.constraints.x0 = np.zeros(self.Nx)

        if terminal_constraint is not None:
            ocp.constraints.ubx_e = terminal_constraint
            ocp.constraints.lbx_e = -terminal_constraint
            ocp.constraints.idxbx_e = np.array(range(self.Nx)) # Assign constraint index to input index
            
        ocp.constraints.idxbu = np.array(range(self.Nu)) # Assign constraint index to input index
        ocp.constraints.idxbx = np.array(range(self.Nx)) # Assign constraint index to input index
        

        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP' # SQP_RTI

        ocp.solver_options.qp_solver_cond_N = self.Nt
        ocp.dims.N = self.Nt
        
        ocp.solver_options.tf = self.Tf
     
        self.acados_ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp_' + dynamics.name + '.json')

        print("NMPC acados solver initialized.\nTime step {} and horizon {}.\nInput constraints:\n higher {}\n lower {}.\nState constraints:\n higher {}, \n lower {}.".format(h, horizon, self.ubu, self.lbu, self.ubx, self.lbx))

        pass

    def solve_mpc(self, x0, x_ref):
   
        self.acados_ocp_solver.set(0, "lbx", x0)
        self.acados_ocp_solver.set(0, "ubx", x0)

        for i in range(self.Nt):
            yref = np.array(x_ref[self.Nx*i:self.Nx*i + self.Nx].tolist() + [14.715, 0.0, 0.0, 0.0])
            self.acados_ocp_solver.set(i, "yref", yref)

            self.acados_ocp_solver.constraints_set(i, "lbu", self.lbu)
            self.acados_ocp_solver.constraints_set(i, "ubu", self.ubu)

            if i > 0:
                self.acados_ocp_solver.constraints_set(i, "lbx", self.lbx)
                self.acados_ocp_solver.constraints_set(i, "ubx", self.ubx)
        
        if self.terminal_constraint is not None:
            self.acados_ocp_solver.constraints_set(self.Nt, "lbx", -self.terminal_constraint)
            self.acados_ocp_solver.constraints_set(self.Nt, "ubx", self.terminal_constraint)
        
         
        yref = x_ref[self.Nx*self.Nt:self.Nx*self.Nt + self.Nx]
        self.acados_ocp_solver.set(self.Nt, "yref", yref)

        # solve ocp
        t = time.time()

        status = self.acados_ocp_solver.solve()
        if status != 0:
            print("acados returned status {} in closed loop iteration {}.".format(status, i))

        elapsed = time.time() - t
        
        x_solve = [self.acados_ocp_solver.get(i, "x") for i in range(self.Nt + 1)]
        u_solve = [self.acados_ocp_solver.get(i, "u") for i in range(self.Nt)]
        
        
        return x_solve, u_solve, elapsed

    def mpc_controller(self, x0,x_ref, u0=None):
        
        x_pred, u_pred, t_solve = self.solve_mpc(x0, x_ref)
        return x_pred, u_pred, t_solve


    
