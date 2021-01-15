import os, shutil
import sys
import numpy as np

from quadcopter import Quadcopter
from mpc_euler_AC import MPC_EULER
from utils import *


if __name__ == "__main__":
    h = 0.02
    g, m, Ixx, Iyy, Izz, dynamics = read_default_params_quad('./srd370_2.yaml')
    quadcopter = Quadcopter(dynamic_type = dynamics,
                            h = h,
                            g = g,
                            Ix = Ixx,Iy = Iyy,Iz = Izz,
                            m = m)
    model = quadcopter.export_acados_model_with_discrete_rk4()

    Q, R, P, uub, ulb, xub, xlb, xt, horizon = read_default_params_mpc('./Exp_1.yaml')

    target_controller = MPC_EULER(dynamics=model,
                                    horizon=horizon,
                                    h = h,
                                    Q = Q , R = R, P = P,
                                    ulb=ulb, uub=uub, 
                                    xlb=xlb, 
                                    xub=xub,
                                    terminal_constraint=xt,)

