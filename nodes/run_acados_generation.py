#!/usr/bin/env python
import numpy as np
import scipy

from reswarm_dmpc.models.astrobee import Astrobee
from reswarm_dmpc.controllers.mpc_acados import MPC
from reswarm_dmpc.simulation import EmbeddedSimEnvironment

# Instantiante Model
abee = Astrobee(h=0.1)

# Instantiate controller
Q = np.diag([10, 10, 10, 0, 0, 0, 10, 10, 10, 1, 1, 1])
R = np.diag([1, 1, 1, 0.5, 0.5, 0.5])*0.0001
P = Q*10

ctl = MPC(model=abee,
          dynamics=abee.model,
          horizon=4,
          Q=Q, R=R, P=P,
          ulb=np.array([-1, -1, -1, -0.1, -0.1, -0.1]),
          uub=np.array([1, 1, 1, 0.1, 0.1, 0.1]),
          xlb=np.array([-1, -1, -1, -0.1, -0.1, -0.1,
                        -1, -1, -1, -1, -0.1, -0.1, -0.1]),
          xub=np.array([1, 1, 1, 0.1, 0.1, 0.1,
                        1, 1, 1, 1, 0.1, 0.1, 0.1]))
