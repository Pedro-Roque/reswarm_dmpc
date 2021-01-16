#!/usr/bin/env python
import numpy as np
import scipy

from reswarm_dmpc.models.astrobee import Astrobee
from reswarm_dmpc.controllers.mpc_acados import MPC
from reswarm_dmpc.simulation import EmbeddedSimEnvironment

# Instantiante Model
abee = Astrobee(h=0.1)

# Instantiate controller
Q = np.diag([10, 10, 10, 100, 100, 100, 10, 10, 10, 100, 100, 100])
R = np.diag([1, 1, 1, 0.5, 0.5, 0.5])*10
P = Q*100

ctl = MPC(model=abee,
          dynamics=abee.model,
          horizon=.5,
          Q=Q, R=R, P=P,
          ulb=np.array([-1, -1, -1, -0.1, -0.1, -0.1]),
          uub=np.array([1, 1, 1, 0.1, 0.1, 0.1]),
          xlb=np.array([-1, -1, -1, -0.1, -0.1, -0.1,
                        -1, -1, -1, -1, -0.1, -0.1, -0.1]),
          xub=np.array([1, 1, 1, 0.1, 0.1, 0.1,
                        1, 1, 1, 1, 0.1, 0.1, 0.1]))
