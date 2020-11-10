#!/usr/bin/env python
import numpy as np
import scipy

from reswarm_dmpc.models.astrobee import Astrobee
from reswarm_dmpc.controllers.mpc_trajectory import TMPC
from reswarm_dmpc.simulation import EmbeddedSimEnvironment

# Part III - Full cart model
abee = Astrobee(h=0.05)

# Instantiate controller
Q = np.diag([0, 0, 0, 10, 10, 10, 100, 100, 100, 10, 10, 10])
R = np.diag([1, 1, 1, 0.5, 0.5, 0.5])*2
P = Q*100

ctl = TMPC(model=abee,
           dynamics=abee.model,
           horizon=.5,
           Q=Q, R=R, P=P,
           ulb=[-1, -1, -1, -0.1, -0.1, -0.1],
           uub=[1, 1, 1, 0.1, 0.1, 0.1],
           xlb=[-1, -1, -1, -0.1, -0.1, -0.1,
                -1, -1, -1, -1, -0.1, -0.1, -0.1],
           xub=[1, 1, 1, 0.1, 0.1, 0.1,
                1, 1, 1, 1, 0.1, 0.1, 0.1])

# Sinusoidal Trajectory
abee.set_trajectory_type("Sinusoidal")
sim_env_full = EmbeddedSimEnvironment(model=abee,
                                      dynamics=abee.model,
                                      controller=ctl.mpc_controller,
                                      time=20)
sim_env_full.use_trajectory_control(True)
sim_env_full.run([0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0])
