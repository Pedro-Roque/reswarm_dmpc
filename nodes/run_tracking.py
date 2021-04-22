#!/usr/bin/env python
import numpy as np
import scipy

from reswarm_dmpc.models.astrobee import Astrobee
from reswarm_dmpc.controllers.trajectory.mpc_trajectory import TMPC
from reswarm_dmpc.simulation_trajectory import EmbeddedSimEnvironment

# Instantiante Model
abee = Astrobee(h=0.2, iface='casadi')

# Instantiate controller (to track a velocity)
Q = np.diag([10, 10, 10, 100, 100, 100, 100, 100, 100, 10, 10, 10])
R = np.diag([0.1, 0.1, 0.1, 0.5, 0.5, 0.5])
P = Q*100

ctl = TMPC(model=abee,
           dynamics=abee.model,
           horizon=2,
           solver_type='ipopt',
           Q=Q, R=R, P=P,
           ulb=[-0.6, -0.3, -0.3, -0.06, -0.03, -0.03],
           uub=[0.6, 0.3, 0.3, 0.06, 0.03, 0.03],
           xlb=[-10, -10, -10, -1, -1, -1,
                -1, -1, -1, -1, -0.1, -0.1, -0.1],
           xub=[10, 10, 10, 1, 1, 1,
                1, 1, 1, 1, 0.1, 0.1, 0.1])

# Sinusoidal Trajectory
abee.set_trajectory_type("SinusoidalOffset")
sim_env_full = EmbeddedSimEnvironment(model=abee,
                                      dynamics=abee.model,
                                      ctl_class=ctl,
                                      controller=ctl.mpc_controller,
                                      noise={"pos": 0.001, "vel": 0.002,
                                             "att": 0.001, "ang": 0.005},
                                      time=20, plot=True)
sim_env_full.use_trajectory_control(True)
sim_env_full.run([0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0])
