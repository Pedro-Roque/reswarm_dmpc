#!/usr/bin/env python
import numpy as np
import scipy

from reswarm_dmpc.model import Astrobee
from reswarm_dmpc.mpc import DMPC
from reswarm_dmpc.simulation import EmbeddedSimEnvironment

# Part III - Full cart model
pendulum = Astrobee(mass=1, inertia=np.diag([1, 1, 1]))

# # Instantiate controller
# ctl = MPC(model=pendulum,
#           dynamics=pendulum.discrete_time_dynamics,
#           horizon=7,
#           Q=Q, R=R, P=P_LQR,
#           ulb=-10, uub=10,
#           xlb=[-2, -10, -np.pi/2, -np.pi/2],
#           xub=[12, 10, np.pi/2, np.pi/2])

# # Solve without disturbance
# ctl.set_reference(x_sp=np.array([10, 0, 0, 0]))
# sim_env_full = EmbeddedSimEnvironment(model=pendulum,
#                                       dynamics=pendulum.model,
#                                       controller=ctl.mpc_controller,
#                                       time=6)
# sim_env_full.run([0, 0, 0, 0])
