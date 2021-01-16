#!/usr/bin/env python
import numpy as np
import scipy

from reswarm_dmpc.models.astrobee import Astrobee
from reswarm_dmpc.controllers.mpc import MPC
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
          ulb=[-1, -1, -1, -0.1, -0.1, -0.1],
          uub=[1, 1, 1, 0.1, 0.1, 0.1],
          xlb=[-1, -1, -1, -0.1, -0.1, -0.1,
               -1, -1, -1, -1, -0.1, -0.1, -0.1],
          xub=[1, 1, 1, 0.1, 0.1, 0.1,
               1, 1, 1, 1, 0.1, 0.1, 0.1])

# Position setpoint
x_ref = np.array([.5, .5, .5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0])
ctl.set_reference(x_sp=x_ref)
sim_env_full = EmbeddedSimEnvironment(model=abee,
                                      dynamics=abee.model,
                                      ctl_class=ctl,
                                      controller=ctl.mpc_controller,
                                      time=20)
_, _, _, avg_t1 = sim_env_full.run([0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0])

# Attitude setpoint
x_ref = np.array([0, 0, 0, 0, 0, 0, 0.189, 0.038, 0.269, 0.944, 0, 0, 0])
ctl.set_reference(x_sp=x_ref)
sim_env_full = EmbeddedSimEnvironment(model=abee,
                                      dynamics=abee.model,
                                      ctl_class=ctl,
                                      controller=ctl.mpc_controller,
                                      time=20)
_, _, _, avg_t2 = sim_env_full.run([0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0])

# Pose setpoint
x_ref = np.array([0.5, 0.5, 0.5, 0, 0, 0, 0.189, 0.038, 0.269, 0.944, 0, 0, 0])
ctl.set_reference(x_sp=x_ref)
sim_env_full = EmbeddedSimEnvironment(model=abee,
                                      dynamics=abee.model,
                                      ctl_class=ctl,
                                      controller=ctl.mpc_controller,
                                      time=20)
_, _, _, avg_t3 = sim_env_full.run([0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0])

# Print average times
print("Average times:\nT1: ", avg_t1, "\nT2: ", avg_t2, "\nT3: ", avg_t3)
