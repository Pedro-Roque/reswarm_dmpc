#!/usr/bin/env python
import numpy as np
import scipy

from reswarm_dmpc.models.astrobee import Astrobee
from reswarm_dmpc.controllers.formation.dmpc import DecentralizedFormationMPC
from reswarm_dmpc.simulation_trajectory import EmbeddedSimEnvironment
from reswarm_dmpc.reference_generation.sinusoidal import SinusoidalReference

"""
Test node to control 3 agents in a distributed formation. The geometry
is, at first, a line formation ( f2<-f1<-l ), followed by a star
formation ( f1<-l->f2 ).
"""

# Line formation
# - Instantiate Models
f_geom_l = np.array([[-1.0, 0, 0]]).T
f_geom_sl = np.array([[1.0, 0, 0], [-1.0, 0, 0]]).T
f_geom_f = np.array([[1.0, 0, 0]]).T

leader = Astrobee(h=0.2, iface='casadi', role='leader',
                  vmax=0.2, num_neighbours=1, fg=f_geom_l)
leader.test_rel_pos_dynamics()

sub_leader = Astrobee(h=0.2, iface='casadi', role='local_leader',
                      vmax=0.2, num_neighbours=2, fg=f_geom_sl)

follower = Astrobee(h=0.2, iface='casadi', role='follower',
                    vmax=0.2, num_neighbours=1, fg=f_geom_f)

# Create controllers for each agent
# - Leader
Q = np.diag([10, 10, 10, 100, 100, 100, 100, 100, 100, 10, 10, 10])
R = np.diag([0.1, 0.1, 0.1, 0.5, 0.5, 0.5])
P = Q*100
Qr = np.diag([10, 10, 10])
Pr = Qr*100

leader_ctl = DecentralizedFormationMPC(
        model=leader,
        dynamics=leader.model,
        horizon=2,
        solver_type='ipopt',
        Q=Q, R=R, P=P, Qr=Qr, Pr=Pr,
        ulb=[-0.6, -0.3, -0.3, -0.06, -0.03, -0.03],
        uub=[0.6, 0.3, 0.3, 0.06, 0.03, 0.03],
        xlb=[-10, -10, -10, -1, -1, -1,
             -1, -1, -1, -1, -0.1, -0.1, -0.1],
        xub=[10, 10, 10, 1, 1, 1,
             1, 1, 1, 1, 0.1, 0.1, 0.1]
    )

# Local leader - local leader trajectory should be fed by propagating a-priori
#                the local leader state
Q = np.diag([10, 10, 10, 100, 100, 100, 100, 100, 100, 10, 10, 10])
R = np.diag([0.1, 0.1, 0.1, 0.5, 0.5, 0.5])
P = Q*100
Qr = np.diag([10, 10, 10, 10, 10, 10])
Pr = Qr*100

local_leader_ctl = DecentralizedFormationMPC(
        model=sub_leader,
        dynamics=sub_leader.model,
        horizon=2,
        solver_type='ipopt',
        Q=Q, R=R, P=P, Qr=Qr, Pr=Pr,
        ulb=[-0.6, -0.3, -0.3, -0.06, -0.03, -0.03],
        uub=[0.6, 0.3, 0.3, 0.06, 0.03, 0.03],
        xlb=[-10, -10, -10, -1, -1, -1,
             -1, -1, -1, -1, -0.1, -0.1, -0.1],
        xub=[10, 10, 10, 1, 1, 1,
             1, 1, 1, 1, 0.1, 0.1, 0.1]
    )

# Follower - local leader trajectory should be fed by propagating a-priori
#            the local leader state
Q = np.diag([10, 10, 10, 100, 100, 100, 100, 100, 100, 10, 10, 10])
R = np.diag([0.1, 0.1, 0.1, 0.5, 0.5, 0.5])
P = Q*100
Qr = np.diag([10, 10, 10])
Pr = Qr*100

follower_ctl = DecentralizedFormationMPC(
        model=follower,
        dynamics=follower.model,
        horizon=2,
        solver_type='ipopt',
        Q=Q, R=R, P=P, Qr=Qr, Pr=Pr,
        ulb=[-0.6, -0.3, -0.3, -0.06, -0.03, -0.03],
        uub=[0.6, 0.3, 0.3, 0.06, 0.03, 0.03],
        xlb=[-10, -10, -10, -1, -1, -1,
             -1, -1, -1, -1, -0.1, -0.1, -0.1],
        xub=[10, 10, 10, 1, 1, 1,
             1, 1, 1, 1, 0.1, 0.1, 0.1]
    )

# -----------------
# At this point, we have a controller for each agent role. Now we need
# to prepare the simulation to solve the MPC problem for each agent, using
# each controller, and communicate the velocities information.
#
# Test section:
x0 = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]]).T
rg = SinusoidalReference(dt=.2, x0=x0)
rg.create_trajectory()
xr = rg.get_trajectory(0, 11)
u, x_pred, r_pred = leader_ctl.controller(x0, xr, np.array([[-1.1, 0, 0]]).T)

print("U: ", u)
print("X pred", x_pred)
print("R pred", r_pred)
