#!/usr/bin/env python
import numpy as np
import scipy

from reswarm_dmpc.models.astrobee import Astrobee
from reswarm_dmpc.controllers.mpc_trajectory import TMPC
from reswarm_dmpc.simulation_trajectory import EmbeddedSimEnvironment

"""
Test node to control 3 agents in a distributed formation. The geometry
is, at first, a line formation ( f2<-f1<-l ), followed by a star
formation ( f1<-l->f2 ).
"""

# Line formation
#
# Instantiate Models
f_geom = np.array([[-1, 0, 0]]).T
leader = Astrobee(h=0.2, iface='casadi', role='leader',
                  vmax=0.2, num_neighbours=1, fg=f_geom)

f_geom = np.array([[1, 0, 0], [-1, 0, 0]]).T
sub_leader = Astrobee(h=0.2, iface='casadi', role='local_leader',
                      vmax=0.2, num_neighbours=2, fg=f_geom)

f_geom = np.array([[1, 0, 0]]).T
follower = Astrobee(h=0.2, iface='casadi', role='follower',
                    vmax=0.2, num_neighbours=1, fg=f_geom)

# Create controllers for each agent
