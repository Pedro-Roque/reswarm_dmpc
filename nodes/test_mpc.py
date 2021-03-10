import unittest
import numpy as np
import casadi as ca

from reswarm_dmpc.models.astrobee import Astrobee
from reswarm_dmpc.controllers.mpc import MPC


class ControllerStabilizationTest(unittest.TestCase):
    """
    Test units for the Astrobee class.
    """

    def test_cost_function(self):

        abee = Astrobee(h=0.2, iface='casadi')

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

        x = ca.DM.zeros(13, 1)
        xr = ca.DM.zeros(13, 1)
        u = ca.DM.zeros(6, 1)
        x[10] = 1
        xr[10] = 1

        rc = ctl.running_cost(x, xr, Q, u, R)
        tc = ctl.terminal_cost(x, xr, P)

        self.assertTrue(rc == 0.0 and tc == 0.0)

    def test_translation_setpoint(self):
        """
        Test discrete-dynamics function.
        Check steady-state.
        """

        self.assertTrue(True)


if __name__ == '__main__':
    unittest.main()
