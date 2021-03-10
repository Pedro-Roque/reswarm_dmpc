import unittest
import numpy as np
import casadi as ca

from reswarm_dmpc.models.astrobee import Astrobee
from reswarm_dmpc.controllers.mpc import MPC
from reswarm_dmpc.simulation import EmbeddedSimEnvironment


class ControllerStabilizationTest(unittest.TestCase):
    """
    Test units for the Astrobee class.
    """

    def test_cost_function(self):
        """
        Test case for cost function evaluation.
        """

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

    def test_setpoint_set(self):
        """
        Test setpoint setting function.
        """

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

        # Set reference
        xr = np.zeros((13, 1))
        xr[10] = 1
        ctl.set_reference(xr)

        self.assertTrue(np.array_equal(ctl.x_sp, xr))

    def test_translation_setpoint(self):
        """
        Test translation setpoints.
        Evaluates if the controller can converge, in the default time,
        to the desired translation-only setpoint.
        """

        # Instantiante Model
        abee = Astrobee(h=0.1, iface='casadi')

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
                                              controller=ctl.mpc_controller)
        _, y, _, avg_t1 = sim_env_full.run([0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0])

        self.assertTrue(np.array_equal(np.around(y[:, -1], 2), x_ref))

    def test_attitude_setpoint(self):
        """
        Test attitude setpoints.
        Evaluates if the controller can converge, in the default time,
        to the desired attitude-only setpoint.
        """
        # Instantiante Model
        abee = Astrobee(h=0.1, iface='casadi')

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
        x_ref = np.array([0, 0, 0, 0, 0, 0, 0.189, 0.038, 0.269, 0.944, 0, 0, 0])
        ctl.set_reference(x_sp=x_ref)
        sim_env_full = EmbeddedSimEnvironment(model=abee,
                                              dynamics=abee.model,
                                              ctl_class=ctl,
                                              controller=ctl.mpc_controller, 
                                              time=20.0)
        _, y, _, avg_t1 = sim_env_full.run([0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0])

        self.assertTrue(np.array_equal(np.around(y[:, -1], 3), x_ref))


if __name__ == '__main__':
    unittest.main()
