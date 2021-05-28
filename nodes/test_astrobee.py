#!py
import unittest
import numpy as np
from reswarm_dmpc.models.astrobee import Astrobee


class AstrobeeTestClass(unittest.TestCase):
    """
    Test units for the Astrobee class.
    """

    def test_dynamics_steady_state(self):
        """
        Test discrete-dynamics function.

        Checks if the system remains at steady-state
        with a steady-state input.
        """
        abee = Astrobee(h=0.2, iface='casadi')
        x0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0])
        u0 = np.array([0, 0, 0, 0, 0, 0])
        x1 = abee.test_dynamics(x0, u0)

        # Expected solution:
        x1_correct = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]]).reshape(13, 1)

        self.assertTrue(np.array_equal(x1, x1_correct))

    def test_dynamics_input(self):
        """
        Test discrete-dynamics function.

        Checks if the system achieves the expected setpoint,
        x1_correct, with the specified input and initial condition.

        x1_correct is obtained using the discrete-time dynamics of the system
        in a nominal operation.
        """
        abee = Astrobee(h=0.2, iface='casadi')
        x0 = np.array([0.1, -0.1, 1, 0.01, 0.1, 0.02, 0, 0, 0, 1, 0.3, 0.5, 0.1])
        u0 = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        x1 = abee.test_dynamics(x0, u0)
        x1_round = np.around(x1, decimals=5)

        # Expected solution:
        x1_correct = np.array([[0.10229, -0.07972, 1.00428,
                                0.01297, 0.1028, 0.02279,
                                0.03932, 0.05912, 0.01915, 0.99729,
                                0.48467, 0.68467, 0.28467]]).reshape(13, 1)

        self.assertTrue(np.array_equal(x1_round, x1_correct))


if __name__ == '__main__':
    unittest.main()
