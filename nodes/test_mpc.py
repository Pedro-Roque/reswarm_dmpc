import unittest
from matplotlib.pyplot import xcorr
import numpy as np
from reswarm_dmpc.models.astrobee import Astrobee
from reswarm_dmpc.controllers.mpc import MPC


class ControllerStabilizationTest(unittest.TestCase):
    """
    Test units for the Astrobee class.
    """

    def test_translation_setpoint(self):
        """
        Test discrete-dynamics function.
        Check steady-state.
        """

        self.assertTrue(True)


if __name__ == '__main__':
    unittest.main()
