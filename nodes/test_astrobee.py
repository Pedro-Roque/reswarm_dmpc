#!py
import unittest
from matplotlib.pyplot import xcorr
import numpy as np
from reswarm_dmpc.models.astrobee import Astrobee


class AstrobeeTestClass(unittest.TestCase):
    """
    Test units for the Astrobee class.
    """

    def test_dynamics_steady_state(self):
        """
        Test discrete-dynamics function.
        Check steady-state.
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
        Check behavior with system input.
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

    def test_set_trajectory_method(self):
        """
        Check trajectory changing method.
        """
        abee = Astrobee(h=0.2, iface='casadi')
        abee.set_trajectory_type("SinusoidalOffset")

        self.assertTrue("SinusoidalOffset" == abee.trajectory_type)

    def test_trajectory_generation_sinusoidal(self):
        """
        Check trajectory generation method for sinusoidal
        starting at x0.
        """
        abee = Astrobee(h=0.2, iface='casadi')
        abee.set_trajectory_type("Sinusoidal")

        x0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0])
        traj = abee.get_trajectory(x0, t0=0, npoints=5)
        traj_round = np.around(traj, decimals=4)

        traj_correct = np.array([[0.    , 0.02  , 0.0398, 0.0592, 0.0778],
                                 [0.    , 0.    , 0.0025, 0.0075, 0.0148],
                                 [0.    , 0.01  , 0.02  , 0.03  , 0.04  ],
                                 [0.1   , 0.0992, 0.0969, 0.093 , 0.0876],
                                 [0.    , 0.0125, 0.0249, 0.0368, 0.0482],
                                 [0.05  , 0.05  , 0.05  , 0.05  , 0.05  ],
                                 [0.    , 0.    , 0.    , 0.    , 0.    ],
                                 [0.    , 0.    , 0.    , 0.    , 0.    ],
                                 [0.    , 0.    , 0.    , 0.    , 0.    ],
                                 [1.    , 1.    , 1.    , 1.    , 1.    ],
                                 [0.    , 0.    , 0.    , 0.    , 0.    ],
                                 [0.    , 0.    , 0.    , 0.    , 0.    ],
                                 [0.    , 0.    , 0.    , 0.    , 0.    ]])

        self.assertTrue(np.array_equal(traj_round, traj_correct))

    def test_trajectory_generation_sinusoidal_offset(self):
        """
        Check trajectory generation method for sinusoidal
        starting at an offset from x0.
        """
        abee = Astrobee(h=0.2, iface='casadi')
        abee.set_trajectory_type("SinusoidalOffset")

        x0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0])
        traj = abee.get_trajectory(x0, t0=0, npoints=5)
        traj_round = np.around(traj, decimals=4)

        traj_correct = np.array([[-1.000e-01, -9.800e-02, -9.600e-02, -9.400e-02, -9.200e-02],
                                 [ 1.000e-01,  1.000e-01,  1.000e-01,  1.001e-01,  1.002e-01],
                                 [ 0.000e+00,  1.000e-03,  2.000e-03,  3.000e-03,  4.000e-03],
                                 [ 1.000e-02,  1.000e-02,  1.000e-02,  1.000e-02,  1.000e-02],
                                 [ 0.000e+00,  1.000e-04,  3.000e-04,  4.000e-04,  5.000e-04],
                                 [ 5.000e-03,  5.000e-03,  5.000e-03,  5.000e-03,  5.000e-03],
                                 [ 0.000e+00,  0.000e+00,  0.000e+00,  0.000e+00,  0.000e+00],
                                 [ 0.000e+00,  0.000e+00,  0.000e+00,  0.000e+00,  0.000e+00],
                                 [ 0.000e+00,  0.000e+00,  0.000e+00,  0.000e+00,  0.000e+00],
                                 [ 1.000e+00,  1.000e+00,  1.000e+00,  1.000e+00,  1.000e+00],
                                 [ 0.000e+00,  0.000e+00,  0.000e+00,  0.000e+00,  0.000e+00],
                                 [ 0.000e+00,  0.000e+00,  0.000e+00,  0.000e+00,  0.000e+00],
                                 [ 0.000e+00,  0.000e+00,  0.000e+00,  0.000e+00,  0.000e+00]])

        self.assertTrue(np.array_equal(traj_round, traj_correct))

    def test_barrier_function_safe_state(self):
        """
        Test barrier condition if system is in C.
        """
        abee = Astrobee(h=0.2, iface='casadi')

        # Set variables:
        p = np.zeros((3, 1))
        pr = np.zeros((3, 1))
        v = np.zeros((3, 1))
        vr = np.zeros((3, 1))

        q = np.array([[0, 0, 0, 1]]).reshape(4, 1)
        qr = np.array([[0, 0, 0, 1]]).reshape(4, 1)
        w = np.zeros((3, 1))
        wr = np.zeros((3, 1))

        x = np.concatenate((p, v, q, w))
        x_r = np.concatenate((pr, vr, qr, wr))

        u = np.zeros((6, 1))

        hp_ineq, hq_ineq = abee.get_barrier_value(x, x_r, u)
        self.assertTrue(hp_ineq >= 0 and hq_ineq >= 0)

    def test_barrier_function_unsafe_position(self):
        """
        Test barrier condition if system has an unsafe position.
        """

        abee = Astrobee(h=0.2, iface='casadi')

        # Set variables:
        p = np.ones((3, 1))*2
        pr = np.zeros((3, 1))
        v = np.zeros((3, 1))
        vr = np.zeros((3, 1))

        q = np.array([[0, 0, 0, 1]]).reshape(4, 1)
        qr = np.array([[0, 0, 0, 1]]).reshape(4, 1)
        w = np.zeros((3, 1))
        wr = np.zeros((3, 1))

        x = np.concatenate((p, v, q, w))
        x_r = np.concatenate((pr, vr, qr, wr))

        u = np.zeros((6, 1))

        hp_ineq, hq_ineq = abee.get_barrier_value(x, x_r, u)
        self.assertTrue(hp_ineq < 0 and hq_ineq >= 0)
    
    def test_barrier_function_unsafe_attitude(self):
        """
        Test barrier condition if system has an unsafe attitude.
        """

        abee = Astrobee(h=0.2, iface='casadi')

        # Set variables:
        p = np.zeros((3, 1))
        pr = np.zeros((3, 1))
        v = np.zeros((3, 1))
        vr = np.zeros((3, 1))

        q = np.array([[0, 1, 0, 0]]).reshape(4, 1)
        qr = np.array([[0, 0, 0, 1]]).reshape(4, 1)
        w = np.zeros((3, 1))
        wr = np.zeros((3, 1))

        x = np.concatenate((p, v, q, w))
        x_r = np.concatenate((pr, vr, qr, wr))

        u = np.zeros((6, 1))

        hp_ineq, hq_ineq = abee.get_barrier_value(x, x_r, u)
        self.assertTrue(hp_ineq >= 0 and hq_ineq < 0)


if __name__ == '__main__':
    unittest.main()
