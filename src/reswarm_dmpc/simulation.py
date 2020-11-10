from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
import time


class EmbeddedSimEnvironment(object):
    def __init__(self, model, dynamics, controller, time=100.0):
        """
        Embedded simulation environment. Simulates the syste given
        dynamics and a control law, plots in matplotlib.

        :param model: model object
        :type model: object
        :param dynamics: system dynamics function (x, u)
        :type dynamics: casadi.DM
        :param controller: controller function (x, r)
        :type controller: casadi.DM
        :param time: total simulation time, defaults to 100 seconds
        :type time: float, optional
        """
        self.model = model
        self.dynamics = dynamics
        self.controller = controller
        self.total_sim_time = time  # seconds
        self.dt = self.model.dt
        self.estimation_in_the_loop = False
        self.using_trajectory_ref = False

        # Plotting definitions
        self.plt_window = float("inf")  # running plot window [s]/float("inf")
        plt.style.use('ggplot')

    def run(self, x0):
        """
        Run simulator with specified system dynamics and control function.
        """

        print("Running simulation....")
        sim_loop_length = int(self.total_sim_time/self.dt) + 1  # account for 0
        t = np.array([0])
        y_vec = np.array([x0]).T
        u_vec = np.array([[0, 0, 0, 0, 0, 0]]).T

        # Start figure
        if len(x0) == 13:
            fig, (ax1, ax2) = plt.subplots(2)
        else:
            print("Check your state dimensions.")
            exit()
        for i in range(sim_loop_length):
            # Translate data to ca.DM
            x = ca.DM(np.size(y_vec, 0), 1).full()
            x = np.array([y_vec[:, -1]]).T

            # Get control input and obtain next state
            if self.using_trajectory_ref:
                u = self.controller(x, i*self.dt)
            else:
                u = self.controller(x)
            x_next = self.dynamics(x, u)

            # Store data
            t = np.append(t, i*self.dt)
            y_vec = np.append(y_vec, np.array(x_next), axis=1)
            u_vec = np.append(u_vec, np.array(u), axis=1)

            # Get plot window values:
            if self.plt_window != float("inf"):
                l_wnd = 0 if int(i+1 - self.plt_window/self.dt) < 1 \
                          else int(i + 1 - self.plt_window/self.dt)
            else:
                l_wnd = 0

            if len(x0) == 13:
                ax1.clear()
                ax1.set_title("Astrobee Setpoint Test")
                ax1.plot(t[l_wnd:-1], y_vec[0, l_wnd:-1],
                         t[l_wnd:-1], y_vec[1, l_wnd:-1],
                         t[l_wnd:-1], y_vec[2, l_wnd:-1])
                ax1.legend(["X", "Y", "Z"])
                ax1.set_ylabel("Position [m]")
                ax1.grid()

                ax2.clear()
                ax2.plot(t[l_wnd:-1], u_vec[0, l_wnd:-1],
                         t[l_wnd:-1], u_vec[1, l_wnd:-1],
                         t[l_wnd:-1], u_vec[2, l_wnd:-1])
                ax2.set_xlabel("Time [s]")
                ax2.set_ylabel("Control [u]")
                ax2.grid()

            else:
                print("Please check your state dimensions.")
                exit()
            plt.pause(0.01)

        plt.show()
        return t, y_vec, u_vec

    def set_window(self, window):
        """
        Set the plot window length, in seconds.

        :param window: window length [s]
        :type window: float
        """
        self.plt_window = window

    def use_trajectory_control(self, value):
        """
        Helper function to set trajectory tracking controller.
        """
        self.using_trajectory_ref = value
