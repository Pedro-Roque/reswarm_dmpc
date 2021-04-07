from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
import time


class EmbeddedSimEnvironment(object):
    def __init__(self, model, dynamics, ctl_class, controller, 
                 time=100.0, plot=False):
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
        self.ctl_class = ctl_class
        self.controller = controller
        self.total_sim_time = time  # seconds
        self.dt = self.model.dt
        self.estimation_in_the_loop = False
        self.using_trajectory_ref = False
        self.plot = plot

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
        ref_vec = np.array([x0]).T
        u_vec = np.array([[0, 0, 0, 0, 0, 0]]).T
        slv_time = np.empty((1, 1))
        
        # Start figure
        if self.plot is True:
            if len(x0) == 13:
                fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5)
            else:
                print("Check your state dimensions.")
                exit()
            
        for i in range(sim_loop_length):
            # Translate data to ca.DM
            x = ca.DM(np.size(y_vec, 0), 1).full()
            x = np.array([y_vec[:, -1]]).T

            # Get control input and obtain next state
            u, ref = self.controller(x, i*self.dt)
            ref_vec = np.append(ref_vec, np.array([ref]).T, axis=1)
            slv_time = np.append(slv_time,
                                 self.ctl_class.get_last_solve_time())
            x_next = self.dynamics(x, u)

            # Store data
            t = np.append(t, i*self.dt)
            y_vec = np.append(y_vec, np.array(x_next), axis=1)
            u_vec = np.append(u_vec, np.array(u), axis=1)

            # Get plot window values:
            if self.plot is True:
                if self.plt_window != float("inf"):
                    l_wnd = 0 if int(i+1 - self.plt_window/self.dt) < 1 \
                            else int(i + 1 - self.plt_window/self.dt)
                else:
                    l_wnd = 0

                # Plot type
                ax1.clear()
                ax1.set_title("Astrobee Testing")
                ax1.plot(t[l_wnd:-1], y_vec[0, l_wnd:-1]-ref_vec[0, l_wnd:-1],
                        t[l_wnd:-1], y_vec[1, l_wnd:-1]-ref_vec[1, l_wnd:-1],
                        t[l_wnd:-1], y_vec[2, l_wnd:-1]-ref_vec[2, l_wnd:-1])
                ax1.legend(["e-x", "e-y", "e-z"])
                ax1.set_ylabel("Error [m]")
                ax1.grid()

                ax2.clear()
                ax2.plot(t[l_wnd:-1], y_vec[3, l_wnd:-1]-ref_vec[3, l_wnd:-1],
                        t[l_wnd:-1], y_vec[4, l_wnd:-1]-ref_vec[4, l_wnd:-1],
                        t[l_wnd:-1], y_vec[5, l_wnd:-1]-ref_vec[5, l_wnd:-1])
                ax2.legend(["e-vx", "e-vy", "e-vz"])
                ax2.set_ylabel("Error [m/s]")
                ax2.grid()

                # Plot attitude
                ax3.clear()
                qerr = np.array([[0]])
                q = y_vec[6:10, l_wnd:-1]
                q_des = ref_vec[6:10, l_wnd:-1]
                for j in range(len(q_des.T)):
                    qerr = np.concatenate((qerr, np.array([[1 - np.dot(q[:, j].T, q_des[:, j])**2]])), axis=1)
                qerr = qerr[:, 1:]  # Remove quaternion error initialization
                ax3.plot(t[l_wnd:-1], qerr.T)
                ax3.set_xlabel("Time [s]")
                ax3.set_ylabel("Attitude Error [norm-u]")
                ax3.grid()

                # Plot controls
                ax4.clear()
                ax4.plot(t[l_wnd:-1], u_vec[0, l_wnd:-1],
                        t[l_wnd:-1], u_vec[1, l_wnd:-1],
                        t[l_wnd:-1], u_vec[2, l_wnd:-1])
                ax4.set_xlabel("Time [s]")
                ax4.set_ylabel("Control F [N]")
                ax4.grid()

                ax5.clear()
                ax5.plot(t[l_wnd:-1], u_vec[3, l_wnd:-1],
                        t[l_wnd:-1], u_vec[4, l_wnd:-1],
                        t[l_wnd:-1], u_vec[5, l_wnd:-1])
                ax5.set_xlabel("Time [s]")
                ax5.set_ylabel("Control T [Nm]")
                ax5.grid()

        if self.plot is True:
            plt.show()

        return t, y_vec, u_vec, np.average(slv_time)

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
