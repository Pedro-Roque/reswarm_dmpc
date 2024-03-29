import numpy as np


class SinusoidalReference(object):
    """
    Sinusoidal trajectory generator

    """
    def __init__(self, dt, x0,
                 f=0.035, A=0.06, time_span=120.0):
        """
        Initialize the Sinusoidal reference class.

        :param dt: time discretization (time interval between samples)
        :type dt: float
        :param x0: first position setpoint of the trajectory
        :type x0: np.array (3,)
        :param f: velocity sinusoidal frequency, defaults to 0.01
        :type f: float, optional
        :param A: velocity sinusoidal amplitude, defaults to 0.01
        :type A: float, optional
        :param time_span: total trajectory time-span, defaults to 20
        :type time_span: float, optional
        """
        # Set internal variables
        self.f = f
        self.A = A
        self.time_span = time_span
        self.dt = dt

        assert (time_span / dt) % 1 == 0, "Time span is not dividable by " + \
                                          "the selected sampling time"

        # Initialize trajectory starting points
        self.x0 = x0[0]
        self.y0 = x0[1]
        self.z0 = x0[2]

        # Initialize trajectory variable
        self.full_trajectory = None

    def get_trajectory(self, t, points):
        """
        Get trajectory after creation

        :param t0: [description]
        :type t0: [type]
        :param points: [description]
        :type points: [type]
        :return: [description]
        :rtype: [type]
        """
        assert self.full_trajectory is not None, "Create the trajectory first."
        assert t >= 0, "Time instance has to >= 0."
        x_sp = self.full_trajectory[:, int(t / self.dt):
                                    (int(t / self.dt) + points)]
        return x_sp.ravel(order='F')

    def get_vel_trajectory(self, t, points):
        """
        Helper function to extract velocity from a given trajectory

        :param t: [description]
        :type t: [type]
        :param points: [description]
        :type points: [type]
        :return: [description]
        :rtype: [type]
        """

        assert self.full_trajectory is not None, "Create the trajectory first."
        assert t >= 0, "Time instance has to >= 0."
        xr = self.full_trajectory[:, int(t / self.dt):
                                  (int(t / self.dt) + points)]
        # vel = xr[3:6,:].reshape(3*points,1)

        return xr[3:6, :]

    def get_vel_trajectory_at_t(self, t, points, vel_prf):
        """
        Create trajectory starting at time t with the specified
        number of points

        :param t: starting trajectory time
        :type t: float
        :param points: number of traejctory points
        :type points: int
        :return: velocity trajectory for the specified interval
        :rtype: np.ndarray
        """

        t = np.linspace(t, (points - 1) * self.dt + t, points)
        vx = vel_prf[0] * np.cos(2 * np.pi * self.f * t)
        vy = vel_prf[1] * np.ones(points)
        vz = vel_prf[2] * np.sin(2 * np.pi * self.f * t)

        # Once we have a velocity profile, we can create the
        # position references
        x = np.array([self.x0])
        y = np.array([self.y0])
        z = np.array([self.z0])

        for i in range(points - 1):
            x = np.append(x, x[-1] + vx[i] * self.dt)
            y = np.append(y, y[-1] + vy[i] * self.dt)
            z = np.append(z, z[-1] + vz[i] * self.dt)
        # Create trajectory matrix
        x_sp = np.array([x])
        x_sp = np.append(x_sp, [y], axis=0)
        x_sp = np.append(x_sp, [z], axis=0)
        x_sp = np.append(x_sp, [vx], axis=0)
        x_sp = np.append(x_sp, [vy], axis=0)
        x_sp = np.append(x_sp, [vz], axis=0)
        x_sp = np.append(x_sp, np.zeros((2, points)), axis=0)
        x_sp = np.append(x_sp, -0.707 * np.ones((1, points)), axis=0)
        x_sp = np.append(x_sp, 0.707 * np.ones((1, points)), axis=0)
        x_sp = np.append(x_sp, np.zeros((3, points)), axis=0)
        self.full_trajectory = np.array(x_sp)

        return self.full_trajectory[3:6, :]

    def get_full_trajectory_at_t(self, t0, points, vel_prf, x0=None):
        """
        Create trajectory starting at time t with the specified
        number of points

        :param t: starting trajectory time
        :type t: float
        :param points: number of traejctory points
        :type points: int
        :return: velocity trajectory for the specified interval
        :rtype: np.ndarray
        """

        t = np.linspace(t0, (points - 1) * self.dt + t0, points)
        vx = vel_prf[0] * np.cos(2 * np.pi * self.f * t)
        vy = vel_prf[1] * np.ones(points)
        vz = vel_prf[2] * np.sin(2 * np.pi * self.f * t)

        # Once we have a velocity profile, we can create the
        # position references
        x = np.array([self.x0])
        y = np.array([self.y0]) + vel_prf[1] * t0
        z = np.array([self.z0])

        for i in range(points - 1):
            x = np.append(x, x[-1] + vx[i] * self.dt)
            y = np.append(y, y[-1] + vy[i] * self.dt)
            z = np.append(z, z[-1] + vz[i] * self.dt)

        # Create trajectory matrix
        x_sp = np.array([x])
        x_sp = np.append(x_sp, [y], axis=0)
        x_sp = np.append(x_sp, [z], axis=0)
        x_sp = np.append(x_sp, [vx], axis=0)
        x_sp = np.append(x_sp, [vy], axis=0)
        x_sp = np.append(x_sp, [vz], axis=0)
        x_sp = np.append(x_sp, np.zeros((2, points)), axis=0)
        x_sp = np.append(x_sp, -0.707 * np.ones((1, points)), axis=0)
        x_sp = np.append(x_sp, 0.707 * np.ones((1, points)), axis=0)
        x_sp = np.append(x_sp, np.zeros((3, points)), axis=0)
        self.full_trajectory = np.array(x_sp)

        return self.full_trajectory[0:6, :]

    def create_trajectory(self):
        """
        Helper method to create the trajectory.
        """
        # Generate whole trajectory for 20 seconds
        # Trajectory params

        gen_points = int(self.time_span / self.dt)
        t = np.linspace(0, (gen_points - 1) * self.dt, gen_points)
        vx = self.A * np.cos(2 * np.pi * self.f * t)
        vy = self.A * np.sin(2 * np.pi * self.f * t)
        vz = 0.005 * np.ones(gen_points)

        # Once we have a velocity profile, we can create the
        # position references
        x = np.array([self.x0])
        y = np.array([self.y0])
        z = np.array([self.z0])

        for i in range(gen_points - 1):
            x = np.append(x, x[-1] + vx[i] * self.dt)
            y = np.append(y, y[-1] + vy[i] * self.dt)
            z = np.append(z, z[-1] + vz[i] * self.dt)
        # Create trajectory matrix
        x_sp = np.array([x])
        x_sp = np.append(x_sp, [y], axis=0)
        x_sp = np.append(x_sp, [z], axis=0)
        x_sp = np.append(x_sp, [vx], axis=0)
        x_sp = np.append(x_sp, [vy], axis=0)
        x_sp = np.append(x_sp, [vz], axis=0)
        x_sp = np.append(x_sp, np.zeros((3, gen_points)), axis=0)
        x_sp = np.append(x_sp, np.ones((1, gen_points)), axis=0)
        x_sp = np.append(x_sp, np.zeros((3, gen_points)), axis=0)
        self.full_trajectory = np.array(x_sp)


class ForwardPropagation(object):

    def __init__(self, dt):
        """
        Forward propagation class of a given velocity and initial pose.

        :param dt: sample time
        :type dt: float
        """

        self.dt = dt
        self.full_trajectory = None
        pass

    def forward_propagate(self, points, p0, v, q0=None, w=None):
        """
        Forward propagation of p0 with velocity v

        :param points: number of points for the generated propagation
        :type points: int
        :param p0: initial position
        :type p0: np.ndarray
        :param v: velocity used in the propagation
        :type v: np.ndarray
        :param q0: attitude quaterion if attitude propagation is needed, defaults to None
        :type q0: np.ndarray, optional
        :param w: angular velocity if attitude propagation is needed, defaults to None
        :type w: np.ndarray, optional
        :raises ValueError: if q0 or w is not None, but the other is
        :return: propagated points
        :rtype: np.ndarray
        """
        # Check if q0 and w are proper
        if q0 is None and w is not None or \
           q0 is not None and w is None:
            raise ValueError("Q0 or W is None in attitude propagation.")

        if q0 is None:
            # Position only propagation
            x = np.array([p0[0]])
            y = np.array([p0[1]])
            z = np.array([p0[2]])

            # Velocity propagation
            v = np.repeat(v, points, axis=1)
            vx = v[0, :]
            vy = v[1, :]
            vz = v[2, :]

            # Propagate the position
            for i in range(points - 1):
                x = np.append(x, x[-1] + vx[i] * self.dt)
                y = np.append(y, y[-1] + vy[i] * self.dt)
                z = np.append(z, z[-1] + vz[i] * self.dt)

            # Create trajectory matrix
            x_sp = np.array([x])
            x_sp = np.append(x_sp, [y], axis=0)
            x_sp = np.append(x_sp, [z], axis=0)
            x_sp = np.append(x_sp, [vx], axis=0)
            x_sp = np.append(x_sp, [vy], axis=0)
            x_sp = np.append(x_sp, [vz], axis=0)
            x_sp = np.append(x_sp, np.zeros((3, points)), axis=0)
            x_sp = np.append(x_sp, np.ones((1, points)), axis=0)
            x_sp = np.append(x_sp, np.zeros((3, points)), axis=0)
            self.full_trajectory = np.array(x_sp)

            return self.full_trajectory.ravel(order='F')
