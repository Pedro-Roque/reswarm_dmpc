#!/usr/bin/env python
import numpy as np
from numpy.core.numeric import Inf
import rospy

from reswarm_dmpc.util import *

import geometry_msgs.msg
import std_srvs.srv
import reswarm_dmpc.srv
import reswarm_dmpc.msg
import ff_msgs.msg
import ff_msgs.srv

DEBUG = False
OVERRIDE_TS = False


class UnitTestsMPC(object):
    """
    Class implementing the unit tests for Acado NMPC
    on the NASA Astrobees.
    """

    def __init__(self):
        """
        Initialize Distributed MPC Class
        """

        self.dt = 1
        self.rate = rospy.Rate(5)
        self.start = False
        self.state = np.zeros((13, 1))
        self.state[9] = 1
        self.rg = None
        self.t0 = 0.0
        self.f1_position = None
        self.twist = None
        self.pose = None

        # Collect parameters
        self.test_num = rospy.get_param("test_num")
        self.test_time = rospy.get_param("unit_test_time")
        self.test_targets = rospy.get_param("targets")

        # Print params:
        if DEBUG:
            print("RG Start: ", self.rg_start)
            print("Bearings: ", self.bearings)

        # Data timestamps and validity threshold
        self.ts_threshold = 1.0
        self.pose_ts = 0.0
        self.twist_ts = 0.0
        self.info_ts = 0.0

        # MPC data
        self.N = 10
        self.Nx = 13
        self.Nu = 6
        self.x_traj = None
        self.u_traj = None

        # Set publishers and subscribers
        self.set_services()
        self.set_subscribers_publishers()

        # Change onboard timeout
        new_timeout = ff_msgs.srv.SetFloatRequest()
        new_timeout.data = 1.5
        ans = self.pmc_timeout(new_timeout)
        if not ans.success:
            rospy.logerr("Couldn't change PMC timeout.")
            exit()
        else:
            rospy.loginfo("Timeout updated.")

        # Set weights and run main loop
        self.set_weights_iface()
        self.run()

        pass

    # ---------------------------------
    # BEGIN: Callbacks Section
    # ---------------------------------
    def pose_sub_cb(self, msg=geometry_msgs.msg.PoseStamped()):
        """
        Pose callback to update the agent's position and attitude.

        :param msg: estimated pose, defaults to geometry_msgs.msg.PoseStamped()
        :type msg: geometry_msgs.msg.PoseStamped
        """

        self.pose_ts = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
        self.pose = np.array([[msg.pose.position.x,
                               msg.pose.position.y,
                               msg.pose.position.z,
                               msg.pose.orientation.x,
                               msg.pose.orientation.y,
                               msg.pose.orientation.z,
                               msg.pose.orientation.w]]).T
        # Update state variable
        self.state[0:3] = self.pose[0:3]
        self.state[6:10] = self.pose[3:7]
        return

    def twist_sub_cb(self, msg=geometry_msgs.msg.TwistStamped()):
        """
        Twist callback to update the agent's lineear and angular velocities.

        :param msg: estimated velocities
        :type msg: geometry_msgs.msg.TwistStamped
        """

        self.twist_ts = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
        self.twist = np.array([[msg.twist.linear.x,
                                msg.twist.linear.y,
                                msg.twist.linear.z,
                                msg.twist.angular.x,
                                msg.twist.angular.y,
                                msg.twist.angular.z]]).T
        # Update state variable
        self.state[3:6] = self.twist[0:3]
        self.state[10:13] = self.twist[3:6]
        return

    def information_sub_cb(self, msg=reswarm_dmpc.msg.InformationStamped()):
        """
        Receive information from another Astrobee.

        :param msg: received data, defaults to reswarm_dmpc.msg.InformationStamped()
        :type msg: InformationStamped, optional
        """

        self.info_ts = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
        self.info_lv = np.array([[msg.leader_velocity.linear.x,
                                  msg.leader_velocity.linear.y,
                                  msg.leader_velocity.linear.z]]).T
        self.info_lt = np.array([[msg.leader_target.linear.x,
                                  msg.leader_target.linear.y,
                                  msg.leader_target.linear.z]]).T
        return

    def start_srv_callback(self, req=std_srvs.srv.SetBoolRequest()):
        """
        Service to start the operation of the autonomous control.

        :param req: request state
        :type req: std_srvs.srv.SetBoolRequest
        :return: success at starting
        :rtype: std_srvs.srv.SetBoolResponse
        """
        state = req.data
        ans = std_srvs.srv.SetBoolResponse()
        if state:
            ans.success = True
            ans.message = "Node started!"
            # Create reference:
            self.t0 = rospy.get_time()
            # Disable onboard controller
            obc = std_srvs.srv.SetBoolRequest()
            obc.data = False
            self.onboard_ctl(obc)
            self.start = True
        else:
            ans.success = True
            ans.message = "Node stopped!"
            # Enable onboard controller
            obc = std_srvs.srv.SetBoolRequest()
            obc.data = True
            self.onboard_ctl(obc)
            self.start = False

        return ans

    # ---------------------------------
    # END: Callbacks Section
    # ---------------------------------

    def set_subscribers_publishers(self):
        """
        Helper function to create all publishers and subscribers.
        """

        # Subscribers
        self.pose_sub = rospy.Subscriber("~pose_topic",
                                         geometry_msgs.msg.PoseStamped,
                                         self.pose_sub_cb)
        self.twist_sub = rospy.Subscriber("~twist_topic",
                                          geometry_msgs.msg.TwistStamped,
                                          self.twist_sub_cb)
        self.information_sub = rospy.Subscriber("~receive_information",
                                                reswarm_dmpc.msg.InformationStamped,
                                                self.information_sub_cb)

        # Publishers
        self.control_pub = rospy.Publisher("~control_topic",
                                           ff_msgs.msg.FamCommand,
                                           queue_size=1, latch=True)

        self.broadcast_pub = rospy.Publisher("~broadcast_information",
                                             reswarm_dmpc.msg.InformationStamped,
                                             queue_size=1)

        self.flight_mode_pub = rospy.Publisher("~flight_mode",
                                               ff_msgs.msg.FlightMode,
                                               queue_size=1)

        pass

    def set_services(self):
        """
        Helper function to create all services.
        """
        # Get control input service
        self.get_control = rospy.ServiceProxy("~get_control_srv",
                                              reswarm_dmpc.srv.GetControl)

        self.set_weights = rospy.ServiceProxy("~set_weights_srv",
                                              reswarm_dmpc.srv.SetWeights)

        # Astrobee control disable and timeout change service
        self.onboard_ctl = rospy.ServiceProxy("~onboard_ctl_enable_srv",
                                              std_srvs.srv.SetBool)
        self.pmc_timeout = rospy.ServiceProxy("~pmc_timeout_srv",
                                              ff_msgs.srv.SetFloat)

        # Start service
        self.start_service = rospy.Service("~start_srv", std_srvs.srv.SetBool,
                                           self.start_srv_callback)

        # Wait for services
        self.get_control.wait_for_service()
        self.set_weights.wait_for_service()
        self.onboard_ctl.wait_for_service()
        self.pmc_timeout.wait_for_service()
        pass

    def set_weights_iface(self):
        """
        Function to set the controller weights.
        """

        srv = reswarm_dmpc.srv.SetWeightsRequest()

        self.weights_size = 3 + 3 + 3 * 1 + 6
        self.weights_size_N = 3 + 3 + 3 * 1
        rpos_weights = np.ones((3 * 1,)) * 0.05
        verr_weights = np.ones((3,)) * 10
        att_weights = np.ones((3,)) * 10
        control_weights = np.array([5, 5, 5, 1, 1, 1]) * 10

        self.ln_weights = np.concatenate((rpos_weights,
                                          verr_weights,
                                          att_weights,
                                          control_weights), axis=0)

        self.V_weights = np.concatenate((rpos_weights,
                                         verr_weights,
                                         att_weights), axis=0) * 200

        srv.W = np.diag(self.ln_weights).ravel(order="F").tolist()
        srv.WN = np.diag(self.V_weights).ravel(order="F").tolist()

        if DEBUG:
            print("Weights matrices size:")
            print("W weight matrix: ", np.diag(self.ln_weights))
            print("WN weight matrix: ", np.diag(self.V_weights))
            print("W: ", len(srv.W))
            print("WN: ", len(srv.WN))

        ans = self.set_weights(srv)
        rospy.loginfo("Set weights success: " + str(ans.success)
                      + "\nSet weights message: " + str(ans.message))

    def check_data_validity(self):
        """
        Helper function to check the data validity.

        :return: True if data is valid, False otherwise
        :rtype: boolean
        """

        pos_val = False
        vel_val = False
        rel_val = False

        # Check state validity
        if rospy.get_time() - self.pose_ts < self.ts_threshold or OVERRIDE_TS:
            pos_val = True
        if rospy.get_time() - self.twist_ts < self.ts_threshold or OVERRIDE_TS:
            vel_val = True
        if rospy.get_time() - self.info_ts < self.ts_threshold or OVERRIDE_TS:
            rel_val = True

        # Check weights validity

        if pos_val is False or vel_val is False or rel_val is False:
            rospy.logwarn("Skipping control. Validity flags:\nPos: "
                          + str(pos_val) + "; Vel: " + str(vel_val) + "; Rel: " + str(rel_val))

        return pos_val and vel_val and rel_val

    def get_setpoint(self, t):
        """
        Collects setpoints from the yaml files for attitude and position unit tests.

        :param t: test time
        :type t: float
        :return: target setpoint
        :rtype: np.ndarray
        """

        if self.test_num == 1:
            # Translation Unit Tests:
            if t < self.test_time:
                xd = np.array([self.test_targets['t']['t1']]).reshape((self.Nx, 1))
            elif t < 2 * self.test_time:
                xd = np.array([self.test_targets['t']['t2']]).reshape((self.Nx, 1))
            elif t < 3 * self.test_time:
                xd = np.array([self.test_targets['t']['t3']]).reshape((self.Nx, 1))
            elif t < 4 * self.test_time:
                xd = np.array([self.test_targets['t']['t4']]).reshape((self.Nx, 1))
            elif t < 5 * self.test_time:
                xd = np.array([self.test_targets['t']['t5']]).reshape((self.Nx, 1))
            elif t < 6 * self.test_time:
                xd = np.array([self.test_targets['t']['t6']]).reshape((self.Nx, 1))

        elif self.test_num == 2:
            # Translation Unit Tests:
            if t < self.test_time:
                xd = np.array([self.test_targets['q']['t1']]).reshape((self.Nx, 1))
            elif t < 2 * self.test_time:
                xd = np.array([self.test_targets['q']['t2']]).reshape((self.Nx, 1))
            elif t < 3 * self.test_time:
                xd = np.array([self.test_targets['q']['t3']]).reshape((self.Nx, 1))
            elif t < 4 * self.test_time:
                xd = np.array([self.test_targets['q']['t4']]).reshape((self.Nx, 1))
            elif t < 5 * self.test_time:
                xd = np.array([self.test_targets['q']['t5']]).reshape((self.Nx, 1))
            elif t < 6 * self.test_time:
                xd = np.array([self.test_targets['q']['t6']]).reshape((self.Nx, 1))
        else:
            xd = np.array([self.test_targets['t']['init']]).reshape((self.Nx, 1))

        return xd

    def prepare_request(self, t):
        """
        Helper function to prepare the control request.

        :return: control service request
        :rtype: reswarm_dmpc.srv.GetControlRequest
        """

        # Get trajectory
        val = False
        val = self.check_data_validity()
        rospy.loginfo("Validity: " + str(val))
        if val is False:
            return val, 0

        # Valid data, so we proceed
        setpoint = self.get_setpoint(t)
        self.x0 = self.state.reshape((13, 1))
        if self.x_traj is None:
            self.x_traj = np.repeat(self.x0, self.N + 1, axis=1)

        if self.u_traj is None:
            self.u_traj = np.repeat(np.zeros((1, 6)), self.N, axis=0)

        self.online_data = np.repeat(setpoint, self.N + 1, axis=1)

        if DEBUG:
            print("Target Velocity: ", self.target_vel.shape)
            print("State dims: ", self.state.shape)
            print("X0 data: ", self.x0.ravel(order="F").tolist())
            print("X data: ", self.x_traj.ravel(order="F").tolist())
            print("U data: ", self.u_traj.ravel(order="F").tolist())
            print("OD data: ", self.online_data.ravel(order="F").tolist())

        srv = reswarm_dmpc.srv.GetControlRequest()
        srv.initial_state = self.x0.ravel(order="F").tolist()
        srv.predicted_state = self.x_traj.ravel(order="F").tolist()
        srv.predicted_input = self.u_traj.ravel(order="F").tolist()
        srv.online_data = self.online_data.ravel(order="F").tolist()

        if DEBUG:
            print("Data shapes on sending:")
            print("X0: ", len(srv.initial_state))
            print("X: ", len(srv.predicted_state))
            print("U: ", len(srv.predicted_input))
            print("OD: ", len(srv.online_data))

        val = True
        return val, srv

    def create_control_message(self):
        """
        Helper function to create the control message to be published

        :return: control input to vehicle
        :rtype: geometry_msgs.msg.WrenchStamped()
        """

        # Create message
        u = ff_msgs.msg.FamCommand()

        # Fill header
        u.header.frame_id = 'body'
        u.header.stamp = rospy.Time.now()

        # Fill force / torque messages
        u.wrench.force.x = self.u_traj[0]
        u.wrench.force.y = self.u_traj[1]
        u.wrench.force.z = self.u_traj[2]
        u.wrench.torque.x = self.u_traj[3]
        u.wrench.torque.y = self.u_traj[4]
        u.wrench.torque.z = self.u_traj[5]

        # Set control mode and status
        u.status = 3
        u.control_mode = 2

        return u

    def create_broadcast_message(self):
        """
        Helper function to create the broadcasted information vector.

        :return: the message to be broadcasted
        :rtype: reswarm_dmpc.msg.InformationStamped
        """

        # Create message
        v = reswarm_dmpc.msg.InformationStamped()

        # Fill header
        v.header.frame_id = 'inertial'
        v.header.stamp = rospy.Time.now()

        # Fill information fields
        v.leader_velocity.linear.x = 1
        v.leader_velocity.linear.y = 2
        v.leader_velocity.linear.z = 3

        v.leader_target.linear.x = 4
        v.leader_target.linear.y = 5
        v.leader_target.linear.z = 6

        return v

    def create_flight_mode_message(self):
        """
        Helper function to create the flight mode message.
        """

        fm = ff_msgs.msg.FlightMode()

        fm.name = "difficult"

        fm.collision_radius = 0.25
        fm.control_enabled = False

        fm.att_ki = Vec(0.002, 0.002, 0.002)
        fm.att_kp = Vec(4.0, 4.0, 4.0)
        fm.omega_kd = Vec(3.2, 3.2, 3.2)

        fm.pos_kp = Vec(.6, .6, .6)
        fm.pos_ki = Vec(0.0001, 0.0001, 0.0001)
        fm.vel_kd = Vec(1.2, 1.2, 1.2)

        fm.speed = 3

        fm.tolerance_pos = 0.2
        fm.tolerance_vel = 0
        fm.tolerance_att = 0.3490
        fm.tolerance_omega = 0
        fm.tolerance_time = 1.0

        fm.hard_limit_accel = 0.0200
        fm.hard_limit_omega = 0.5236
        fm.hard_limit_alpha = 0.2500
        fm.hard_limit_vel = 0.4000

        return fm

    def reset_control_request(self, srv):
        """
        Reset the solver internal variables.

        :param srv: solver request
        :type srv: reswarm_dmpc.srv.GetControlRequest
        :return: resetted solver request
        :rtype: reswarm_dmpc.srv.GetControlRequest
        """

        self.x_traj = np.repeat(self.x0, self.N + 1, axis=1)
        self.u_traj = np.repeat(np.zeros((1, 6)), self.N, axis=0)

        srv.initial_state = self.x0.ravel(order="F").tolist()
        srv.predicted_state = self.x_traj.ravel(order="F").tolist()
        srv.predicted_input = self.u_traj.ravel(order="F").tolist()
        srv.online_data = self.online_data.ravel(order="F").tolist()

        return srv

    def propagate_predicted_state(self, ans):
        """
        Fills the predicted_state request once data is available.

        :param ans: [description]
        :type ans: [type]
        :return: [description]
        :rtype: [type]
        """
        predicted_state = np.asarray(ans.predicted_state).reshape(((self.N + 1), self.Nx)).T
        for i in range(self.N + 1):
            q = predicted_state[6:10, i]
            predicted_state[6:10, i] = q / np.linalg.norm(q)
        return predicted_state.ravel(order="F").tolist()

    def run(self):
        """
        Main operation loop.
        """

        while not rospy.is_shutdown():
            # Broadcast static info
            v = self.create_broadcast_message()
            self.broadcast_pub.publish(v)

            # Only do something when started
            if self.start is False:
                self.rate.sleep()
                rospy.loginfo("Sleeping...")
                continue

            rospy.loginfo("Looping!")
            # Use self.pose, self.twist to generate a control input
            t = rospy.get_time() - self.t0
            val, req = self.prepare_request(t)
            if val is False:
                self.rate.sleep()
                continue

            # Start the RTI Loop
            tin = rospy.get_time()
            temp_kkt = Inf
            tries = 0
            while (temp_kkt > 100 or np.isnan(temp_kkt)) and tries < 10:
                rospy.loginfo("[RTI Loop] Trial: " + str(tries))
                ans = self.get_control(req)
                temp_kkt = ans.kkt_value
                if np.isnan(temp_kkt) or tries == 9:
                    rospy.logwarn("NaN detected on solver output.")
                    req = self.reset_control_request(req)
                else:
                    req.predicted_state = self.propagate_predicted_state(ans)
                    req.predicted_input = np.asarray(ans.predicted_input).reshape((self.Nu * self.N, 1)).ravel(order="F").tolist()
                rospy.loginfo("[RTI Loop] Solver kkT: " + str(ans.kkt_value))
                tries += 1
            tout = rospy.get_time() - tin
            rospy.loginfo("Time for control: " + str(tout))

            # Update the trajectories for the next iteration
            self.x_traj = np.asarray(ans.predicted_state).reshape((self.Nx * (self.N + 1), 1))
            self.u_traj = np.asarray(ans.predicted_input).reshape((self.Nu * self.N, 1))
            if DEBUG:
                print("X traj: ", self.x_traj.ravel(order="F").tolist())
                print("U traj: ", self.u_traj.ravel(order="F").tolist())
            rospy.loginfo("Solver status: " + str(ans.status))
            rospy.loginfo("Solver cpuTime: " + str(ans.solution_time))
            rospy.loginfo("Solver Cost: " + str(ans.objective_value))

            # Create control input message
            u = self.create_control_message()
            v = self.create_broadcast_message()
            fm = self.create_flight_mode_message()

            # Publish control
            self.control_pub.publish(u)
            self.broadcast_pub.publish(v)
            self.flight_mode_pub.publish(fm)
            self.rate.sleep()
    pass


if __name__ == "__main__":
    rospy.init_node("leader_dmpc")
    dmpc = UnitTestsMPC()
    rospy.spin()
    pass
