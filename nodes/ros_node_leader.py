#!/usr/bin/env python
import numpy as np
import scipy
import rospy

from reswarm_dmpc.reference_generation.sinusoidal import SinusoidalReference
from reswarm_dmpc.util import *

import geometry_msgs.msg
import std_srvs.srv
import reswarm_dmpc.srv
import reswarm_dmpc.msg

DEBUG = False
OVERRIDE_TS = True

class DistributedMPC(object):
    """
    Class implementing the distributed formation control MPC
    on the NASA Astrobees.
    """

    def __init__(self):
        """
        Initialize Distributed MPC Class
        """

        self.dt = 1
        self.rate = rospy.Rate(1)
        self.start = False
        self.state = np.zeros((13,1))
        self.state[9] = 1
        self.rg = None
        self.t0 = 0.0
        self.f1_position = None
        self.twist = None
        self.pose = None

        # Collect parameters
        rg_start = rospy.get_param("traj_start")
        self.rg_start = np.array([rg_start]).reshape((3,1))
        bearings = rospy.get_param("bearings")
        self.bearings = np.array([bearings['f1']]).reshape((3,1))

        # Print params:
        if DEBUG:
            print("RG Start: ", self.rg_start)
            print("Bearings: ", self.bearings)

        # Data timestamps and validity threshold
        self.ts_threshold = 1.0
        self.pose_ts = 0.0
        self.twist_ts = 0.0
        self.f1_position_ts = 0.0

        # MPC data
        self.N = 10
        self.Nx = 3+3+4+3+3*1
        self.Nu = 6
        self.x_traj = None
        self.u_traj = None
        self.weights_size = 3+3+3*1+6
        self.weights_size_N = 3+3+3*1
        rpos_weights = np.ones((3*1,))*0.05
        verr_weights = np.ones((3,))*10
        att_weights = np.ones((3,))*10
        control_weights = np.array([5, 5, 5, 1, 1, 1])*10

        self.ln_weights = np.concatenate((rpos_weights,
                                          verr_weights,
                                          att_weights,
                                          control_weights), axis = 0)

        self.V_weights = np.concatenate((rpos_weights,
                                         verr_weights,
                                         att_weights), axis = 0)*200

        # Set publishers and subscribers
        self.set_services()
        self.set_subscribers_publishers()
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

        self.pose_ts = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs
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

        self.twist_ts = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs
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

    def f1_pose_sub_cb(self, msg=geometry_msgs.msg.PoseStamped()):
        """
        Follower 1 position callback (for further simulation of a rel. pos. sensor)

        :param msg: follower 1 position
        :type msg: geometry_msgs.msg.PoseStamped
        """

        self.f1_position_ts = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs
        self.f1_position = np.array([[msg.pose.position.x, 
                                      msg.pose.position.y, 
                                      msg.pose.position.z]]).T
        return

    def start_srv_callback(self, req=std_srvs.srv.SetBoolRequest()):
        """
        Service to start the operation of the autonomous control.

        :param req: request state
        :type req: std_srvs.srv.SetBoolRequest
        :return: success at starting
        :rtype: std_srvs.srv.SetBoolResponse
        """

        self.start = req.data

        ans = std_srvs.srv.SetBoolResponse()
        ans.success = True
        ans.message = "Node started!"

        # Check for valid pose and twist
        self.rg = SinusoidalReference(self.dt, self.rg_start, A=0.1, time_span=45)
        self.t0 = rospy.get_time()

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
        self.f1_sub = rospy.Subscriber("~neigh_pose",
                                       geometry_msgs.msg.PoseStamped,
                                       self.f1_pose_sub_cb)

        # Publishers
        self.control_pub = rospy.Publisher("~control_topic",
                                           geometry_msgs.msg.WrenchStamped,
                                           queue_size=1)
        
        self.broadcast_pub = rospy.Publisher("~broadcast_information", 
                                             reswarm_dmpc.msg.InformationStamped,
                                             queue_size=1)

        pass

    def set_services(self):
        """
        Helper function to create all services.
        """
        # Get control input service
        self.get_control = rospy.ServiceProxy("/leader/get_control", 
                                              reswarm_dmpc.srv.GetControl)

        self.set_weights = rospy.ServiceProxy("/leader/set_weights", 
                                              reswarm_dmpc.srv.SetWeights)

        self.start_service = rospy.Service("~start", std_srvs.srv.SetBool, self.start_srv_callback)

        # Wait for services
        self.get_control.wait_for_service()
        self.set_weights.wait_for_service()
        pass

    def set_weights_iface(self):
        """
        Function to set the controller weights.
        """

        srv = reswarm_dmpc.srv.SetWeightsRequest()
        srv.W = np.diag(self.ln_weights).ravel(order="F").tolist()
        srv.WN = np.diag(self.V_weights).ravel(order="F").tolist()

        if DEBUG:
            print("Weights matrices size:")
            print("W weight matrix: ", np.diag(self.ln_weights))
            print("WN weight matrix: ", np.diag(self.V_weights))
            print("W: ", len(srv.W))
            print("WN: ", len(srv.WN))

        ans = self.set_weights(srv)
        rospy.loginfo("Set weights success: "+str(ans.success)+\
                      "\nSet weights message: "+str(ans.message))

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
        if rospy.get_time() - self.f1_position_ts < self.ts_threshold or OVERRIDE_TS:
            rel_val = True

        # Check weights validity
        
        if pos_val is False or vel_val is False or rel_val is False:
            rospy.logwarn("Skipping control. Validity flags:\nPos: "\
                           +str(pos_val)+"; Vel: "+str(vel_val)+"; Rel: "+str(rel_val))
        
        return pos_val and vel_val and rel_val

    def get_relative_pos(self):
        """
        Helper function to return the relative position of the neighbours.

        :return: relative position to follower 1
        :rtype: np.ndarray
        """

        rmat = r_mat_np(self.state[6:10]).T
        rel_pos = np.dot(rmat, self.state[0:3] - self.f1_position)
        return rel_pos

    def prepare_request(self, t):
        """
        Helper function to prepare the control request.

        :return: control service request
        :rtype: reswarm_dmpc.srv.GetControlRequest
        """

        # Get trajectory
        val = False
        val = self.check_data_validity()
        rospy.loginfo("Validity: "+ str(val))
        if val is False:
            return val, 0
        
        # Valid data, so we proceed
        self.target_vel = self.rg.get_vel_trajectory_at_t(t, self.N+1)
        rel_pos = self.get_relative_pos()
        x0 = np.concatenate((self.state, rel_pos), axis=0).reshape((13+3,1))
        if self.x_traj is None:
            self.x_traj = np.repeat(x0, self.N+1, axis=1)

        if self.u_traj is None:
            self.u_traj = np.repeat(np.zeros((1,6)), self.N, axis=0)

        online_data = np.repeat(self.bearings, self.N+1, axis=1)
        if DEBUG:
            print("Bearings repeated shape: ", online_data.shape)
            print("Velocity shape: ", self.target_vel.shape)
        
        online_data = np.concatenate((online_data,self.target_vel), axis=0)

        if DEBUG:
            print("Target Velocity: ", self.target_vel.shape)
            print("Neighour position: ", rel_pos.shape)
            print("State dims: ", self.state.shape)
            print("rel_pos dims: ", rel_pos.shape)
            print("Bearings dims: ", self.bearings.shape)
            print("X0 data: ", x0.ravel(order="F").tolist(), " - X0 shape: ", x0.shape)
            print("X data: ", self.x_traj.ravel(order="F").tolist()) # .ravel(order="C")
            print("U data: ", self.u_traj.ravel(order="F").tolist()) # .ravel(order="C")
            print("OD data: ", online_data.ravel(order="F").tolist())

        srv = reswarm_dmpc.srv.GetControlRequest()
        srv.initial_state = x0.ravel(order="F").tolist()
        srv.predicted_state = self.x_traj.ravel(order="F").tolist()
        srv.predicted_input = self.u_traj.ravel(order="F").tolist()
        srv.online_data = online_data.ravel(order="F").tolist()  # TODO(@Pedro-Roque): remove zD param

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
        u = geometry_msgs.msg.WrenchStamped()

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
        v.leader_velocity.linear.x = self.state[3]
        v.leader_velocity.linear.y = self.state[4]
        v.leader_velocity.linear.z = self.state[5]
        
        v.leader_target.linear.x = self.target_vel[0,0]
        v.leader_target.linear.y = self.target_vel[1,0]
        v.leader_target.linear.z = self.target_vel[2,0]

        return v

    def run(self):
        """
        Main operation loop.
        """
        
        while not rospy.is_shutdown():

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
                continue

            # For a valid request, proceed
            tin = rospy.get_time()
            ans = self.get_control(req)  # TODO(@Pedro-Roque): we might want to put this in a loop for kkt < 1
            tout = rospy.get_time() - tin
            rospy.loginfo("Time for control: "+str(tout))

            self.x_traj = np.asarray(ans.predicted_state).reshape((self.Nx*(self.N+1),1))
            self.u_traj = np.asarray(ans.predicted_input).reshape((self.Nu*self.N, 1))
            if DEBUG:
                print("X traj: ", self.x_traj.ravel(order="F").tolist())
                print("U traj: ", self.u_traj.ravel(order="F").tolist())
            rospy.loginfo("Solver status: "+str(ans.status))
            rospy.loginfo("Solver kkT: "+str(ans.kkt_value))
            rospy.loginfo("Solver cpuTime: "+str(ans.solution_time))

            # Create control input message
            u = self.create_control_message()
            v = self.create_broadcast_message()

            # Publish control
            self.control_pub.publish(u)
            self.broadcast_pub.publish(v)


            self.rate.sleep()
    pass


if __name__ == "__main__":
    rospy.init_node("leader_dmpc")
    dmpc = DistributedMPC()
    rospy.spin()
    pass
