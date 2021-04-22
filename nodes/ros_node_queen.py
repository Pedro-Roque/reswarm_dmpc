#!/usr/bin/env python
import numpy as np
import scipy
import rospy

from reswarm_dmpc.models.astrobee import Astrobee
from reswarm_dmpc.controllers.formation.mpc_trajectory import MPC
from reswarm_dmpc.simulation import EmbeddedSimEnvironment

import geometry_msgs.msg
import std_srvs.srv


class DistributedMPC(object):
    """
    Class implementing the distributed formation control MPC
    on the NASA Astrobees.
    """

    def __init__(self):

        self.rate = rospy.Rate(50)

        pass

    # ---------------------------------
    # BEGIN: Callbacks Section
    # ---------------------------------
    def pose_sub_cb(self, msg=geometry_msgs.msg.PoseStamped()):
        self.pose = msg
        return

    def twist_sub_cb(self, msg=geometry_msgs.msg.TwistStamped()):
        self.twist = msg
        return

    def start_srv_callback(self, req=std_srvs.srv.SetBoolRequest()):
        self.start = req.data

        ans = std_srvs.srv.SetBoolResponse()
        ans.success = True
        ans.message = "Node started!"

        return ans

    # ---------------------------------
    # END: Callbacks Section
    # ---------------------------------

    def set_subscribers_publishers(self):

        # Subscribers
        self.pose_sub = rospy.Subscriber("~pose_topic",
                                         geometry_msgs.msg.PoseStamped,
                                         self.pose_sub_cb)
        self.twist_sub = rospy.Subscriber("~twist_topic",
                                          geometry_msgs.msg.TwistStamped,
                                          self.twist_sub_cb)

        # Publishers
        self.control_pub = rospy.Publisher("~control_topic",
                                           geometry_msgs.msg.WrenchStamped,
                                           queue_size=1)
        pass

    def set_services(self):

        # Set Service Servers
        self.start_srv = rospy.Service("~start_address", std_srvs.srv.SetBool,
                                       self.start_srv_callback)

        # Set Service Clients
        self.update_viz = rospy.ServiceProxy("~update_viz_address",
                                             std_srvs.srv.SetBool)
        pass

    def run(self):

        while not rospy.is_shutdown():

            # Only do something when started
            if self.start is False:
                self.rate.sleep()
                continue

            # Use self.pose, self.twist to generate a control input
            # u = self.control(self.pose, self.twist)

            # u needs to be geometry_msgs.msg.Wrench
            # self.control_pub.publish(u)

            self.rate.sleep()
    pass


if __name__ == "__main__":
    dmpc = DistributedMPC()
    dmpc.run()
    rospy.spin()
    pass
