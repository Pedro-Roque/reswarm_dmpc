#!/usr/bin/env python
import numpy as np
from numpy.core.numeric import Inf
import rospy

from reswarm_dmpc.util import *

import ff_msgs.msg
import ff_msgs.srv


class SimpleControlExample(object):
    """
    Class implementing a simple controller example that
    does absolutely nothing on the NASA Astrobees.
    """

    def __init__(self):
        """
        Initialize controller class
        """

        # Initialize basic parameters
        self.rate = rospy.Rate(100)

        # Data timestamps and validity threshold
        self.control_ts = -Inf
        self.low_rate_ctl_msg = ff_msgs.msg.FamCommand()

        # Set publishers and subscribers
        self.set_subscribers_publishers()

        self.run()

        pass

    # ---------------------------------
    # BEGIN: Callbacks Section
    # ---------------------------------
    def control_sub_cb(self, msg=ff_msgs.msg.FamCommand()):
        """
        Low rate control input callback
        """
        self.control_ts = rospy.get_time()
        self.low_rate_ctl_msg = msg
        return

    # ---------------------------------
    # END: Callbacks Section
    # ---------------------------------

    def set_subscribers_publishers(self):
        """
        Helper function to create all publishers and subscribers.
        """

        # Subscribers
        self.control_sub = rospy.Subscriber("~low_rate_ctl_topic",
                                            ff_msgs.msg.FamCommand,
                                            self.control_sub_cb)

        # Publishers
        self.control_pub = rospy.Publisher("~high_rate_ctl_topic",
                                           ff_msgs.msg.FamCommand,
                                           queue_size=1, latch=True)

        self.flight_mode_pub = rospy.Publisher("~flight_mode",
                                               ff_msgs.msg.FlightMode,
                                               queue_size=1)

        pass

    def create_control_message(self):
        """
        Helper function to create the control message to be published

        :return: control input to vehicle
        :rtype: ff_msgs.msg.FamCommand()
        """

        return self.low_rate_ctl_msg

    def create_flight_mode_message(self):
        """
        Helper function to create the flight mode message.

        Here we use the flight-mode difficult to have full access to the
        actuation capabilities of the Astrobee.
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

    def run(self):
        """
        Main operation loop.
        """

        while not rospy.is_shutdown():

            # Use self.pose, self.twist to generate a control input
            t = rospy.get_time()

            if t - self.control_ts < 1.0:
                # Create control input message
                u = self.create_control_message()
                fm = self.create_flight_mode_message()

                # Publish control
                self.control_pub.publish(u)
                self.flight_mode_pub.publish(fm)
            self.rate.sleep()
    pass


if __name__ == "__main__":
    rospy.init_node("node_template")
    dmpc = SimpleControlExample()
    rospy.spin()
    pass
