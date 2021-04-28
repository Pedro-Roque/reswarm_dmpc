#ifndef LEADER_CTL_H
#define LEADER_CTL_H

#include <ros/ros.h>

#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Wrench.h>

#include <reswarm_dmpc/GetControl.h>
#include <reswarm_dmpc/SetWeights.h>

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

class LeaderMPC{
    private:
        // Variables
        ros::NodeHandle nh_;
        ros::ServiceServer get_control_;
        ros::ServiceServer set_weights_;

        // Functions
        bool GetControlCallback(reswarm_dmpc::GetControl::Request &req, reswarm_dmpc::GetControl::Response &res);
        bool SetWeightsCallback(reswarm_dmpc::SetWeights::Request &req, reswarm_dmpc::SetWeights::Response &res);
    public:
        // Constructor / Destructor
        LeaderMPC(const ros::NodeHandle &nh);
        virtual ~LeaderMPC();

};

#endif