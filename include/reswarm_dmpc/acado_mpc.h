#ifndef ACADO_MPC_H
#define ACADO_MPC_H

#include <ros/ros.h>

#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>

#include <std_msgs/Float32MultiArray.h>

#include <reswarm_dmpc/GetControl.h>
#include <reswarm_dmpc/SetWeights.h>

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

class AcadoMPC{
    private:
        // Variables
        ros::NodeHandle nh_;
        ros::ServiceServer get_control_;
        ros::ServiceServer set_weights_;

        // Functions
        bool GetControlCallback(reswarm_dmpc::GetControl::Request &req, reswarm_dmpc::GetControl::Response &res);
        bool SetWeightsCallback(reswarm_dmpc::SetWeights::Request &req, reswarm_dmpc::SetWeights::Response &res);
        void ExtractGetControlData(reswarm_dmpc::GetControl::Request &req);
        void ExtractAcadoPredictedTrajectories(reswarm_dmpc::GetControl::Response &res);
    public:
        // Constructor / Destructor
        AcadoMPC(const ros::NodeHandle &nh);
        virtual ~AcadoMPC();

};

#endif