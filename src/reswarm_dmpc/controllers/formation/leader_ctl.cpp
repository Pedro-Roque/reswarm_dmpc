#include "reswarm_dmpc/leader_ctl.h"

using namespace std;

LeaderMPC::LeaderMPC(const ros::NodeHandle &nh):
nh_(nh){
  
  // Initialize solver memory
  acado_initializeSolver();

  // Set control service
  get_control_ = nh_.advertiseService("get_control", &LeaderMPC::GetControlCallback, this);

  // Set weigths service
  set_weights_ = nh_.advertiseService("get_control", &LeaderMPC::SetWeightsCallback, this);

}

LeaderMPC::~LeaderMPC() {
  // Instantiate destructor
}

bool LeaderMPC::GetControlCallback(reswarm_dmpc::GetControl::Request &req, 
                                   reswarm_dmpc::GetControl::Response &res){
  // Extract requested info

  // Call controller

  // predicted control and state trajectories

}

bool LeaderMPC::SetWeightsCallback(reswarm_dmpc::SetWeights::Request &req, 
                                   reswarm_dmpc::SetWeights::Response &res){

}