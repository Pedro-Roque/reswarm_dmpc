#include "reswarm_dmpc/acado_mpc.h"

using namespace std;

AcadoMPC::AcadoMPC(const ros::NodeHandle &nh):
nh_(nh){
  
  // Initialize solver memory
  acado_initializeSolver();

  // Set control service
  get_control_ = nh_.advertiseService("get_control", &AcadoMPC::GetControlCallback, this);

  // Set weigths service
  set_weights_ = nh_.advertiseService("set_weights", &AcadoMPC::SetWeightsCallback, this);

}

AcadoMPC::~AcadoMPC() {
  // Instantiate destructor
}

bool AcadoMPC::GetControlCallback(reswarm_dmpc::GetControl::Request &req, 
                                   reswarm_dmpc::GetControl::Response &res){
  // Extract requested info
  ExtractGetControlData(req);

  // Call controller
  acado_timer t;
	acado_preparationStep();
	acado_tic( &t );
  int status;

  // Perform the feedback step.
  status = acado_feedbackStep( );

	// Extract parameters
  res.status = status;
	res.solution_time = (float) acado_toc( &t );
  res.kkt_value = (float) acado_getKKT();

  // Fill predicted state and control trajectories
  ExtractAcadoPredictedTrajectories(res);
  return true;
}

bool AcadoMPC::SetWeightsCallback(reswarm_dmpc::SetWeights::Request &req, 
                                   reswarm_dmpc::SetWeights::Response &res){
  
  int i = 0;
  // Extract initial state
  for(std::vector<float>::const_iterator it = req.W.begin(); it != req.W.end(); ++it, ++i)
	{
		acadoVariables.W[i] = *it;
	}

  // Reset iteration variable
  i = 0;
  // Extract initial state
  for(std::vector<float>::const_iterator it = req.WN.begin(); it != req.WN.end(); ++it, ++i)
	{
		acadoVariables.WN[i] = *it;
	}

}

void AcadoMPC::ExtractGetControlData(reswarm_dmpc::GetControl::Request &req){

  int i = 0;
  // Extract initial state
  for(std::vector<float>::const_iterator it = req.initial_state.begin(); it != req.initial_state.end(); ++it, ++i)
	{
		acadoVariables.x0[i] = *it;
	}

  // Reset index i
  i = 0;
  // Extract predicted state
  for(std::vector<float>::const_iterator it = req.predicted_state.begin(); it != req.predicted_state.end(); ++it, ++i)
	{
		acadoVariables.x[i] = *it;
	}

  // Reset index i
  i = 0;
  // Extract predicted input
  for(std::vector<float>::const_iterator it = req.predicted_input.begin(); it != req.predicted_input.end(); ++it, ++i)
	{
		acadoVariables.u[i] = *it;
	}

  // Reset index i
  i = 0;
  // Extract online data
  for(std::vector<float>::const_iterator it = req.online_data.begin(); it != req.online_data.end(); ++it, ++i)
	{
		acadoVariables.od[i] = *it;
	}
}

void AcadoMPC::ExtractAcadoPredictedTrajectories(reswarm_dmpc::GetControl::Response &res){

  // Extract predicted state output
  std_msgs::Float32MultiArray state_array;
  state_array.data.clear();
  for(int i = 0; i < ACADO_NX*ACADO_N; i++){
    state_array.data.push_back(acadoVariables.x[i]);
  }

  // Extract predicted control input
  std_msgs::Float32MultiArray control_array;
  control_array.data.clear();
  for(int i = 0; i < ACADO_NU*ACADO_N; i++){
    control_array.data.push_back(acadoVariables.u[i]);
  }
}