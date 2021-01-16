// std
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
using namespace std;

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <quadcopter_trajectory_msg/QuadcopterTrajectory.h>
#include <dlt_tmpc/MPCTube.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <dynamic_reconfigure/server.h>
#include <dlt_tmpc/RequestTube.h>

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"
#include "blasfeo/include/blasfeo_i_aux_ext_dep.h"
#include "blasfeo/include/blasfeo_d_aux.h"

// acados
#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

// crazyflie specific
#include "quadcopter_rk4_model/quadcopter_rk4_model.h"
#include "acados_solver_quadcopter_rk4.h"
#include <dlt_tmpc/mpc_params_expConfig.h>
// #include <dlt_tmpc/mpc_params_simConfig.h>

// Number of intervals in the horizon
#define N 15
// Number of differential state variables
#define NX 12
// Number of control inputs
#define NU 4
// Number of measurements/references on nodes 0..N-1
#define NY 16
// Number of measurements/references on node N
#define NYN 12
// Constants
#define pi  3.14159265358979323846

#define SET_WEIGHTS 1
#define SET_COEFF 1
#define SET_CONTROLLER 1
#define SET_CONSTRAINTS 1

#define SETPOINT 2
#define TRACKING 1
#define HOLDING 0 

class NMPC
	{
		enum systemStates{
			xq = 0,
			yq = 1,
			zq = 2,
			vbx = 3,
			vby = 4,
			vbz = 5,
			wx = 6,
			wy = 7,
			wz = 8,
			roll = 9,
			pitch = 10,
			yaw = 11,
		};

		enum controlInputs{
			thurst = 0,
			torquex = 1,
			torquey = 2,
			torquez = 3
		};

		struct solver_output{
			double status, KKT_res, cpu_time;
			double u[(N*NU)];
			double x[(N*NX)];
			};

		struct solver_input{
			double x0[NX];
			double yref[(NY*N)];
			double yref_e[NYN];
			double lbu[(NU*N)];
			double ubu[(NU*N)];
			double lbx[(NX*N)];
			double ubx[(NX*N)];
			double W[NY*NY];
			double WN[NX*NX];
		};

		// acados struct
		solver_output acados_out;
		int acados_status;

		double *trajectory_ptr = NULL;
		int trajectory_size = 0;
		int iter = 0;

		double actual_state[NX] = {0.0};
		double setpoint[NX] = {0.0};
		double target_point[NX] = {0.0};
		bool setpoint_init = false;

		bool enable_mpc = false;

		// Variables for dynamic reconfigure
		double Wdiag_xq,Wdiag_yq,Wdiag_zq;
		double Wdiag_roll,Wdiag_pitch,Wdiag_yaw;
		double Wdiag_vbx,Wdiag_vby,Wdiag_vbz;
		double Wdiag_wx,Wdiag_wy,Wdiag_wz;
		double Wdiag_T,Wdiag_Tx,Wdiag_Ty,Wdiag_Tz;
		
		double WN_factor;

		double lbx[N*NX], ubx[N*NX];

		ros::Subscriber s_trajectory;
		ros::Subscriber s_position;
		ros::Subscriber s_velocity;
		ros::Subscriber s_setpoint;
		ros::Publisher p_command;
		ros::Publisher p_vis_ref;
		ros::Publisher p_vis_mpc;
		ros::Publisher p_error_pose, p_ref_pose, p_ref_velocity;
		ros::Publisher p_error_velocity;

		ros::ServiceServer srv_enable;
		ros::ServiceClient srv_tube_request;

		ros::Time positionTime;
		ros::Time velocityTime;

		double system_mass ;
		double dt;
		double gravity_z = -9.8066;
		int controle_type = HOLDING;
		double thrust_coeff = 0.0;
		double lbu[NU] = {0.0, -1.5, -1.5, -1.5};
		double ubu[NU] = {30.0, 1.5, 1.5, 1.5};
		double lbx_d[NX] = {-10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -1.5, -1.5, -1.5, -30, -30, -180};
		double ubx_d[NX] = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 1.5, 1.5, 1.5, 30, 30, 180};
		
		ros::Time stamp;

	public:
		NMPC(ros::NodeHandle& n){
			int status = 0;

			status = acados_create();

			if (status){
				ROS_INFO_STREAM("acados_create() returned status " << status << ". Exiting." << endl);
				exit(1);
			}
			for (int i = 0 ; i < NX ; i++)
				actual_state[i] = 0.0;

			for(int i = 0 ; i < N ; i++)
				for (int j = 0 ; j<NX ; j++){
					ubx[i*NX + j] = ubx_d[j];
					lbx[i*NX + j] = lbx_d[j];
				}

			s_position = n.subscribe("/mavros/local_position/pose",100, &NMPC::position_callback, this);
			s_velocity = n.subscribe("/mavros/local_position/velocity_local",100, &NMPC::velocity_callback, this);
			s_trajectory = n.subscribe("/poly_trajectory/Trajectory", 5, &NMPC::trajectory_callback, this);
			s_setpoint = n.subscribe("/mavros/setpoint_position/local", 100, &NMPC::setpoint_callback, this);
			
			p_command = n.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1000);
			p_vis_ref = n.advertise<visualization_msgs::MarkerArray>( "mpc_reference_viz", 100 );
			p_vis_mpc = n.advertise<visualization_msgs::MarkerArray>( "mpc_estimation_viz", 100 );
			p_error_pose = n.advertise<geometry_msgs::PoseStamped>( "error_pose", 100 );
			p_error_velocity = n.advertise<geometry_msgs::TwistStamped>( "error_velocity", 100);
			p_ref_pose = n.advertise<geometry_msgs::PoseStamped>( "ref_pose", 100);
			p_ref_velocity = n.advertise<geometry_msgs::TwistStamped>( "ref_velocity", 100);

			srv_enable = n.advertiseService("enable_mpc", &NMPC::enable_callback, this);
			srv_tube_request = n.serviceClient<dlt_tmpc::RequestTube>("/serve_tube");

			ros::param::get("~time_step", dt);
			ros::param::get("~quadcopter/mass", system_mass);
			ros::param::get("~control_type", controle_type);
			ros::param::get("~thrust_coeff", thrust_coeff);
			ros::param::set("/mavros/setpoint_raw/thrust_scaling", -thrust_coeff / (system_mass * gravity_z));
			
			ros::param::get("/default_x_pose", setpoint[xq]);
			ros::param::get("/default_y_pose", setpoint[yq]);
			ros::param::get("/default_z_pose", setpoint[zq]);
		}

//----------------------------------------------------------Callback functions---------------------------------------------------------------------//

		bool enable_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
		{
			// if (req.data == true){
			// 	ROS_INFO("Enable MPC from : [%lf, %lf, %lf] ",target_point[xq], target_point[yq], target_point[zq]);
			// }else{
			// 	ROS_INFO("Disable MPC from : [%lf, %lf, %lf] ",target_point[xq], target_point[yq], target_point[zq]);
			// }	

			if (setpoint_init){
				target_point[xq] = setpoint[xq];
				target_point[yq] = setpoint[yq];
				target_point[zq] = setpoint[zq];
				target_point[yaw] = setpoint[yaw];
				setpoint_init = true;
			}

			enable_mpc = req.data;
			return true;
		}


		/* 
			Callback function for MAVROS setpoint position.
			Storing target setpoint send by external mode for position holding.
		*/
		void setpoint_callback(const geometry_msgs::PoseStampedConstPtr& msg){

			setpoint[xq] = msg->pose.position.x;
			setpoint[yq] = msg->pose.position.y;
			setpoint[zq] = msg->pose.position.z;
			
			tf::Quaternion attitude_q; 
			tf::Vector3 attitude_e;

			// Convert quaternion to euler angle using RPY convention.
			quaternionMsgToTF(msg->pose.orientation, attitude_q);
			tf::Matrix3x3 m(attitude_q);
			m.getRPY(setpoint[roll], setpoint[pitch], setpoint[yaw]);
		}

		/* 
			Callback function for MAVROS local_position pose.
			Storing actual local position.
		*/
		void position_callback(const geometry_msgs::PoseStampedConstPtr& msg){
			actual_state[xq] = msg->pose.position.x;
			actual_state[yq] = msg->pose.position.y;
			actual_state[zq] = msg->pose.position.z;
			
			tf::Quaternion attitude_q; 
			tf::Vector3 attitude_e;

			// Convert quaternion to euler angle using RPY convention.			
			quaternionMsgToTF(msg->pose.orientation, attitude_q);
			tf::Matrix3x3 m(attitude_q);
			m.getRPY(actual_state[roll], actual_state[pitch], actual_state[yaw]);
		}

		/* 
			Callback function for MAVROS local_position velocity.
			Storing local velocity.
		*/
		void velocity_callback(const geometry_msgs::TwistStampedConstPtr& msg){
			actual_state[wx] = msg->twist.angular.x;
			actual_state[wy] = msg->twist.angular.y;
			actual_state[wz] = msg->twist.angular.z;
			actual_state[vbx] = msg->twist.linear.x;
			actual_state[vby] = msg->twist.linear.y;
			actual_state[vbz] = msg->twist.linear.z;
		}
		
		/* 
			Callback function for quadcopter_trajectory trajectory.
			Storing target trajectory publish by external nodes.
		*/
		void trajectory_callback(const quadcopter_trajectory_msg::QuadcopterTrajectoryConstPtr& msg){
			
			//Adapte trajectory array to actual trajectory size
			trajectory_size =  msg->size;
			trajectory_ptr = (double*) malloc(trajectory_size*NY*sizeof(double));

			for (int i= 0; i < trajectory_size; i++){

				tf::Quaternion attitude_q; 
				tf::Vector3 attitude_e;

				// Convert quaternion to euler angle using RPY convention.
				quaternionMsgToTF(msg->states[i].attitude, attitude_q);
				tf::Matrix3x3 m(attitude_q);
				m.getRPY(attitude_e[0], attitude_e[1], attitude_e[2]);

				trajectory_ptr[i * NY + xq] = msg->states[i].position.x;
				trajectory_ptr[i * NY + yq] = msg->states[i].position.y;
				trajectory_ptr[i * NY + zq] = msg->states[i].position.z;
				trajectory_ptr[i * NY + vbx] = msg->states[i].velocity.x;
				trajectory_ptr[i * NY + vby] = msg->states[i].velocity.y;
				trajectory_ptr[i * NY + vbz] = msg->states[i].velocity.z;
				trajectory_ptr[i * NY + wx] = msg->states[i].rates.x; 
				trajectory_ptr[i * NY + wy] = msg->states[i].rates.y;
				trajectory_ptr[i * NY + wz] = msg->states[i].rates.z;
				trajectory_ptr[i * NY + roll] = attitude_e.x();
				trajectory_ptr[i * NY + pitch] = attitude_e.y();
				trajectory_ptr[i * NY + yaw] = attitude_e.z();
				trajectory_ptr[i * NY + NX + thurst] = -gravity_z*system_mass; 
				trajectory_ptr[i * NY + NX + torquex] = 0.0;
				trajectory_ptr[i * NY + NX + torquey] = 0.0;
				trajectory_ptr[i * NY + NX + torquez] = 0.0;

				// ROS_INFO("I heard: [%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf] ",trajectory_ptr[i * NY + 0], trajectory_ptr[i * NY + 1], trajectory_ptr[i * NY + 2], trajectory_ptr[i * NY + 3], trajectory_ptr[i * NY + 4], trajectory_ptr[i * NY + 5], trajectory_ptr[i * NY + 6], trajectory_ptr[i * NY + 7], trajectory_ptr[i * NY + 8], trajectory_ptr[i * NY + 9], trajectory_ptr[i * NY + 10], trajectory_ptr[i * NY + 11], trajectory_ptr[i * NY + 12], trajectory_ptr[i * NY + 13], trajectory_ptr[i * NY + 14], trajectory_ptr[i * NY + 15]);
			}
		}

		/* 
			Callback function for dynamic_reconfigure node.
			Update parameters from dynamic reconfigure GUI.
		*/
		void dynamic_reconfigure_callback(dlt_tmpc::mpc_params_expConfig &config, uint32_t level)
		{
			if (level && SET_CONTROLLER){

				// Switch control mode between position holding, setpoint point regulation and trajectory tracking.
				if (config.enable_holding){
					setpoint_init = true;
					config.enable_traj_tracking = false;
					config.enable_setpoint = false;

					target_point[xq] = actual_state[xq];
					target_point[yq] = actual_state[yq];
					target_point[zq] = actual_state[zq];
					target_point[yaw] = actual_state[yaw];

					controle_type = HOLDING;
				}
				if(config.enable_traj_tracking)
				{
					config.enable_holding = false;
					config.enable_setpoint = false;
					controle_type = TRACKING;
				}
				if(config.enable_setpoint)
				{	
					setpoint_init = true;
					config.enable_traj_tracking = false;
					config.enable_holding = false;
					
					target_point[xq] = config.xq_des;
					target_point[yq] = config.yq_des;
					target_point[zq] = config.zq_des;
					target_point[yaw] = config.yaw_des;

					controle_type = SETPOINT;
				}
			}
			if (level && SET_WEIGHTS)
			{
				//Update MPC weight matrice
				ROS_INFO("Changing MPC setup!");
				Wdiag_xq	= config.Wdiag_xq;
				Wdiag_yq	= config.Wdiag_yq;
				Wdiag_zq	= config.Wdiag_zq;
				Wdiag_roll	= config.Wdiag_roll;
				Wdiag_pitch	= config.Wdiag_pitch;
				Wdiag_yaw	= config.Wdiag_yaw;
				Wdiag_vbx	= config.Wdiag_vbx;
				Wdiag_vby	= config.Wdiag_vby;
				Wdiag_vbz	= config.Wdiag_vbz;
				Wdiag_wx	= config.Wdiag_wx;
				Wdiag_wy	= config.Wdiag_wy;
				Wdiag_wz	= config.Wdiag_wz;
				Wdiag_T  	= config.Wdiag_T;
				Wdiag_Tx	= config.Wdiag_Tx;
				Wdiag_Ty	= config.Wdiag_Ty;
				Wdiag_Tz	= config.Wdiag_Tz;

				WN_factor = config.WN_factor;
			}
		}

//----------------------------------------------------------Building MPC input---------------------------------------------------------------------//

		/* 
			Build acados input structure for MPC trajectory tracking solving. 

			Fill x0 starting point with updated actual position.
			Fill yref array [N*NY] reference using received trajectory.
			Fill yref_e array [NX] final reference using received trajectory.

			return filled structure acados_in
		*/
		solver_input build_input_tracking(void){
			solver_input acados_in;

			for (int i = 0; i < NX; i++)
				acados_in.x0[i] = actual_state[i];
			// ROS_INFO("x0: [%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf] ",acados_in.x0[0], acados_in.x0[1], acados_in.x0[2], acados_in.x0[3], acados_in.x0[4], acados_in.x0[5], acados_in.x0[6], acados_in.x0[7], acados_in.x0[8], acados_in.x0[9], acados_in.x0[10], acados_in.x0[11]);

			for (int i = 0; i < N; i++){
				//Control that reference index do not exceed trajectory size. 
				if (i + iter < trajectory_size){
					acados_in.yref[i * NY + xq] = trajectory_ptr[(i + iter) * NY + xq];
					acados_in.yref[i * NY + yq] = trajectory_ptr[(i + iter) * NY + yq];
					acados_in.yref[i * NY + zq] = trajectory_ptr[(i + iter) * NY + zq];
					acados_in.yref[i * NY + vbx] = trajectory_ptr[(i + iter) * NY + vbx];
					acados_in.yref[i * NY + vby] = trajectory_ptr[(i + iter) * NY + vby];
					acados_in.yref[i * NY + vbz] = trajectory_ptr[(i + iter) * NY + vbz];
					acados_in.yref[i * NY + wx] = trajectory_ptr[(i + iter) * NY + wx];
					acados_in.yref[i * NY + wy] = trajectory_ptr[(i + iter) * NY + wy];
					acados_in.yref[i * NY + wz] = trajectory_ptr[(i + iter) * NY + wz];
					acados_in.yref[i * NY + roll] = trajectory_ptr[(i + iter) * NY + roll];
					acados_in.yref[i * NY + pitch] = trajectory_ptr[(i + iter) * NY + pitch];
					acados_in.yref[i * NY + yaw] = trajectory_ptr[(i + iter) * NY + yaw];
					acados_in.yref[i * NY + NX + thurst] = trajectory_ptr[(i + iter) * NY + thurst];
					acados_in.yref[i * NY + NX + torquex] = trajectory_ptr[(i + iter) * NY + torquex];
					acados_in.yref[i * NY + NX + torquey] = trajectory_ptr[(i + iter) * NY + torquey];
					acados_in.yref[i * NY + NX + torquez] = trajectory_ptr[(i + iter) * NY + torquez];
				}
				// If reference index exceed trajectory size, fill remaining reference with last trajectory point.
				else {
					acados_in.yref[i * NY + xq] = trajectory_ptr[(trajectory_size - 1) * NY + xq];
					acados_in.yref[i * NY + yq] = trajectory_ptr[(trajectory_size - 1) * NY + yq];
					acados_in.yref[i * NY + zq] = trajectory_ptr[(trajectory_size - 1) * NY + zq];
					acados_in.yref[i * NY + vbx] = trajectory_ptr[(trajectory_size - 1) * NY + vbx];
					acados_in.yref[i * NY + vby] = trajectory_ptr[(trajectory_size - 1) * NY + vby];
					acados_in.yref[i * NY + vbz] = trajectory_ptr[(trajectory_size - 1) * NY + vbz];
					acados_in.yref[i * NY + wx] = trajectory_ptr[(trajectory_size - 1) * NY + wx];
					acados_in.yref[i * NY + wy] = trajectory_ptr[(trajectory_size - 1) * NY + wy];
					acados_in.yref[i * NY + wz] = trajectory_ptr[(trajectory_size - 1) * NY + wz];
					acados_in.yref[i * NY + roll] = trajectory_ptr[(trajectory_size - 1) * NY + roll];
					acados_in.yref[i * NY + pitch] = trajectory_ptr[(trajectory_size - 1) * NY + pitch];
					acados_in.yref[i * NY + yaw] = trajectory_ptr[(trajectory_size - 1) * NY + yaw];
					acados_in.yref[i * NY + NX + thurst] = trajectory_ptr[(trajectory_size - 1) * NY + thurst];
					acados_in.yref[i * NY + NX + torquex] = trajectory_ptr[(trajectory_size - 1) * NY + torquex];
					acados_in.yref[i * NY + NX + torquey] = trajectory_ptr[(trajectory_size - 1) * NY + torquey];
					acados_in.yref[i * NY + NX + torquez] = trajectory_ptr[(trajectory_size - 1) * NY + torquez];
				}
				// ROS_INFO("y_ref: [%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf] ",acados_in.yref[i * NY + 0], acados_in.yref[i * NY + 1], acados_in.yref[i * NY + 2], acados_in.yref[i * NY + 3], acados_in.yref[i * NY + 4], acados_in.yref[i * NY + 5], acados_in.yref[i * NY + 6], acados_in.yref[i * NY + 7], acados_in.yref[i * NY + 8], acados_in.yref[i * NY + 9], acados_in.yref[i * NY + 10], acados_in.yref[i * NY + 11], acados_in.yref[i * NY + 12], acados_in.yref[i * NY + 13], acados_in.yref[i * NY + 14], acados_in.yref[i * NY + 15]);
			}

			acados_in.yref_e[xq] = ((N + iter) < trajectory_size) ? trajectory_ptr[(N + iter) * NY + xq] : trajectory_ptr[(trajectory_size-1) * NY + xq];
			acados_in.yref_e[yq] = ((N + iter) < trajectory_size) ? trajectory_ptr[(N + iter) * NY + yq] : trajectory_ptr[(trajectory_size-1) * NY + yq];
			acados_in.yref_e[zq] = ((N + iter) < trajectory_size) ? trajectory_ptr[(N + iter) * NY + zq] : trajectory_ptr[(trajectory_size-1) * NY + zq];
			acados_in.yref_e[vbx] = ((N + iter) < trajectory_size) ? trajectory_ptr[(N + iter) * NY + vbx] : trajectory_ptr[(trajectory_size-1) * NY + vbx];
			acados_in.yref_e[vby] = ((N + iter) < trajectory_size) ? trajectory_ptr[(N + iter) * NY + vby] : trajectory_ptr[(trajectory_size-1) * NY + vby];
			acados_in.yref_e[vbz] = ((N + iter) < trajectory_size) ? trajectory_ptr[(N + iter) * NY + vbz] : trajectory_ptr[(trajectory_size-1) * NY + vbz];
			acados_in.yref_e[wx] = ((N + iter) < trajectory_size) ? trajectory_ptr[(N + iter) * NY + wx] : trajectory_ptr[(trajectory_size-1) * NY + wx];
			acados_in.yref_e[wy] = ((N + iter) < trajectory_size) ? trajectory_ptr[(N + iter) * NY + wy] : trajectory_ptr[(trajectory_size-1) * NY + wy];
			acados_in.yref_e[wz] = ((N + iter) < trajectory_size) ? trajectory_ptr[(N + iter) * NY + wz] : trajectory_ptr[(trajectory_size-1) * NY + wz];
			acados_in.yref_e[roll] = ((N + iter) < trajectory_size) ? trajectory_ptr[(N + iter) * NY + roll] : trajectory_ptr[(trajectory_size-1) * NY + roll];
			acados_in.yref_e[pitch] = ((N + iter) < trajectory_size) ? trajectory_ptr[(N + iter) * NY + pitch] : trajectory_ptr[(trajectory_size-1) * NY + pitch];
			acados_in.yref_e[yaw] = ((N + iter) < trajectory_size) ? trajectory_ptr[(N + iter) * NY + yaw] : trajectory_ptr[(trajectory_size-1) * NY + yaw];

			// ROS_INFO("Yref_e: [%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf] ",acados_in.yref_e[0], acados_in.yref_e[1], acados_in.yref_e[2], acados_in.yref_e[3], acados_in.yref_e[4], acados_in.yref_e[5], acados_in.yref_e[6], acados_in.yref_e[7], acados_in.yref_e[8], acados_in.yref_e[9], acados_in.yref_e[10], acados_in.yref_e[11]);
			return acados_in;
		}

		/* 
			Build acados input structure for MPC trajectory tracking solving. 
			
			Fill x0 starting point with updated actual position.
			Fill yref array [N*NY] reference using target point, can be new setpoint or holding point.
			Fill yref_e array [NX] final reference using target point, can be new setpoint or holding point.

			return filled structure acados_in
		*/
		solver_input build_input_hold(void){
			solver_input acados_in;
			
			for (int i = 0; i < NX; i++)
				acados_in.x0[i] = actual_state[i];
			// ROS_INFO("x0: [%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf] ",acados_in.x0[0], acados_in.x0[1], acados_in.x0[2], acados_in.x0[3], acados_in.x0[4], acados_in.x0[5], acados_in.x0[6], acados_in.x0[7], acados_in.x0[8], acados_in.x0[9], acados_in.x0[10], acados_in.x0[11]);

			for (int i = 0; i < N; i++){
				
				acados_in.yref[i * NY + xq] = target_point[xq];
				acados_in.yref[i * NY + yq] = target_point[yq];
				acados_in.yref[i * NY + zq] = target_point[zq];
				acados_in.yref[i * NY + vbx] = 0.0;
				acados_in.yref[i * NY + vby] = 0.0;
				acados_in.yref[i * NY + vbz] = 0.0;
				acados_in.yref[i * NY + wx] = 0.0;
				acados_in.yref[i * NY + wy] = 0.0;
				acados_in.yref[i * NY + wz] = 0.0;
				acados_in.yref[i * NY + roll] = 0.0;
				acados_in.yref[i * NY + pitch] = 0.0;
				acados_in.yref[i * NY + yaw] = target_point[yaw];
				acados_in.yref[i * NY + NX + thurst] = -gravity_z*system_mass;
				acados_in.yref[i * NY + NX + torquex] = 0.0;
				acados_in.yref[i * NY + NX + torquey] = 0.0;
				acados_in.yref[i * NY + NX + torquez] = 0.0;
				// ROS_INFO("y_ref: [%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf] ",acados_in.yref[i * NY + 0], acados_in.yref[i * NY + 1], acados_in.yref[i * NY + 2], acados_in.yref[i * NY + 3], acados_in.yref[i * NY + 4], acados_in.yref[i * NY + 5], acados_in.yref[i * NY + 6], acados_in.yref[i * NY + 7], acados_in.yref[i * NY + 8], acados_in.yref[i * NY + 9], acados_in.yref[i * NY + 10], acados_in.yref[i * NY + 11], acados_in.yref[i * NY + 12], acados_in.yref[i * NY + 13], acados_in.yref[i * NY + 14], acados_in.yref[i * NY + 15]);
			}

			acados_in.yref_e[xq] = target_point[xq];
			acados_in.yref_e[yq] = target_point[yq];
			acados_in.yref_e[zq] = target_point[zq];
			acados_in.yref_e[vbx] = 0.0;
			acados_in.yref_e[vby] = 0.0;
			acados_in.yref_e[vbz] = 0.0;
			acados_in.yref_e[wx] = 0.0;
			acados_in.yref_e[wy] = 0.0;
			acados_in.yref_e[wz] = 0.0;
			acados_in.yref_e[roll] = 0.0;
			acados_in.yref_e[pitch] = 0.0;
			acados_in.yref_e[yaw] = target_point[yaw];

			// ROS_INFO("Yref_e: [%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf] ",acados_in.yref_e[0], acados_in.yref_e[1], acados_in.yref_e[2], acados_in.yref_e[3], acados_in.yref_e[4], acados_in.yref_e[5], acados_in.yref_e[6], acados_in.yref_e[7], acados_in.yref_e[8], acados_in.yref_e[9], acados_in.yref_e[10], acados_in.yref_e[11]);
			return acados_in;
		}

		/* 
			Update MPC weight in acados input structure. 
			
			Fill W weight matrice using state weight.
			Fill Wn final weight matrice using state weight and multiplication factor.

			return Updated structure target
		*/
		solver_input set_mpc_weight(solver_input target){

			target.W[xq+xq*(NU+NX)]   = Wdiag_xq;
			target.W[yq+yq*(NU+NX)]   = Wdiag_yq;
			target.W[zq+zq*(NU+NX)]   = Wdiag_zq;
			target.W[vbx+vbx*(NU+NX)]   = Wdiag_vbx;
			target.W[vby+vby*(NU+NX)]   = Wdiag_vby;
			target.W[vbz+vbz*(NU+NX)]   = Wdiag_vbz;
			target.W[wx+wx*(NU+NX)] = Wdiag_wx;
			target.W[wy+wy*(NU+NX)] = Wdiag_wy;
			target.W[wz+wz*(NU+NX)] = Wdiag_wz;
			target.W[roll+roll*(NU+NX)]   = Wdiag_roll;
			target.W[pitch+pitch*(NU+NX)]   = Wdiag_pitch;
			target.W[yaw+yaw*(NU+NX)]   = Wdiag_yaw;
			target.W[(NX+thurst)+(NX+thurst)*(NU+NX)] = Wdiag_T;
			target.W[(NX+torquex)+(NX+torquex)*(NU+NX)] = Wdiag_Tx;
			target.W[(NX+torquey)+(NX+torquey)*(NU+NX)] = Wdiag_Ty;
			target.W[(NX+torquez)+(NX+torquez)*(NU+NX)] = Wdiag_Tz;

			target.WN[xq+xq*(NX)]   = Wdiag_xq * WN_factor;
			target.WN[yq+yq*(NX)]   = Wdiag_yq * WN_factor;
			target.WN[zq+zq*(NX)]   = Wdiag_zq * WN_factor;
			target.WN[vbx+vbx*(NX)]   = Wdiag_vbx * WN_factor;
			target.WN[vby+vby*(NX)]   = Wdiag_vby * WN_factor;
			target.WN[vbz+vbz*(NX)]   = Wdiag_vbz * WN_factor;
			target.WN[wx+wx*(NX)] = Wdiag_wx * WN_factor;
			target.WN[wy+wy*(NX)] = Wdiag_wy * WN_factor;
			target.WN[wz+wz*(NX)] = Wdiag_wz * WN_factor;
			target.WN[roll+roll*(NX)]   = Wdiag_roll * WN_factor;
			target.WN[pitch+pitch*(NX)]   = Wdiag_pitch * WN_factor;
			target.WN[yaw+yaw*(NX)]   = Wdiag_yaw * WN_factor;

			return target;
		}

		/* 
			Update MPC constraints in acados input structure. 
			
			Fill constraints with dynamic updated constraints or default constraints. 

			return Updated structure target
		*/
		solver_input set_constraints(solver_input target){

			for (int i = 0 ; i < N ; i++){
				target.lbu[i * NU + thurst] = lbu[thurst];
				target.lbu[i * NU + torquex] = lbu[torquex];
				target.lbu[i * NU + torquey] = lbu[torquey];
				target.lbu[i * NU + torquez] = lbu[torquez];

				target.ubu[i * NU + thurst] = ubu[thurst];
				target.ubu[i * NU + torquex] = ubu[torquex];
				target.ubu[i * NU + torquey] = ubu[torquey];
				target.ubu[i * NU + torquez] = ubu[torquez];

				target.lbx[i * NX + xq] = lbx[i * NX +xq];
				target.lbx[i * NX + yq] = lbx[i * NX +yq];
				target.lbx[i * NX + zq] = lbx[i * NX +zq];
				target.lbx[i * NX + vbx] = lbx[i * NX +vbx];
				target.lbx[i * NX + vby] = lbx[i * NX +vby];
				target.lbx[i * NX + vbz] = lbx[i * NX +vbz];
				target.lbx[i * NX + wx] = lbx[i * NX +wx];
				target.lbx[i * NX + wy] = lbx[i * NX +wy];
				target.lbx[i * NX + wz] = lbx[i * NX +wz];
				target.lbx[i * NX + roll] = lbx[i * NX +roll];
				target.lbx[i * NX + pitch] = lbx[i * NX +pitch];
				target.lbx[i * NX + yaw] = lbx[i * NX +yaw];

				target.ubx[i * NX + xq] = ubx[i * NX + xq];
				target.ubx[i * NX + yq] = ubx[i * NX + yq];
				target.ubx[i * NX + zq] = ubx[i * NX + zq];
				target.ubx[i * NX + vbx] = ubx[i * NX +vbx];
				target.ubx[i * NX + vby] = ubx[i * NX +vby];
				target.ubx[i * NX + vbz] = ubx[i * NX +vbz];
				target.ubx[i * NX + wx] = ubx[i * NX +wx];
				target.ubx[i * NX + wy] = ubx[i * NX +wy];
				target.ubx[i * NX + wz] = ubx[i * NX +wz];
				target.ubx[i * NX + roll] = ubx[i * NX +roll];
				target.ubx[i * NX + pitch] = ubx[i * NX +pitch];
				target.ubx[i * NX + yaw] = ubx[i * NX +yaw];
			}
			return target;	
		}

		/* 
			Refresh constraints. 
			
			Fill constraints calling external service.

			return Updated structure target
		*/
		void refresh_tube(int index){
			dlt_tmpc::RequestTube srv; 
			srv.request.index = index;

			if (srv_tube_request.call(srv))
				for (int i = 0 ; i < N ; i++){
					ubx[i*NX + xq] = srv.response.tube.sets[i].ubx[xq];
					lbx[i*NX + xq] = srv.response.tube.sets[i].lbx[xq];
					ubx[i*NX + yq] = srv.response.tube.sets[i].ubx[yq];
					lbx[i*NX + yq] = srv.response.tube.sets[i].lbx[yq];
					ubx[i*NX + zq] = srv.response.tube.sets[i].ubx[zq];
					lbx[i*NX + zq] = srv.response.tube.sets[i].lbx[zq];

					ubx[i*NX + vbx] = srv.response.tube.sets[i].ubx[vbx];
					lbx[i*NX + vbx] = srv.response.tube.sets[i].lbx[vbx];
					ubx[i*NX + vby] = srv.response.tube.sets[i].ubx[vby];
					lbx[i*NX + vby] = srv.response.tube.sets[i].lbx[vby];
					ubx[i*NX + vbz] = srv.response.tube.sets[i].ubx[vbz];
					lbx[i*NX + vbz] = srv.response.tube.sets[i].lbx[vbz];

					ubx[i*NX + roll] = srv.response.tube.sets[i].ubx[roll];
					lbx[i*NX + roll] = srv.response.tube.sets[i].lbx[roll];
					ubx[i*NX + pitch] = srv.response.tube.sets[i].ubx[pitch];
					lbx[i*NX + pitch] = srv.response.tube.sets[i].lbx[pitch];
					ubx[i*NX + yaw] = srv.response.tube.sets[i].ubx[yaw];
					lbx[i*NX + yaw] = srv.response.tube.sets[i].lbx[yaw];
					
					ubx[i*NX + wx] = srv.response.tube.sets[i].ubx[wx];
					lbx[i*NX + wx] = srv.response.tube.sets[i].lbx[wx];
					ubx[i*NX + wy] = srv.response.tube.sets[i].ubx[wy];
					lbx[i*NX + wy] = srv.response.tube.sets[i].lbx[wy];
					ubx[i*NX + wz] = srv.response.tube.sets[i].ubx[wz];
					lbx[i*NX + wz] = srv.response.tube.sets[i].lbx[wz];
				}
			else
				ROS_ERROR("Failed to call service requestTube");
		}

//----------------------------------------------------------Publication functions---------------------------------------------------------------------//

		/* 
			Publish MPC solved steps and MPC reference for Rviz visualization.
		*/
		void publish_visualization(solver_output results, solver_input input){
			visualization_msgs::MarkerArray markerArrayEst;
			visualization_msgs::MarkerArray markerArrayRef;

			for (int i = 0 ; i < N ; i++){
				visualization_msgs::Marker marker;
				tf::Quaternion attitude;

				marker.header.frame_id = "map"; // Target frame
				marker.id = i;					// Marker index
				marker.type = visualization_msgs::Marker::ARROW;	// Marker shape
				marker.action = visualization_msgs::Marker::ADD;	// Marker action

				// Build marker with MPC solved points
				marker.pose.position.x = results.x[NX * i + xq];	// Marker position x axis
				marker.pose.position.y = results.x[NX * i + yq];	// Marker position y axis
				marker.pose.position.z = results.x[NX * i + zq];	// Marker position z axis

				attitude.setRPY(results.x[NX * i + roll], results.x[NX * i + pitch], results.x[NX * i + yaw]);	// Build quaternion for visualization.

				marker.pose.orientation.x = attitude.x();	// Marker attitude x
				marker.pose.orientation.y = attitude.y();	// Marker attitude y
				marker.pose.orientation.z = attitude.z();	// Marker attitude z
				marker.pose.orientation.w = attitude.w();	// Marker attitude w
				marker.scale.x = 1.0;			// Marker axis display scale 
				marker.scale.y = 0.1;			// Marker axis display scale 
				marker.scale.z = 0.1;			// Marker axis display scale 
				marker.color.a = 1.0;			// Marker alpha coeff
				marker.color.r = 1.0;			// Marker display color
				marker.color.g = 0.0;			// Marker display color
				marker.color.b = 0.0;			// Marker display color
				markerArrayEst.markers.push_back(marker);

				// Build marker with MPC reference points				
				marker.pose.position.x = input.yref[NY * i + xq];	// Marker position x axis
				marker.pose.position.y = input.yref[NY * i + yq];	// Marker position y axis
				marker.pose.position.z = input.yref[NY * i + zq];	// Marker position z axis

				attitude.setRPY(input.yref[NY * i + roll], input.yref[NY * i + pitch], input.yref[NY * i + yaw]);	// Build quaternion for visualization.
				
				marker.pose.orientation.x = attitude.x();	// Marker attitude x
				marker.pose.orientation.y = attitude.y();	// Marker attitude y
				marker.pose.orientation.z = attitude.z();	// Marker attitude z
				marker.pose.orientation.w = attitude.w();	// Marker attitude w
				marker.scale.x = 1.0;			// Marker axis display scale 
				marker.scale.y = 0.1;			// Marker axis display scale 
				marker.scale.z = 0.1;			// Marker axis display scale 
				marker.color.a = 1.0;			// Marker alpha coeff
				marker.color.r = 0.0;			// Marker display color
				marker.color.g = 0.0;			// Marker display color
				marker.color.b = 1.0;			// Marker display color
				
				markerArrayRef.markers.push_back(marker);

			}

			// Publish
			p_vis_mpc.publish( markerArrayEst );
			p_vis_ref.publish( markerArrayRef );
		}

		/* 
			Publish MPC optimal input for quadcopter command.
		*/
		void publish_command(solver_output results){
			//	Use target attitude message to command quadcopter using optimal computed angular velocity and thrust.
			mavros_msgs::AttitudeTarget target_attitude;

			target_attitude.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
			target_attitude.body_rate.x = results.x[NX + wx];
			target_attitude.body_rate.y = results.x[NX + wy];
			target_attitude.body_rate.z = results.x[NX + wz];
			target_attitude.thrust = results.u[0];

			p_command.publish( target_attitude );
		}

		/* 
			Publish Error between last received local position and target point on custom topic. 
		*/
		void publish_error(solver_input input){

			geometry_msgs::PoseStamped pose_msg;
			geometry_msgs::TwistStamped velocity_msg;
			tf::Quaternion attitude;

			pose_msg.pose.position.x = input.x0[xq]- input.yref[xq];
			pose_msg.pose.position.y = input.x0[yq] - input.yref[yq];
			pose_msg.pose.position.z = input.x0[zq] - input.yref[zq];

			attitude.setRPY( (input.x0[roll]- input.yref[roll]), (input.x0[pitch] - input.yref[pitch]), (input.x0[yaw] - input.yref[yaw]));

			pose_msg.pose.orientation.x = attitude.x();
			pose_msg.pose.orientation.y = attitude.y();
			pose_msg.pose.orientation.z = attitude.z();
			pose_msg.pose.orientation.w = attitude.w();

			velocity_msg.twist.angular.x = input.x0[wx] - input.yref[wx];
			velocity_msg.twist.angular.y = input.x0[wy] - input.yref[wy];
			velocity_msg.twist.angular.z = input.x0[wz] - input.yref[wz];

			velocity_msg.twist.linear.x = input.x0[vbx] - input.yref[vbx];
			velocity_msg.twist.linear.y = input.x0[vby] - input.yref[vby];
			velocity_msg.twist.linear.z = input.x0[vbz] - input.yref[vbz];
			
			p_error_pose.publish(pose_msg);
			p_error_velocity.publish(velocity_msg);

			pose_msg.pose.position.x = input.yref[xq];
			pose_msg.pose.position.y = input.yref[yq];
			pose_msg.pose.position.z = input.yref[zq];

			attitude.setRPY( (input.yref[roll] ), (input.yref[pitch] ), (input.yref[yaw] ));

			pose_msg.pose.orientation.x = attitude.x();
			pose_msg.pose.orientation.y = attitude.y();
			pose_msg.pose.orientation.z = attitude.z();
			pose_msg.pose.orientation.w = attitude.w();

			velocity_msg.twist.angular.x = input.yref[wx];
			velocity_msg.twist.angular.y = input.yref[wy];
			velocity_msg.twist.angular.z = input.yref[wz];

			velocity_msg.twist.linear.x = input.yref[vbx];
			velocity_msg.twist.linear.y = input.yref[vby];
			velocity_msg.twist.linear.z = input.yref[vbz];
	
			p_ref_pose.publish(pose_msg);
			p_ref_velocity.publish(velocity_msg);
		}

		/* 
			Solve MPC function
		*/
		int solve_mpc(void){
			solver_input acados_in;

			if (enable_mpc){
				switch(controle_type){
					case HOLDING:
						acados_in = build_input_hold();
						iter = 0;
						break;
					case TRACKING:
						acados_in = build_input_tracking();
						// refresh_tube(iter);
						break;
					case SETPOINT:
						acados_in = build_input_hold();
						break;
					default:
						ROS_WARN("Unexisting control type. Switch off MPC.");
						enable_mpc = false;
						break;
				}

				acados_in = set_mpc_weight(acados_in);
				acados_in = set_constraints(acados_in);

				// Setup acados problem
				ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", acados_in.x0); // Set initial point constraint
				ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", acados_in.x0); // Set initial point constraint
				
				for (int j=0; j< N; j++){
					ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, j, "yref", acados_in.yref + j*NY);	 // Set reference for the N steps
				}
				
				ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", acados_in.yref_e); // Set final references

				if (SET_WEIGHTS)
					for (int ii = 0; ii < N; ii++){
						ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "W", acados_in.W);
					}
					ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", acados_in.WN);

				if (SET_CONSTRAINTS)
					for (int jj = 0; jj < N; jj++){
						ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, jj, "lbu", acados_in.lbu + jj*NU);
						ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, jj, "ubu", acados_in.ubu + jj*NU);	
						if (jj > 0){
							ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, jj, "lbx", acados_in.lbx + (jj-1)*NX);
							ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, jj, "ubx", acados_in.ubx + (jj-1)*NX);		
						}			
					}

				// Solve acados problem
				acados_status = acados_solve();
				
				// Assign output signals
				acados_out.status = acados_status;
				acados_out.KKT_res = (double)nlp_out->inf_norm_res;
				acados_out.cpu_time = (double)nlp_out->total_time;

				// Extract optimal computed inputs and states
				for (int i = 0 ; i < N ; i++){
					ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "u", acados_out.u + i*NU);
					ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", acados_out.x + i*NX);
					// cout << "[" << acados_out.x[i*NX + 0] << " " << acados_out.x[i*NX + 1] << " " << acados_out.x[i*NX + 2] <<  "]" << endl;
					// cout << "[" << acados_out.u[i*NU + 0] << "]" << endl;
				}	
				// ROS_INFO("x0: [%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf] ",acados_in.x0[0], acados_in.x0[1], acados_in.x0[2], acados_in.x0[3], acados_in.x0[4], acados_in.x0[5], acados_in.x0[6], acados_in.x0[7], acados_in.x0[8], acados_in.x0[9], acados_in.x0[10], acados_in.x0[11]);
				// ROS_INFO("Yref_e: [%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf] ",acados_in.yref_e[0], acados_in.yref_e[1], acados_in.yref_e[2], acados_in.yref_e[3], acados_in.yref_e[4], acados_in.yref_e[5], acados_in.yref_e[6], acados_in.yref_e[7], acados_in.yref_e[8], acados_in.yref_e[9], acados_in.yref_e[10], acados_in.yref_e[11]);
				// ROS_INFO("U: [%lf, %lf, %lf, %lf] ",acados_out.u[0], acados_out.u[1], acados_out.u[2], acados_out.u[3]);
				// ROS_INFO("X1: [%lf, %lf, %lf, %lf, %lf, %lf] ",acados_out.x[NX + 6], acados_out.x[NX + 7], acados_out.x[NX + 8], acados_out.x[NX + 9], acados_out.x[NX + 10], acados_out.x[NX + 11]);
				// cout << "Solved in : ";
				// cout << acados_out.cpu_time << endl;
			
				ros::Time begin = ros::Time::now();
				
				// Publication round
				publish_command(acados_out);
				publish_visualization(acados_out, acados_in);
				publish_error(acados_in);

				iter++;
				ros::Time end = ros::Time::now();
				// cout << "Publication time " <<  end - begin << endl;

			}else {
				// acados_in = build_input_hold();	
				// publish_error(acados_in);
				iter = 0 ;
			}

			return 0;
		}

		void run(ros::NodeHandle& n){

			ros::Timer nmpc_controller = n.createTimer(ros::Duration(dt), &NMPC::nmpcCallback, this);

			dynamic_reconfigure::Server<dlt_tmpc::mpc_params_expConfig> server;
			dynamic_reconfigure::Server<dlt_tmpc::mpc_params_expConfig>::CallbackType f;
			f = boost::bind(&NMPC::dynamic_reconfigure_callback, this, _1, _2);
			server.setCallback(f);

			ros::spin();
		}

		void nmpcCallback(const ros::TimerEvent&){
			solve_mpc();
		}

	};

int main(int argc, char **argv){
	ros::init(argc, argv, "cf_nmpc");
	ros::NodeHandle n("~");

	NMPC nmpc(n);
	nmpc.run(n);

	return 0;
}