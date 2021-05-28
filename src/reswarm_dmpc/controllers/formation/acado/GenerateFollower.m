% MPC is initialized with zeros, except for linear and angular velocity!
cd Generators/
clear all
close all
acadoSet('results_to_file', false); 

%% NMPC parameters solver
N = 20;
Ts = 0.05;

m = 1.43; 
moment_arm = 0.2;
V_MAX = 0.1;

J = diag([2.3,2.4,21]*10^-2); invJ = inv(J);

%% Problem setup
DifferentialState p1(3) v1(3) q1(4) w1(3) relPos(3)         % position, velocity, heading-(World), angular velocity 
OnlineData relPosD(3) ...                   % desired relative position
           vD(3)      ... 
           vL(3)      ...
           qD(4)
Control u1(6)                                                 % force (2), torque(1)

%% Functions         
Rq = @(q)   [1 - 2*q(2)^2 - 2*q(3)^2,	2*q(1)*q(2) - 2*q(3)*q(4),	2*q(1)*q(3) + 2*q(2)*q(4); ...
             2*q(1)*q(2) + 2*q(3)*q(4),	1 - 2*q(1)^2 - 2*q(3)^2,	2*q(2)*q(3) - 2*q(1)*q(4); ...
             2*q(1)*q(3) - 2*q(2)*q(4),	2*q(2)*q(3) + 2*q(1)*q(4),	1 - 2*q(1)^2 - 2*q(2)^2];
tr = @(R) R(1,1) + R(2,2) + R(3,3);
invskew = @(R) [R(3,2); R(1,3); R(2,1)];
err_ang = @(R,Rd) (invskew(Rd'*R - R'*Rd)/(2*sqrt(1+tr(Rd'*R))));

%% Cost function
W_mat = eye(15); 
WN_mat = eye(9); 
W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);

R1 = Rq(q1);
RD = Rq(qD);
curr_z_vec_1 = R1(:,3);

J1 = [relPos - relPosD; ...
      v1 - vD; ...  
      err_ang(R1,RD); ...
      u1]; % dim 4    TOTAL: 3 + 3 + 3 + 4 = 13
                            
%% Dynamics Agent 1
qx = q1(1); qy = q1(2); qz = q1(3); qw = q1(4);
Xi = [ qw -qz  qy; ...  
       qz  qw -qx; ...
      -qy  qx  qw; ...
      -qx -qy -qz];

w_skew = -[0 -w1(3) w1(2); w1(3) 0 -w1(1); -w1(2) w1(1) 0];

dyn = dot([p1; v1; q1; w1 ]) == [v1; ...
                                  (1/m)*R1*u1(1:3); ...
                                  0.5*Xi*w1; ...
                                  invJ*(-w_skew*J*w1+u1(4:6))];  
                              
% Error propagation:
dynFL = dot(relPos(1:3)) == R1'*(vL-v1);

% Agent 1 control constraints
ocp = acado.OCP(0.0, N*Ts, N);
ocp.minimizeLSQ(  W, J1 );                   
ocp.minimizeLSQEndTerm(  WN, J1(1:end-6) );

ocp.setModel([dyn;dynFL]); 
ocp.subjectTo( -4*m <= u11 <= 4*m);
ocp.subjectTo( -4*m <= u12 <= 4*m); 
ocp.subjectTo( -4*m <= u13 <= 4*m); 
ocp.subjectTo( -m*moment_arm <= u14 <= m*moment_arm); 
ocp.subjectTo( -m*moment_arm <= u15 <= m*moment_arm); 
ocp.subjectTo( -m*moment_arm <= u16 <= m*moment_arm); 
ocp.subjectTo( -V_MAX <= v1 <= V_MAX ); 
 
mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',         2*N                );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
mpc.set( 'HOTSTART_QP',                 'YES'             	);
mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-10				); %Steps over horizon

if strcmp(mexext,'mexa64')
    [~,~] = mkdir('.','SolverFilesFollower');
    cd SolverFilesFollower
    mpc.exportCode('MPC_export');%export code and compile
    copyfile('~/Programs/ACADOtoolkit/external_packages/qpoases', 'MPC_export/qpoases', 'f');
    cd MPC_export
    make_acado_solver('../ControlFollower')
    cd ..
    movefile 'ControlFollower.mexa64' '../../.'    
elseif strcmp(mexext,'mexw64')
    [~,~] = mkdir('.','SolverFilesFollower');
    cd SolverFilesFollower
    mpc.exportCode('MPC_export');%export code and compile
    copyfile('C:\ACADOtoolkit\external_packages\qpoases', 'MPC_export/qpoases', 'f');
    cd MPC_export
    make_acado_solver('../ControlFollower')
    cd ..
    movefile 'ControlFollower.mexw64' '../../.'    
else
    disp('Wrong operating system - are you sure you want to compile here?');
end
cd ../../.