% MPC is initialized with zeros, except for linear and angular velocity!
clear all
close all
acadoSet('results_to_file', false);
followers = 1;
%% NMPC parameters solver
N = 10;
Ts = 0.5;

m = 18.97; 
moment_arm = 0.1;
V_MAX = 0.5;
J = diag([0.2517,0.2517,0.2517]); invJ = inv(J);

% Exponential decay
alpha = 4;
epsilon = 0.00001;


%% Problem setup
DifferentialState p1(3) v1(3) q1(4) w1(3) relPos(3*followers)
OnlineData relPosD(3*followers) qD(4) vD(3)

Control u1(6)                                            

%% Functions         
Rq = @(q)   [1 - 2*q(2)^2 - 2*q(3)^2,	2*q(1)*q(2) - 2*q(3)*q(4),	2*q(1)*q(3) + 2*q(2)*q(4); ...
             2*q(1)*q(2) + 2*q(3)*q(4),	1 - 2*q(1)^2 - 2*q(3)^2,	2*q(2)*q(3) - 2*q(1)*q(4); ...
             2*q(1)*q(3) - 2*q(2)*q(4),	2*q(2)*q(3) + 2*q(1)*q(4),	1 - 2*q(1)^2 - 2*q(2)^2];
tr = @(R) R(1,1) + R(2,2) + R(3,3);
invskew = @(R) [R(3,2); R(1,3); R(2,1)];
err_ang = @(R,Rd) (invskew(Rd'*R - R'*Rd)/(2*sqrt(1+tr(Rd'*R))));

%% Cost function
n = followers;
W_mat = eye(3*n+3+3+6); 
WN_mat = eye(3*n+3+3); 
W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);

R1 = Rq(q1);
RD = Rq(qD);

J1 = [relPos - relPosD; ...
      v1 - vD; ...  
      err_ang(R1,RD); ...
      u1]; % dim 4    TOTAL: 3*n + 3 + 3 + 6 = 13
                            
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
dynamicsVec = [dyn];
% Error propagation:
getError = @(xd,x) xd - x;
% Using the velocity near vmax with an epsilon does not explode, but it is
% a recipe for disaster, it seems
dynF1 = dot(relPos(1:3)) == ((relPosD(1:3)-relPos(1:3))/(norm(relPosD(1:3)-relPos(1:3))+epsilon))* ... 
                        ( (V_MAX-norm(v1-epsilon*[1;1;1]))*(1-exp(-alpha*norm(relPosD(1:3)-relPos(1:3)))));
dynamicsVec = [dynamicsVec;dynF1];
if followers == 2
    dynF2 = dot(relPos(4:6)) == ((relPosD(4:6)-relPos(4:6))/(norm(relPosD(4:6)-relPos(4:6))+epsilon))* ... 
                        ( (V_MAX-norm(v1-epsilon*[1;1;1]))*(1-exp(-alpha*norm(relPosD(4:6)-relPos(4:6)))));
    dynamicsVec = [dynamicsVec;dynF2];
end                   

% Agent 1 control constraints
ocp = acado.OCP(0.0, N*Ts, N);
ocp.minimizeLSQ(  W, J1 );                   
ocp.minimizeLSQEndTerm(  WN, J1(1:end-6) );

ocp.setModel(dynamicsVec);
ocp.subjectTo( -0.6 <= u11 <= 0.6);
ocp.subjectTo( -0.3 <= u12 <= 0.3); 
ocp.subjectTo( -0.3 <= u13 <= 0.3); 
ocp.subjectTo( -0.6*moment_arm <= u14 <= 0.6*moment_arm); 
ocp.subjectTo( -0.3*moment_arm <= u15 <= 0.3*moment_arm); 
ocp.subjectTo( -0.3*moment_arm <= u16 <= 0.3*moment_arm); 
ocp.subjectTo( -V_MAX <= v1 <= V_MAX ); 
 
mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      ); % GAUSS_NEWTON
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       );  % INT_IRK_GL4
mpc.set( 'NUM_INTEGRATOR_STEPS',         2*N                );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
mpc.set( 'HOTSTART_QP',                 'YES'             	);
mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-10				); % -10

% Export the solver
mpc.exportCode('leader_gnd');
delete test.cpp test_data_acadodata_M1.txt test_data_acadodata_M2.txt
delete test_RUN.m test_RUN.mexa64