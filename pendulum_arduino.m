%% Motor
% Resistance
Rm = 2.0;
% Current-torque (N-m/A)
kt = 0.018; % 0.018
% Back-emf constant (V-s/rad)
ke = 0.018;
% Gear ratio
kg = 5.75;  % 5.75
% Efficiency
eff = 0.8;

%% Rotary Arm
% Mass (kg)
Mr = 0.095;
% Total length (m)
Lr = 0.085;
% Moment of inertia about pivot (kg-m^2)
Jr = Mr*Lr^2/12 * 1.1;
% Equivalent Viscous Damping Coefficient (N-m-s/rad)
Dr = 1e-3;

%% Pendulum Link
% Mass (kg)
Mp = 0.024;
% Total length (m)
Lp = 0.129;
% Moment of inertia about pivot (kg-m^2)
Jp = Mp*Lp^2/12;
% Equivalent Viscous Damping Coefficient (N-m-s/rad)
Dp = 1e-5;
% Gravity Constant
g = 9.81;

%% Second-order states breakdown
% There is an analytical derivation in the Rotary Pendulum workbook
% (not QUBE) in pages 9-10.
% "Add actuator dynamics" procedure in QUBE files is wrong,
% the B line should come AFTER the A lines.

acc_matrix=[(Mp*Lr^2+Jr), -0.5*Mp*Lp*Lr;
            -0.5*Mp*Lp*Lr, (0.25*Mp*Lp^2+Jp)];
state_matrix=[0, 0, -(Dr+eff*kt*ke*kg^2/Rm), 0;
              0, 0.5*Mp*Lp*g, 0, -Dp];
input_matrix=[eff*kt*kg/Rm; 0];

disjointed_state=acc_matrix\state_matrix;
disjointed_input=acc_matrix\input_matrix;

%% State-space model
A=[zeros(2), eye(2); disjointed_state];
B=[0; 0; disjointed_input];
C=eye(4); %[eye(2), zeros(2)];
D=zeros(4,1); %zeros(2,1);
hpf_freq = 100;
system=ss(A,B,C,D);
R=5;
Q=diag([2, 1, 1, 1]);
K=lqr(A,B,Q,R)

sim("s_rotpen_bal.mdl");






