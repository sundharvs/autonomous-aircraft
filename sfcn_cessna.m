% sfcn_cessna.m - Models the Cessna 172 Longitudinal Motion
function [sys,x0,str,ts] = sfcn_cessna(t,x,u,flag)
% t is time
% x is state
% u is inout
% flag is a calling argument used by Simulink.
% The value of flag determines what Simulink wants to be executed.
switch flag
case 0 % Initialization
[sys,x0,str,ts]=mdlInitializeSizes;
case 1 % Compute xdot
sys=mdlDerivatives(t,x,u);
case 2 % Not needed for continuous-time systems
case 3 % Compute output
sys = mdlOutputs(t,x,u);
case 4 % Not needed for continuous-time systems
case 9 % Not needed here
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mdlInitializeSizes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
function [sys,x0,str,ts]=mdlInitializeSizes
%
% Create the sizes structure
sizes=simsizes;
sizes.NumContStates = 4; %Set number of continuous-time state variables
sizes.NumDiscStates = 0;
sizes.NumOutputs = 2; %Set number of outputs
sizes.NumInputs = 2; %Set number of intputs
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1; %Need at least one sample time
sys = simsizes(sizes);
%x0=[1.465515441321413e+02;0.065286128206560;-0.005037403392839;-9.841730839629762e-05]; % Set initial state
x0=[1.465743251497600e+02;0.055097602429410;-0.013603290175767;0.037445905481846];
% x0=[162;0.05;0.05;0]; % Set initial state
str=[]; % str is always an empty matrix
ts=[0 0]; % ts must be a matrix of at least one row and two columns
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mdlDerivatives
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
function sys = mdlDerivatives(t,x,u)
%
% Compute xdot based on (t,x,u) and set it equal to sys
%
% State Variables
V = x(1);
alpha = x(2);
theta = x(3);
q = x(4);
% Given Constants
W = 2650; %lbs
S = 174; %ft^2
cbar = 4.9; %ft
J2 = 1346; %slug*ft^2
rho = 2.377e-3; %slug/ft^3 (sea-level)
g = 32.2; %ft/s^2
m = W/g; %lbm
% Dynamic Pressure
qbar = 1/2*rho*V^2;
% Lift Coefficient
Cl0 = 0.307;
Cla = 4.41; %/rad
Clel = 0.43; %/rad
Cladot = 1.7; %/rad
Clq = 3.9; %/rad
% Drag Coefficient
CdM = 0.0223;
k = 0.0554;
CldM = 0;
% Pitching Moment Coefficient
Cm0R = 0.04;
CmaR = -0.613; %/rad
Cmel = -1.122; %/rad
Cmadot = -7.27; %/rad
Cmq = -12.4; %/rad
% Other Specs
eta = 0.7;
ep = 0;
% System Inputs
th = u(1);
el = u(2);
% Thrust
T = 550*th*eta/V;
% Gamma
gamma = theta-alpha;
% Calculate alpha_dot
term1 = -qbar*S*(Cl0+Cla*alpha+Clel*el+cbar/(2*V)*Clq*q);
num = term1 + W*cos(gamma)-T*sin(alpha-ep)+m*V*q;
den = m*V-cbar*Cladot/(2*V);
alpha_dot = num/den;
% Calculate lift, drag, pitch coefficients
Cl = Cl0+Cla*alpha+Clel*el+cbar/(2*V)*Clq*q+cbar/(2*V)*Cladot*alpha_dot;
Cd = CdM+k*(Cl0+Cla*alpha+Clel*el-CldM)^2;
Cm = Cm0R+CmaR*alpha+Cmel*el+cbar/(2*V)*Cmadot*alpha_dot+cbar/(2*V)*Cmq*q;
% Calculate lift, drag, pitch moment
L = qbar*S*Cl;
D = qbar*S*Cd;
M = qbar*S*cbar*Cm;
% Calculate V_dot
V_dot = 1/m*(-D-W*sin(gamma)+T*cos(alpha-ep));

% Calculate theta_dot
theta_dot = q;
% Calculate q_dot
q_dot = M/J2;
% Derivatives
sys(1) = V_dot;
sys(2) = alpha_dot;
sys(3) = theta_dot;
sys(4) = q_dot;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mdlOutput
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
function sys = mdlOutputs(t,x,u)
%
% Compute output based on (t,x,u) and set it equal to sys
sys(1) = x(1); %Output Velocity
sys(2) = x(3)-x(2); %Output Gamma