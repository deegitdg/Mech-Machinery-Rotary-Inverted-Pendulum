%% Inverted Pendulum model parameters
% Run this block to initialise simscape pendulum model parameters

% DC Motor Parameters
kt=0.07;% torque constant
km=0.042;% motor back-emf constant
R=8.4;% terminal resistance
L=0.85e-3;% terminal inductance

% Inverted Pendulum Parameters
% lengths are in cm and mass in kg
base_size=9.7; % length of pendulum base cube
% Rotary arm parameters
rod_rad=0.3;% rotary arm radius
rod_length=10;% rotary arm length
rod_mass=0.05;% rotary arm mass
% Pendulum parameters
p_rad=0.03;% pendulum radius
p_length=10;% pendulum length
p_mass=0.1;% pendulum mass
rt1=base_size/2;
rt2=rod_length-0.4;
rt4=p_length/2;
D_r=0.0015/2;
D_p=0.0005/2;

% Initial pendulum intial angle in degrees
% 180 corresponds to upright position
% can also be selected in the simulink model
alpha_0=170;

%% State space equation for state-feedback controller for Inverted Pendulum
% Run this block to initialise state-space controller parameters

% Inverted Pendulum Parameters
% Rotary arm parameters
L_r=rod_length/100;%convert cm to m
r_r=rod_rad/100;%convert cm to m
m_r=rod_mass;
J_r=(1/4)*m_r*r_r^2+(1/3)*m_r*L_r^2;% rotary arm moment of inertia
% Pendulum parameters
L_p=p_length/100;
m_p=p_mass;
r_p=p_rad/100;
J_p=(1/4)*m_p*r_p^2+(1/3)*m_p*L_p^2;% pendulum moment of inertia

% System linearized at [0 0 0 0]' or [0 pi 0 0]'
% For [0 0 0 0]' set l=1 For comparing state-space and nonlinear model
% For [0 pi 0 0]' set l=-1 (for controller design)
l=-1;

M=[m_p*L_r^2+J_r -l*0.5*m_p*L_p*L_r;
   l*0.5*m_p*L_p*L_r J_p+0.25*m_p*L_p^2];
G=[D_r+kt*km/R 0;0 D_p];
K=[0 0;0 l*0.5*m_p*L_p*9.8];
F=[kt/R;0];

%Linearized System Matrices
A=[zeros(2) eye(2);-inv(M)*K -inv(M)*G];
B=[0;0;inv(M)*F];
C=[1 0 0 0;
   0 1 0 0];
D=[0;0];

% Observer based state-feedback controller
%Feedback controller design
K_c=place(A,B,[-8 -5 -6 -1]);
%Observer design
L_o=place(A',C',5*[-8 -5 -6 -1])';

%% LQR controller design
%Process Noise for Kalman Filter
W=10e-3*B*B';
%Sensor Noise for Kalman Filter
V=10e-3*eye(2);
%System for Kalman filter design
sys=ss(A,[B eye(4)],C,[zeros(2,1) zeros(2,4)]);
%Kalman filter based Observer design
[kest,L_q,P] = kalman(sys,W,V,zeros(4,2));

q_u=1;% control weight
q_y=100;% output weight
Q=q_y*(C'*C+eye(4));
R_p=q_u;
%Controller Design using LQR
[X,K,Leig] = icare(A,B,Q,R_p,zeros(4,1),eye(4),zeros(4));

K_final=K;