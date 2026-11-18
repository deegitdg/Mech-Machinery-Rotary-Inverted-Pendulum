
%% DC motor + driver (SI units)
kt = 0.035;      % torque constant   [N*m/A]
km = 0.035;      % back-EMF constant [V*s/rad]  (in SI units km must equal kt)
R  = 7.14;        % armature resistance [ohm]
L  = 264e-6;    % armature inductance [H]
N  = 1;          % gearbox ratio 
V_max = 24;      % driver voltage limit [V]

%% Geometry / mass (lengths in cm, mass in kg) - same values as Symbolic_math.m
base_size  = 9.7;    % base cube size
rod_rad    = 0.3;    % arm radius
rod_length = 11.219; % arm length
rod_mass   = 0.05;   % encoder body on the arm
link_mass  = 0.08;    % arm link body (was hard-coded 0.1 inside the model)
p_rad      = 0.3;    % pendulum radius
p_length   = 10;      % pendulum length
p_mass     = 0.1;    % pendulum mass
% CAD assembly offsets (cm) - these only position the STEP parts so they touch.
% They match the CAD (base 7 cm, arm part 8.5 cm); do NOT tie them to the physical lengths above.
rt1 = 7/2;  rt2 = 8.5-0.4;  rt4 = p_length/2;

%% Joint friction (viscous). Measure on hardware with a free-swing log decrement:
%  delta=(1/n)*log(A1/An); zeta=delta/sqrt(4*pi^2+delta^2); D_p=2*zeta*sqrt(J_p*m_p*g*L_p/2)
D_r = 0.0015/2;  % arm bearing      [N*m*s/rad]
D_p = 2e-5;      % pendulum bearing [N*m*s/rad]

%% Initial pendulum angle and encoder calibration
alpha_0   = 170;  % deg, 180 = upright, 0 = hanging
alpha_cal = -3.678;    % deg, CAD-frame offset so hanging reads 0 and upright reads 180

%% Linear model at upright [0 pi 0 0] for controller design
g   = 9.81;
L_r = rod_length/100;  L_p = p_length/100;  m_p = p_mass;  m_r = rod_mass+link_mass;
J_r = (1/3)*m_r*L_r^2;          % arm about the motor axis
J_p = (1/3)*m_p*L_p^2;          % pendulum about its own pivot (no double counting)
M  = [m_p*L_r^2+J_r,       -0.5*m_p*L_p*L_r;
      -0.5*m_p*L_p*L_r,     J_p];               % symmetric mass matrix
G  = [D_r+N^2*kt*km/R 0; 0 D_p];               % viscous + back-EMF damping
Kg = [0 0; 0 -0.5*m_p*L_p*g];                  % gravity stiffness (unstable at upright)
F  = [N*kt/R; 0];                              % volts -> arm torque
A_hand = [zeros(2) eye(2); -M\Kg -M\G];
B_hand = [0; 0; M\F];
linfile = fullfile(fileparts(mfilename('fullpath')),'Rotary_inverted_pendulum_control_.mat');
if isfile(linfile)
    S = load(linfile);  A = S.A;  B = S.B;     % identified from Simscape
else
    A = A_hand;  B = B_hand;                    % fallback
end
C = [1 0 0 0; 0 1 0 0];
D = [0; 0];

%% Observer-based state feedback by pole placement (used by the commented-out blocks)
K_c = place(A,B,[-8 -5 -6 -1]);
L_o = place(A',C',5*[-8 -5 -6 -1])';

%% LQR + Kalman filter (used by Controller1)
W   = 10e-3*(B*B');           % process noise
V   = 10e-3*eye(2);           % sensor noise
sys = ss(A,[B eye(4)],C,[zeros(2,1) zeros(2,4)]);
[kest,L_q,P] = kalman(sys,W,V,zeros(4,2));
Q   = 100*(C'*C+eye(4));
R_p = 1;
[X,K,Leig] = icare(A,B,Q,R_p,zeros(4,1),eye(4),zeros(4));
K_final = K;
