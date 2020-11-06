%% Open file from specified folder
uiopen('C:\Users\aaron\Documents\2020s2\ECE4095\1Current\Final\Current\ThreePhaseVectorControl.slx');

%% Set variables
clear; clc; close all;

% System values
Vdc = 2000;     % DC-link voltage
wn = 3000;      % Cut-off frequency for 2nd order LPF
zeta=0.707;     % Damping ratio for 2nd order LPF
Rf = 0.1;       % Line filter resistance
Lf = 0.005;     % Line filter inductance
Pn = 8e4;       % Nominal power of VSC
XR = 4.5;       % X/R ratio
SCR = 1;        % Grid SCR
Rg = sqrt((1/(1+XR^2))*((500^2)/(Pn*SCR))^2);       % Grid resistance
Lg = XR*Rg/(100*pi);                                % Grid inductance

% PQ Control
% Kp_PQ = 20;
% Ki_PQ = 0.001;

% Static Power Limitation
%Rg_eq = Rg/4;      % Refer values to transformer primary side
%Lg_eq = Lg/4;      % Refer values to transformer primary side
Rt = Rf + Rg;  
Lt = Lf + Lg;
Zt = sqrt(Rt^2 + (2*pi*50*Lt)^2);
phi = atan((2*pi*50*Lt)/Rt);
p_lim = (500*500 - 500^2*cos(phi))/Zt;

% VSC
Kp_VSC = 500;   
wL = 2*pi*50*Lf;

% PLL
Kp_PLL = 0.05;      % Slow = 0.1, fast = 1 
Ki_PLL = 0.005;         

% MPM
MPM_Ts = 1/5000;    % Timestep
Buffer = 500;       % Buffer size (no. of samples)
Overlap = Buffer - 1;   % Frequency of computation, (Buffer - 1) = compute every sample

% KF
% R = 0; %2e8;          % Measurement error covariance matrix
% Q_KF = 0.05*eye(2);   % Error covariance matrix %0.003

% CF
%  wc = 2*pi*10; 
 
% CF2
%  wc1 = 2*pi*17.5;
%  wc2 = 2*pi*1500;
%  wb = (wc2 + wc1)/2;
%  Q = 0.1;

%% Run
sim('ThreePhaseVectorControl.slx')
