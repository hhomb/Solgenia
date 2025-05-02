%% Minimum example to simulate the solgenia model %%
close 
clear all 
clc

%Parameters
N  = 100; % number of timesteps [-]
h  = 0.1; % discretization time [s]

%initial state
x0 = [0;  % local x-position [m]
      10; % local y-position [m]
       15/8*pi; % yaw angle to the x-axis [rad] 
       1; % surge velo [m/s]
       0; % sway velo [m/s]
       0];% yaw angle velocity [rad/s]
nx = 6; %dimension   

% control
u =   [ 1500/60; % turn rate azimuth thruster [Hz]
        pi/16; % orientation angle azimuth thruster [rad]
        -2000/60]; % turn rate bow thruster [Hz]
nu = 3; %dimension

% disturbance
nd = 3; %dimension sea current [m/s]

% load parameters
p  = ParametersSolgenia(); % Load model parameters
 
% alloc memory
U = zeros(nu,N);   % control trajectory
X = zeros(nx,N+1); % state trajectory
D = zeros(nd,N);   % disturbance trajectory

X(:,1) = x0; % determine init state 

% Simulate and visualize ship trajectory
for k = 1:N
    U(:,k) = u;
    X(:,k+1) = RK4_step(X(:,k),U(:,k),p,D(:,k),h);
    t = k*h; % simulation time
    visuSolgenia(X(:,k),U(:,k),t); 
    pause(h); % real time
end


