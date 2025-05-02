function p = ParametersSolgenia()
%% Thruster Location
%Distance of Az thruster in negativ x^b-direction 
L_AT=2.9;
%Distance of bow thrusters in positv x^b-direction 
L_BT=3.7;
ThrusterLocationSolgenia=[L_AT,L_BT];

%% Leverarm
LeverarmSolgenia=[0,0,1.5];


%% Propeller Parameter
Dp=0.36; %Propeller diameter
rho=1000; %Water density

%Attention! n in [Hz] and u in [m/s]
a1 = 0.63; %0.42
b1 = 0.0;  %0.38
a2 = 0.63;
b2 = 0.0;


c1=0.055;
c2=0.055;
cb=0.62;

PropellerParameter=[a1,b1,a2,b2,c1,c2,cb,ThrusterLocationSolgenia,LeverarmSolgenia]'; 

%% Actuator Dynamics Parameter
T_Az=0.2;
T_alpha=0.1;
T_Bow=0.3;

%% Actuator Saturation Parameter
domega_Az_max=4500/60; %Max Azimuth thruster Rev Rate Acc [Hz/s]
nAzMax=2000/60; %Max Azimuth thruster Rev Rate [Hz]
omega_alpha_max=85/180*pi; %Max panning rate of azimuth thruster [rad/s]
nBowMax=4000/60; %Max bow thruster Rev Rate [Hz]
                     
%% Paramter Solgenia 
DynamicParameters=zeros(27,1);

% Distance Antenna to CG in x_BF direction  
DynamicParameters(1) = 0; %xg

% Rigid body mass
DynamicParameters(2) = 3100; %Displacement

% Moment of inertia including added mass 
DynamicParameters(3) = 21179;    %J_comb

% Added mass 
DynamicParameters(4) = -155.42;      %X_du
DynamicParameters(5) = -1070;           %Y_dv
DynamicParameters(6) = -3328;           %N_dv
DynamicParameters(7) = -1008;           %Y_dr

% Linear damping
DynamicParameters(8) = -84.01;               %X_u
DynamicParameters(9) = -795.58;                %Y_v
DynamicParameters(10) = -958.4;               %N_v
DynamicParameters(11) = -5319.88;              %N_r
DynamicParameters(12) = -896.11;               %Y_r

% Nonlinear Damping
DynamicParameters(13) = -46.73;          %X_uu      
DynamicParameters(14) = 0;          %Y_vv
DynamicParameters(15) = -234.94;          %N_vv
DynamicParameters(16) = -149.38;          %Y_rr
DynamicParameters(17) = 0;          %N_rr
DynamicParameters(18) = -312.21;          %X_rr
DynamicParameters(19) = 434.41;          %X_vr
DynamicParameters(20) = 395;          %Y_uv
DynamicParameters(21) = -368.2;          %Y_ur
DynamicParameters(22) = 392.76;          %N_ur
DynamicParameters(23) = -138.5;          %N_uv
DynamicParameters(24) = 70.39;          %Y_vr
DynamicParameters(25) = 24.09;          %Y_rv
DynamicParameters(26) = -85.67;          %N_vr 
DynamicParameters(27) = 14.09;          %N_rv

% Combined parameter vector
p=[DynamicParameters;PropellerParameter];% #27 Dynamic Ship Parameters #12 Propeller Parameters 