function [TAU_c] = PropModel(CurrentState,ControlInputs,PropParams)

u = CurrentState(4); %surge
v = CurrentState(5); %sway
r = CurrentState(6); %yaw rate

n_AT = ControlInputs(1); %revolution rate azimuth thruster        
alpha = ControlInputs(2); %azimuth angle azimuth thruster      
n_BT = ControlInputs(3); %revolution rate bow thruster    

% propeller parameters of azimuth thruster 
a1=PropParams(1);
b1=PropParams(2);


% propeller parameters of bow thruster 
c1=PropParams(5);
cb=PropParams(7);

L_AT=PropParams(8); % Dist to origin of BF-Frame in x^b direction of azimuth thruster 
L_BT=PropParams(9); % Dist to origin of BF-Frame in x^b direction of  bow thruster 

% calc axial velocity of azimuth thruster 
u_a=u*cos(alpha)+(v-r*L_AT)*sin(alpha);

% Thrust azimuth thruster
F_AT = PropellerThrust(n_AT,u_a,a1,b1);

% Thrust bow thruster
F_BT = PropellerThrustBowThruster(n_BT,u,c1,cb);

TAU_c = [ F_AT*cos(alpha);
 F_AT*sin(alpha)+F_BT;
 F_BT*L_BT-F_AT*sin(alpha)*L_AT;];


function f = PropellerThrust(n,u_a,a1,b1)
    c_AT = a1;
    d_AT = b1;
    f = c_AT*n*abs(n)-d_AT*u_a*abs(n);  
end

function f = PropellerThrustBowThruster(n,u,c1,cb)
    c_BT = c1;
    d_BT = cb;
    f = c_BT*n*abs(n)*exp(-d_BT*u^2);
end

end