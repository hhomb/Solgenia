function [dState] = ShipModel(CurrentState,Forces,SeaCurrent,DynamicParameters)
    % 3 degree of freedom dynamic shipmodell
    
    %% State independent parameter
    % Distance Origin of BF Frame to Center of Gravity in x_BF
    % direction
    xg = DynamicParameters(1);

    % Rigid body mass
    Displacement = DynamicParameters(2); 

    % Moment of inertia
    J_comb =  DynamicParameters(3);

    % Added mass translation
    X_du = DynamicParameters(4);
    Y_dv = DynamicParameters(5);  
    N_dv = DynamicParameters(6);

    % Added mass rotation
    Y_dr = DynamicParameters(7);
    
    % Linear damping
    X_u = DynamicParameters(8);   
    Y_v = DynamicParameters(9);
    N_v = DynamicParameters(10);
    N_r = DynamicParameters(11);
    Y_r = DynamicParameters(12);

    % Nonlinear Damping
    X_uu = DynamicParameters(13);      
    Y_vv = DynamicParameters(14);      
    N_vv = DynamicParameters(15);
    Y_rr = DynamicParameters(16);
    N_rr = DynamicParameters(17);
    
    X_rr = DynamicParameters(18);
    X_vr = DynamicParameters(19);
    Y_uv = DynamicParameters(20);
    Y_ur = DynamicParameters(21);
    N_ur = DynamicParameters(22);
    N_uv = DynamicParameters(23);
    Y_vr = DynamicParameters(24);
    Y_rv = DynamicParameters(25);
    N_vr = DynamicParameters(26);
    N_rv = DynamicParameters(27);
    
    
    %% State decomposition
    eta0 = CurrentState(1:3); % position [east, north, yaw]
    nue0_r = CurrentState(4:6); % relative (to water) body fix velocity [surge, sway, yawrate]

    
    %% Build systemmatrizes
    
    % Matrix of inertia
    MRB = [Displacement, 0 , 0;
           0, Displacement, Displacement*xg; 
           0, Displacement*xg, J_comb];
    
    % Added mass matrix
    MA = -[X_du, 0, 0;
           0, Y_dv, Y_dr;
           0, N_dv, 0];
    
    % Combined mass matrix   
    M = MRB + MA;    
       
    % Rigid body Coriolis Matrix  
    CRB=[0,-Displacement*nue0_r(3),-Displacement*xg*nue0_r(3);
        Displacement*nue0_r(3),0,0;
        Displacement*xg*nue0_r(3),0,0];

    
    % Term accounts for added mass coriolis and centripetal effects and hydrodynmic damping
    Nnue=[-X_u*nue0_r(1)-X_uu*abs(nue0_r(1))*nue0_r(1)+X_vr*nue0_r(2)*nue0_r(3)+X_rr*nue0_r(3)*nue0_r(3);...
          -Y_v*nue0_r(2)-Y_r*nue0_r(3)-Y_vv*abs(nue0_r(2))*nue0_r(2)+Y_ur*nue0_r(1)*nue0_r(3)+Y_uv*nue0_r(1)*nue0_r(2)-Y_rr*abs(nue0_r(3))*nue0_r(3)-Y_vr*abs(nue0_r(2))*nue0_r(3)-Y_rv*abs(nue0_r(3))*nue0_r(2);...
          -N_r*nue0_r(3)-N_v*nue0_r(2)-N_rr*abs(nue0_r(3))*nue0_r(3)+N_uv*nue0_r(1)*nue0_r(2)+N_ur*nue0_r(1)*nue0_r(3)-N_vv*abs(nue0_r(2))*nue0_r(2)-N_vr*abs(nue0_r(2))*nue0_r(3)-N_rv*abs(nue0_r(3))*nue0_r(2)];


    
    %% Control forces
    TAU = Forces;
    
     %% Calc derivatives
    
    % dnue_relative
    dnue_r = M\(TAU -CRB*nue0_r - Nnue);
    
    % dPose
    deta = RotationMatrix2dYaw(eta0(3))*nue0_r+[SeaCurrent(1);SeaCurrent(2);0]; 
    
    % dx
    dState = [deta;dnue_r];
    
    function R = RotationMatrix2dYaw(psi)
        cy = cos(psi); sy = sin(psi);
        R = [cy,-sy,0; sy, cy,0; 0,0,1];
    end
end