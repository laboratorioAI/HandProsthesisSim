function [tau1, Tnot] = torqueLink1_thumb(theta1, COM_L1, m1, g, torqueMotor, radioPuley, radioProximalThumb, R, specialTransformation)
    % === Fixed transformation from global to proximal frame (UCS1) ===
    R_fixed = [0, 0, -1;
               0, 1, 0;
              1, 0, 0];

    R_1to2 = [specialTransformation.XX, specialTransformation.YX, specialTransformation.ZX;
              specialTransformation.XY, specialTransformation.YY, specialTransformation.ZY;
              specialTransformation.XZ, specialTransformation.YZ, specialTransformation.ZZ];
    
    t_1to2 = [specialTransformation.Trans_x;
              specialTransformation.Trans_y;
              specialTransformation.Trans_z];

    % ---- Orientation chain ----------------------------------------------
    R_0to1_base   = R_fixed;           % proximal at zero angle
    R_0to1        = R_0to1_base * R(theta1);
    
    R_0to2_base   = R_0to1 * R_1to2;   % middle at zero θ₂
    % R_0to2        = R_0to2_base * R(theta2);
    
    % R_0to3        = R_0to2 * R(theta3);  % distal after θ₃

    % === Joint Origins ===
    O0 = [0; 0; 0];
    O1 = O0;
    O2 = O1 + R_0to1 * t_1to2;
    % O3 = O2 + R_0to2 * obj.L2;
    % === Gravity force vector in global coordinates ===
    Fg = [0; -m1 * g; 0];
    
    % === Center of Mass in global coordinates ===
    COM_L1_global = O1 + R_0to1 * COM_L1;     % COM_L1 is local (proximal body)
    
    r_COM_wrt_O0 = COM_L1_global - O0;   % vector from O0 to COM

    % === Gravitational torque at joint-1 ===
    tau_gravity = cross(r_COM_wrt_O0, Fg);
    
    % === Motor torque effect scaled by gear/pulley ratio ===
    tau1_motor = torqueMotor * radioProximalThumb / radioPuley;
    
    % === Net torque at joint-1 (Z component only) ===
    tau1 = tau1_motor + tau_gravity(3);
    
    % === Placeholder torque (used for symbolic integration or tracking) ===
    Tnot = 0;

end