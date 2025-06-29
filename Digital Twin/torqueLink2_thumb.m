% function [tau2, T2_1] = torqueLink2_thumb(theta3, theta2, theta1, Tmag, COM_L2, ...
%     P0_1, P1_2, P2_2, P3_3, L0, L1, L2, g, m2, R, specialTransformation)
% 
%     % === 2D Tendon Geometry Setup ===
%     L_PO2 = [0.018; -0.003];  % O2 local base
%     O2 = [0; 0] + L_PO2;
% 
%     rot_th2 = R(theta2);
%     O3 = O2 + rot_th2(1:2, 1:2) * L2(1:2);
% 
%     % Tendon-related points in local 2D
%     P0 = [0; 0];
%     P1 = O2 + rot_th2(1:2, 1:2) * P1_2(1:2);
%     P2 = O2 + rot_th2(1:2, 1:2) * P2_2(1:2);
%     rot_th23 = R(theta2 + theta3);
%     P3 = O3 + rot_th23(1:2, 1:2) * P3_3(1:2);
% 
%     % Tendon torque contributions
%     dir_T2_1 = P0 - P1;
%     T2_1 = Tmag * dir_T2_1 / norm(dir_T2_1);
%     tauT2_1 = cross([P1 - O2; 0], [T2_1; 0]);
% 
%     dir_T2_3 = P3 - P2;
%     T2_3 = Tmag * dir_T2_3 / norm(dir_T2_3);
%     tauT2_3 = cross([P2 - O2; 0], [T2_3; 0]);
% 
%     % === Global Gravity Torque Setup ===
%     R_fixed = [0, 0, -1;
%                0, 1, 0;
%                1, 0, 0];
% 
%     R_1to2 = [specialTransformation.XX, specialTransformation.YX, specialTransformation.ZX;
%               specialTransformation.XY, specialTransformation.YY, specialTransformation.ZY;
%               specialTransformation.XZ, specialTransformation.YZ, specialTransformation.ZZ];
% 
%     t_1to2 = [specialTransformation.Trans_x;
%               specialTransformation.Trans_y;
%               specialTransformation.Trans_z];
% 
%     R_0to1 = R_fixed * R(theta1);
%     R_0to2 = R_0to1 * R_1to2 * R(theta2);
% 
%     O0 = [0; 0; 0];
%     O2_global = O0 + R_0to1 * t_1to2;
%     COM_L2_global = O2_global + R_0to2 * COM_L2;
% 
%     m2g = [0; -m2 * g; 0];
%     r_COM_O2 = COM_L2_global - O2_global;
%     tau_m2g_2_global = cross(r_COM_O2, m2g);
% 
%     % === Net Torque at Joint-2 ===
%     tau2 = tauT2_1(3) + tauT2_3(3) + tau_m2g_2_global(3);
% 
% end


function [tau2, T2_1] = torqueLink2_thumb(theta3, theta2, theta1, Tmag, COM_L2, ...
    P0_1, P1_2, P2_2, P3_3, L0, L1, L2, g, m2, R, specialTransformation)

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
    R_0to2        = R_0to2_base * R(theta2);

    R_0to3        = R_0to2 * R(theta3);  % distal after θ₃

    % === Joint Origins ===
    O0 = [0; 0; 0];
    O1 = O0;
    O2 = O1 + R_0to1 * t_1to2;
    O3 = O2 + R_0to2 * L2;
    % === Attachment and guide points in global frame ===
    P0 = O1 + R_0to1 * P0_1;                      % Proximal anchor (global)
    P1 = O2 + R_0to2 * P1_2;                      % Tendon guide near O2
    P2 = O2 + R_0to2 * P2_2;                      % Link-2 attach
    P3 = O3 + R_0to3 * P3_3;                      % Link-3 attach

    % === Tendon forces and their torques ===
    dir_T2_1 = P0 - P1;
    T2_1 = Tmag * dir_T2_1 / norm(dir_T2_1);
    tauT2_1 = cross(P1 - O2, T2_1);                   % Torque from base side

    dir_T2_3 = P3 - P2;
    T2_3 = Tmag * dir_T2_3 / norm(dir_T2_3);
    tauT2_3 = cross(P2 - O2, T2_3);                   % Torque from distal side

    % === Gravity torque on link-2 ===
    COM_L2_global = O2 + R_0to2 * COM_L2;         % COM in global coordinates
    m2g = [0; -m2 * g; 0];                            % Gravity in global frame
    tau_m2g_2 = cross(COM_L2_global - O2, m2g);       % Torque due to gravity

    % === Final torque at joint-2 (projected about Z) ===
    % tau2 = tauT2_1(3) + tauT2_3(3) + tau_m2g_2(3);

    z2 = R_0to2(:,3);  % local z-axis of joint-2 in global frame
    tau2 = dot(tauT2_1 + tauT2_3 + tau_m2g_2, z2);  % scalar projection onto z2



end