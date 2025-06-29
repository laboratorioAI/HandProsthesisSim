% function [tau3, T3_2] = torqueLink3_thumb(theta3, theta2, theta1, Tmag, COM_L3, ...
%     P1_2, P2_2, P3_3, L0, L1, L2, g, m3, R, specialTransformation)
% 
%     % === Local 2D Kinematic Setup (for tendon forces) ===
%     L_PO2 = [0.018; -0.003];  % O2 position in local 2D layout
%     O2 = [0; 0] + L_PO2;
% 
%     rot_th2 = R(theta2);
%     O3 = O2 + rot_th2(1:2, 1:2) * L2(1:2);
% 
%     % Tendon-related points in local 2D layout
%     P0 = [0; 0];
%     P1 = O2 + rot_th2(1:2, 1:2) * P1_2(1:2);
%     P2 = O2 + rot_th2(1:2, 1:2) * P2_2(1:2);
%     rot_th23 = R(theta2 + theta3);
%     P3 = O3 + rot_th23(1:2,1:2) * P3_3(1:2);
% 
%     % === Tendon Force and Torque at O3 ===
%     dir_T3_2 = P2 - P3;
%     T3_2 = Tmag * dir_T3_2 / norm(dir_T3_2);
%     tauT3_2 = cross([P3 - O3; 0], [T3_2; 0]);  % Torque in Z from tendon
% 
%     % === GLOBAL Gravity Torque Setup ===
%     % 1. Full 3D rotation chain (global)
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
%     R_0to3 = R_0to2 * R(theta3);
% 
%     % 2. Joint origins in global frame
%     O0 = [0; 0; 0];
%     O2_global = O0 + R_0to1 * t_1to2;
%     O3_global = O2_global + R_0to2 * L2;
% 
%     % 3. COM in global coordinates
%     COM_L3_global = O3_global + R_0to3 * COM_L3;
% 
%     % 4. Gravity torque
%     m3g = [0; -m3 * g; 0];  % Gravity in global Y
%     r_COM_O3 = COM_L3_global - O3_global;
%     tau_m3g_3_global = cross(r_COM_O3, m3g);
% 
%     % 5. Project onto Z axis
%     tau_m3g_3 = tau_m3g_3_global(3);
% 
%     % === Total Torque at Joint-3 ===
%     tau3 = tauT3_2(3) + tau_m3g_3;
% end


function [tau3, T3_2] = torqueLink3_thumb(theta3, theta2, theta1, Tmag, COM_L3, P2_2, P3_3, L0, L1, L2, ...
    g, m3, R, specialTransformation)
    %---------------------------------------------------------------
    % theta  : [theta1; theta2; theta3]  (rad)
    % T32mag : tension magnitude (scalar, N)
    %
    % returns tau3 : scalar torque about z at joint‑3  (N·m)
    %---------------------------------------------------------------

    %theta3;                 % orientation of link‑3d
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
    % --- Compute world coordinates of relevant tendon/force points ---

    P2 = O2 + R_0to2 * P2_2;                  % Tendon guide point on link 2
    P3 = O3 + R_0to3 * P3_3;

    % Compute tendon force T3_2 (from guide P2 to tendon attach P3)
    dir_T3_2 = P2 - P3;
    T3_2 = Tmag * dir_T3_2 / norm(dir_T3_2);

    % Torque of T3_2 at joint-3 (O3)
    tauT3_2 = cross(P3 - O3, T3_2);

    % Gravity force on link-3 (in global frame)
    COM_L3_global = O3 + R_0to3 * COM_L3;
    m3g = [0; -m3 * g; 0];

    % Torque from gravity at joint-3 (O3)
    tau_m3g_3 = cross(COM_L3_global - O3, m3g);

    % Net torque about joint-3
    % tau3 = tauT3_2(3) + tau_m3g_3(3);

    z3 = R_0to3(:,3);  % Local z-axis of joint-3 in global coordinates

    tau3 = dot(tauT3_2 + tau_m3g_3, z3);  % Scalar torque about joint-3
end

