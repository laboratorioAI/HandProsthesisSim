function [tau3, T3_2] = torqueLink3_vFinal(theta3, theta2, theta1, Tmag, COM_L3, P2_2, P3_3, L0, L1, L2, ...
    g, m3, R)
    %---------------------------------------------------------------
    % theta  : [theta1; theta2; theta3]  (rad)
    % T32mag : tension magnitude (scalar, N)
    %
    % returns tau3 : scalar torque about z at joint‑3  (N·m)
    %---------------------------------------------------------------
    
    %theta3;                 % orientation of link‑3
    O0 = [0; 0];
    O1 = O0 + L0;
    O2 = O1 + R(theta1) * L1;
    O3 = O2 + R(theta1 + theta2) * L2;

    % P0 = O1 + R(theta1) * P0_1;
    % P1 = O2 + R(theta1 + theta2) * P1_2;
    P2 = O2 + R(theta1 + theta2) * P2_2;
    P3 = O3 + R(theta1 + theta2 + theta3) * P3_3;
    
    % % tendon guide on link‑2 (example: 3 mm behind O3 along link‑2):
    % P2_3  = R(theta3)*(P2_2 - (O3 - O2));      % ← ✅ EDIT to match your pulley
    % 
    % % ---------- tendon force & its torque ----------------------------
    % T3_2 = Tmag * (P2_3 - P3_3)/norm(P2_3 - P3_3);
    % tauT3_2 = cross([P3_3; 0;], [T3_2; 0;]);
    % 
    % m3g_3 = R(theta1 + theta2 + theta3) * [0; -m3 * g];
    % tau_m3g_3 = cross([COM_L3; 0], [m3g_3;0 ]);
    % 
    % tau3 = tau_m3g_3(3) + tauT3_2(3);

    % Compute tendon force T3_2 (from guide P2 to tendon attach P3)
    dir_T3_2 = P2 - P3;
    T3_2 = Tmag * dir_T3_2 / norm(dir_T3_2);

    % Torque of T3_2 at joint-3 (O3)
    tauT3_2 = cross([P3 - O3; 0], [T3_2; 0]);

    % Gravity force on link-3 (in global frame)
    COM_L3_global = O3 + R(theta1 + theta2 + theta3) * COM_L3;
    m3g = [0; -m3 * g];

    % Torque from gravity at joint-3 (O3)
    tau_m3g_3 = cross([COM_L3_global - O3; 0], [m3g; 0]);

    % Net torque about joint-3
    tau3 = tauT3_2(3) + tau_m3g_3(3);

end
