function [tau2, T2_1] = torqueLink2_vFinal(theta3, theta2, theta1, Tmag, COM_L2, ...
    P0_1, P1_2, P2_2, P3_3, L0, L1, L2, g, m2, R)

    % origenes
    O0 = [0; 0];
    O1 = O0 + L0;
    O2 = O1 + R(theta1) * L1;
    O3 = O2 + R(theta1 + theta2) * L2;

    % coordenadas globales
    P0 = O1 + R(theta1) * P0_1;
    P1 = O2 + R(theta1 + theta2) * P1_2;
    P2 = O2 + R(theta1 + theta2) * P2_2;
    P3 = O3 + R(theta1 + theta2 + theta3) * P3_3;

    % P0_2  = R(theta1 + theta2)*(P0_1 - (O2 - O1));
    % P2_3  = R(theta1 + theta2 + theta3)*(P2_2 - (O3 - O2));
    % 
    % T2_3 = Tmag * (P3_3 - P2_3)/norm(P3_3 - P2_3);
    % tauT2_3 = cross([P2_3; 0;], [T2_3; 0;]);
    % 
    % T2_1 = Tmag * (P0_2 - P1_2)/norm(P0_2 - P1_2);
    % tauT2_1 = cross([P1_2; 0;], [T2_1; 0;]);
    % 
    % m2g_2 = R(theta2 + theta1) * [0; -m2 * g];
    % tau_m2g_2 = cross([COM_L2; 0], [m2g_2; 0]);
    % 
    % tau2 = tauT2_1(3) + tau_m2g_2(3) + tauT2_3(3);


    % Compute tendon T2_1 (from O1 side pulling to O2)
    dir_T2_1 = (P0 - P1);
    T2_1 = Tmag * dir_T2_1 / norm(dir_T2_1);
    tauT2_1 = cross([P1 - O2; 0], [T2_1; 0]);  % Torque at O2

    % Compute tendon T2_3 (pulling from link 3 back to link 2)
    dir_T2_3 = (P3 - P2);
    T2_3 = Tmag * dir_T2_3 / norm(dir_T2_3);
    tauT2_3 = cross([P2 - O2; 0], [T2_3; 0]);  % Torque at O2

    % Gravity torque
    COM_L2_global = O2 + R(theta1 + theta2) * COM_L2;
    m2g = [0; -m2 * g];
    tau_m2g_2 = cross([COM_L2_global - O2; 0], [m2g; 0]);

    % Net torque about joint-2
    tau2 = tauT2_1(3) + tau_m2g_2(3) + tauT2_3(3);

end