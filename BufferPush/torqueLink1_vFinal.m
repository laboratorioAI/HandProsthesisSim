function [tau1, T1_0] = torqueLink1_vFinal(theta2, theta1, Tmag, ...
    COM_L1, AP_0, P_1, P0_1, P1_2, P2_2, P3_3, L0, L1, L2, g, m1, R)
    
    O0 = [0; 0];
    O1 = O0 + L0;
    O2 = O1 + R(theta1) * L1;
    O3 = O2 + R(theta1 + theta2) * L2;

    P0 = O1 + R(theta1) * P0_1;
    P1 = O2 + R(theta1 + theta2) * P1_2;
    % P2 = O2 + R(theta1 + theta2) * P2_2;
    % P3 = O3 + R(theta1 + theta2 + theta3) * P3_3;


    % AP_1 = R(theta1)*(AP_0 - (O1 - O0));
    % P0_2 = R(theta1 + theta2)*(P0_1 - (O2 - O1));
    % 
    % T1_2 = Tmag * (P1_2 - P0_2)/norm(P1_2 - P0_2);
    % tauT1_2 = cross([P0_2; 0], [T1_2; 0]);
    % 
    % T1_0 = Tmag * (AP_1 - P_1)/norm(AP_1 - P_1);
    % tauT1_0 = cross([P_1; 0], [T1_0; 0]);
    % 
    % m1g_1 = R(theta1) * [0; -m1 * g];
    % tau_m1g_1 = cross([COM_L1; 0], [m1g_1; 0]);
    % 
    % tau1 = tauT1_2(3) + tauT1_0(3) + tau_m1g_1(3);


    % Compute T1_0: pulling from anchor AP_0 to point P_1 (in global)
    dir_T1_0 = AP_0 - P1;
    T1_0 = Tmag * dir_T1_0 / norm(dir_T1_0);
    tauT1_0 = cross([P1 - O1; 0], [T1_0; 0]);  % torque about O1

    % Compute T1_2: pulling from link-2 back to link-1
    dir_T1_2 = P1 - P0;
    T1_2 = Tmag * dir_T1_2 / norm(dir_T1_2);
    tauT1_2 = cross([P0 - O1; 0], [T1_2; 0]);

    % Gravity torque
    COM_L1_global = O1 + R(theta1) * COM_L1;
    m1g = [0; -m1 * g];
    tau_m1g_1 = cross([COM_L1_global - O1; 0], [m1g; 0]);

    % Total torque
    tau1 = tauT1_0(3) + tauT1_2(3) + tau_m1g_1(3);

end