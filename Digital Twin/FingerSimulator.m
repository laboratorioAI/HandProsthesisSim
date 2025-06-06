classdef FingerSimulator
    properties
        % === Raw configuration ===
        config                          % Original JSON structure loaded as struct
        bufferManager
        dataAccumulator
        digitalTwinVisual logical = false
    
        % === Time settings ===
        t0 double                       % Initial time
        tf double                       % Final time
        dt double                       % Time step
        time double                  % Time vector
        N double                        % Number of simulation steps
    
        % === Torque and simulation parameters ===
        Tmag function_handle            % Function handle for Tmag(t)
        thetaMin double                 % Lower joint angle limit [rad]
        thetaMax double                 % Upper joint angle limit [rad]
        desiredFPS double              % Desired simulation FPS
        g double                        % Gravity acceleration
        R function_handle               % 2D rotation matrix function
    
        % === Geometry & kinematics ===
        L0 double                       % Base phalanx vector [2x1]
        AP double                       % Fixed anchor point of tendon [2x1]
    
        % === Key points (all as 2x1 column vectors) ===
        P0 double
        P double
        P1 double
        P2 double
        P3 double
        PFin double
    
        % === Segment lengths ===
        L1 double
        L2 double
        L3 double
    
        % === Masses ===
        m1 double
        m2 double
        m3 double
    
        % === Centers of mass ===
        COM_L1 double
        COM_L2 double
        COM_L3 double
    
        % === Moments of inertia (around z) ===
        I1z double
        I2z double
        I3z double
    
        % === fingers initial angles ===
        theta1_init double
        theta2_init double
        theta3_init double
    
        % === fingers initial angles velocities ===
        omega1_init double
        omega2_init double
        omega3_init double
    
        % === fingers initial angles aceleration ===
        alpha1_init double
        alpha2_init double
        alpha3_init double
    
        % === thetas ===
        theta1 double
        theta2 double
        theta3 double
    
        % === thetas ===
        omega1 double
        omega2 double
        omega3 double
    
        % === aphas ===
        alpha1 double
        alpha2 double
        alpha3 double
    
        % === Y's ===
        Y1 double
        Y2 double
        Y3 double
    
        % === Simulation results ===
        results struct = struct()       % Container for simulation outputs (Y1, Y2, Y3, alpha, etc.)
    end

    methods
        function obj = FingerSimulator(configFile)
            obj.config = obj.loadConfigFile(configFile);
            obj = obj.initializeParameters();
        end

        % Helper function to convert arrays to column vectors
        function v = columnVec(obj, arr)
            v = reshape(arr, [], 1);
        end

        function obj = initializeParameters(obj)
            c = obj.config;
            obj.t0 = c.time.t0;
            obj.tf = c.time.tf;
            obj.dt = c.time.dt;
            %obj.time.vec = c.time.t0:c.time.dt:c.time.tf;
            % obj.time.N = numel(obj.time.vec);
            obj.time = obj.t0:obj.dt:obj.tf;
            obj.N = numel(obj.time);

            % obj.Tmag = @(t) 160 * sin(pi * 0.05 * t);
            obj.Tmag = @(t) 160 * sin(pi * 0.05 * t);

            obj.thetaMin = deg2rad(c.thetaLimits.thetaMin_deg);
            obj.thetaMax = deg2rad(c.thetaLimits.thetaMax_deg);
            obj.desiredFPS = c.simulation.desiredFPS;
            obj.g = c.simulation.g;
            obj.R = @(phi)[cos(phi), -sin(phi); sin(phi), cos(phi)];

            % Geometry (converted to column vectors)
            obj.L0 = obj.columnVec(c.basePhalanx.L0);
            obj.AP = obj.columnVec(c.basePhalanx.AP);
        
            obj.P0 = obj.columnVec(c.points.P0);
            obj.P  = obj.columnVec(c.points.P);
            obj.P1 = obj.columnVec(c.points.P1);
            obj.P2 = obj.columnVec(c.points.P2);
            obj.P3 = obj.columnVec(c.points.P3);
            obj.PFin = obj.columnVec(c.points.PFin);
        
            obj.L1 = obj.columnVec(c.lengths_m.L1);
            obj.L2 = obj.columnVec(c.lengths_m.L2);
            obj.L3 = obj.columnVec(c.lengths_m.L3);
        
            obj.m1 = c.masses_kg.m1;
            obj.m2 = c.masses_kg.m2;
            obj.m3 = c.masses_kg.m3;
        
            obj.COM_L1 = obj.columnVec(c.centersOfMass_m.COM_L1);
            obj.COM_L2 = obj.columnVec(c.centersOfMass_m.COM_L2);
            obj.COM_L3 = obj.columnVec(c.centersOfMass_m.COM_L3);
        
            obj.I1z = c.inertias_kgm2.I1z;
            obj.I2z = c.inertias_kgm2.I2z;
            obj.I3z = c.inertias_kgm2.I3z;

            obj.theta1_init = c.thetaInit.theta1_init;
            obj.theta2_init = c.thetaInit.theta2_init;
            obj.theta3_init = c.thetaInit.theta3_init;

            obj.omega1_init = c.omegaInit.omega1_init;
            obj.omega2_init = c.omegaInit.omega2_init;
            obj.omega3_init = c.omegaInit.omega3_init;

            obj.alpha1_init = c.alphaInit.alpha1_init;
            obj.alpha2_init = c.alphaInit.alpha2_init;
            obj.alpha3_init = c.alphaInit.alpha3_init;
        end

        function tau_val = getFirstOutput(obj, theta3, theta2, theta1, t, tau, varargin)
            [tau_val, ~] = tau(theta3, theta2, theta1, t, varargin{:});  % Catch only the first output (torque)
        end

        function obj = runSimulation(obj, TMag, th_init1, th_init2, th_init3, o_init1, o_init2, o_init3, a_init1, a_init2, a_init3, N_steps, delta_t, phanlanxIds)
            % === Handle Optional Inputs ===
            if nargin < 2 || isempty(TMag),     TMag     = obj.Tmag; end

            % === Normalize TMag to always be a function handle ===
            if isnumeric(TMag)
                constVal = TMag;
                TMag = @(t) constVal;  % make it act like a function
            elseif ~isa(TMag, 'function_handle')
                error('TMag must be either a numeric constant or a function handle.');
            end
        
            if nargin < 3 || isempty(th_init1), th_init1 = obj.theta1_init; end
            if nargin < 4 || isempty(th_init2), th_init2 = obj.theta2_init; end
            if nargin < 5 || isempty(th_init3), th_init3 = obj.theta3_init; end
        
            if nargin < 6 || isempty(o_init1),  o_init1 = obj.omega1_init; end
            if nargin < 7 || isempty(o_init2),  o_init2 = obj.omega2_init; end
            if nargin < 8 || isempty(o_init3),  o_init3 = obj.omega3_init; end
        
            if nargin < 9 || isempty(a_init1),  a_init1 = obj.alpha1_init; end
            if nargin < 10 || isempty(a_init2), a_init2 = obj.alpha2_init; end
            if nargin < 11 || isempty(a_init3), a_init3 = obj.alpha3_init; end
        
            if nargin < 12 || isempty(N_steps), N_steps = obj.N; end
            if nargin < 13 || isempty(delta_t), delta_t = obj.dt; end
            %if nargin < 14 || isempty(time_vec), time_vec = 0:delta_t:(N_steps-1)*delta_t; end
        
            % === Apply simulation timing ===
            obj.N = N_steps;
            obj.dt = delta_t;
            obj.time = 0:delta_t:(N_steps-1)*delta_t;
        
            % === Set Initial Conditions ===
            obj.theta1_init = th_init1;
            obj.theta2_init = th_init2;
            obj.theta3_init = th_init3;

            obj.omega1_init = o_init1;
            obj.omega2_init = o_init2;
            obj.omega3_init = o_init3;

            obj.alpha1_init = a_init1;
            obj.alpha2_init = a_init2;
            obj.alpha3_init = a_init3;

            % === Torque functions ===
            tau1 = @(th3, th2, th1, t) torqueLink1_vFinal(th2, th1, TMag(t), obj.COM_L1, obj.AP, obj.P, obj.P0, obj.P1, obj.P2, obj.P3, obj.L0, obj.L1, obj.L2, obj.g, obj.m1, obj.R);
            tau2 = @(th3, th2, th1, t) torqueLink2_vFinal(th3, th2, th1, TMag(t), obj.COM_L2, obj.P0, obj.P1, obj.P2, obj.P3, obj.L0, obj.L1, obj.L2, obj.g, obj.m2, obj.R);
            tau3 = @(th3, th2, th1, t) torqueLink3_vFinal(th3, th2, th1, TMag(t), obj.COM_L3, obj.P2, obj.P3, obj.L0, obj.L1, obj.L2, obj.g, obj.m3, obj.R);

            getTorque1Only = @(th3, th2, th1, t) obj.getFirstOutput(th3, th2, th1, t, tau1);
            getTorque2Only = @(th3, th2, th1, t) obj.getFirstOutput(th3, th2, th1, t, tau2);
            getTorque3Only = @(th3, th2, th1, t) obj.getFirstOutput(th3, th2, th1, t, tau3);
        
            obj.Y1 = zeros(2, obj.N);
            obj.Y2 = zeros(2, obj.N);
            obj.Y3 = zeros(2, obj.N);

            obj.Y1(:,1) = [obj.theta1_init; obj.omega1_init];
            obj.Y2(:,1) = [obj.theta2_init; obj.omega2_init];
            obj.Y3(:,1) = [obj.theta3_init; obj.omega3_init];
        
            obj.alpha1 = zeros(1, obj.N);
            obj.alpha2 = zeros(1, obj.N);
            obj.alpha3 = zeros(1, obj.N);

            obj.alpha1(:, 1) = obj.alpha1_init;
            obj.alpha2(:, 1) = obj.alpha2_init;
            obj.alpha3(:, 1) = obj.alpha3_init;

            % === Inertia Calculation ===
        
            f1 = @(t, th_3, th_2, y, I) [ ...
                y(2);
                % getTorque1Only(th_3, th_2, y(1), t) / obj.I1z ...
                getTorque1Only(th_3, th_2, y(1), t) / I ...
            ];
        
            f2 = @(t, th_3, y, th_1, I) [ ...
                y(2);
                % getTorque2Only(th_3, y(1), th_1, t) / obj.I2z ...
                getTorque2Only(th_3, y(1), th_1, t) / I ...
            ];
        
            f3 = @(t, y, th_2, th_1, I) [ ...
                y(2);
                % getTorque3Only(y(1), th_2, th_1, t) / obj.I3z ...
                getTorque3Only(y(1), th_2, th_1, t) / I ...
            ];
        
            for k = 2:obj.N
                t = obj.time(k-1);
            
                % THIRD PHALANX
                y3 = obj.Y3(:,k-1);
                % SECOND PHALANX
                y2 = obj.Y2(:,k-1);
                % FIRST PHALANX
                y1 = obj.Y1(:,k-1);

                % === Finger Phalanx Data Gathering ===
                % proximalInertia, middleInertia, distalInertia = GetPhalanxInfo(phanlanxIds);
                [proxInertia, middInertia, distInertia] = obj.GetPhalanxsInertias(phanlanxIds, y1(1), y2(1), y3(1));
            
                % CALCULATIONS THIRD PHALANX
                k1 = f3(t             , y3              , y2(1), y1(1), distInertia);
                k2 = f3(t+obj.dt/2    , y3+obj.dt/2*k1  , y2(1), y1(1), distInertia);
                k3 = f3(t+obj.dt/2    , y3+obj.dt/2*k2  , y2(1), y1(1), distInertia);
                k4 = f3(t+obj.dt      , y3+obj.dt  *k3  , y2(1), y1(1), distInertia);
            
                y3Next = y3 + obj.dt/6*(k1 + 2*k2 + 2*k3 + k4);
            
                % ---------- HARD‑STOP LOGIC ---------------------------------
                [tau_now, T3_2] = tau3(y3Next(1), y2(1), 0, obj.time(k));                % torque at this step
            
                if y3Next(1) < obj.thetaMin
                    y3Next(1) = obj.thetaMin;
                elseif y3Next(1) > obj.thetaMax
                    y3Next(1) = obj.thetaMax;
                end
                
                if y3Next(1) == obj.thetaMin
                        y3Next(2) = 0;           %se pega al tope
                        obj.alpha3(k) = 0;
                elseif y3Next(1) == obj.thetaMax
                        y3Next(2) = 0;           %se pega al tope
                        obj.alpha3(k) = 0;
                else
                    obj.alpha3(k) = tau_now/obj.I3z;    % free motion
                end
            
                obj.Y3(:,k) = y3Next;
            
            
            
                % CALCULATIONS SECOND PHALANX
                k1 = f2(t             , y3Next(1) , y2              , y1(1), middInertia);
                k2 = f2(t+obj.dt/2    , y3Next(1) , y2+obj.dt/2*k1  , y1(1), middInertia);
                k3 = f2(t+obj.dt/2    , y3Next(1) , y2+obj.dt/2*k2  , y1(1), middInertia);
                k4 = f2(t+obj.dt      , y3Next(1) , y2+obj.dt  *k3  , y1(1), middInertia);
            
                y2Next = y2 + obj.dt/6*(k1 + 2*k2 + 2*k3 + k4);
            
                % ---------- HARD‑STOP LOGIC ---------------------------------
                [tau_now, T2_1] = tau2(y3Next(1), y2Next(1), 0, obj.time(k));
            
                if y2Next(1) < obj.thetaMin
                    y2Next(1) = obj.thetaMin;
                elseif y2Next(1) > obj.thetaMax
                    y2Next(1) = obj.thetaMax;
                end
            
                if y2Next(1) == obj.thetaMin
                    y2Next(2) = 0;
                    obj.alpha2(k) = 0;
                elseif y2Next(1) == obj.thetaMax
                    y2Next(2) = 0;
                    obj.alpha2(k) = 0;
                else
                    obj.alpha2(k) = tau_now/obj.I2z;
                end
            
                obj.Y2(:,k) = y2Next;
            
            
                
                % CALCULATIONS FIRST PHALANX
                k1 = f1(t             , y3Next(1), y2Next(1), y1             , proxInertia);
                k2 = f1(t+obj.dt/2    , y3Next(1), y2Next(1), y1+obj.dt/2*k1 , proxInertia);
                k3 = f1(t+obj.dt/2    , y3Next(1), y2Next(1), y1+obj.dt/2*k2 , proxInertia);
                k4 = f1(t+obj.dt      , y3Next(1), y2Next(1), y1+obj.dt  *k3 , proxInertia);
            
                y1Next = y1 + obj.dt/6*(k1 + 2*k2 + 2*k3 + k4);
            
                % ---------- HARD‑STOP LOGIC ---------------------------------
                [tau_now, T1_0] = tau1(y3Next(1), y2Next(1), y1Next(1), obj.time(k));
            
                if y1Next(1) < obj.thetaMin
                    y1Next(1) = obj.thetaMin;
                elseif y1Next(1) > obj.thetaMax
                    y1Next(1) = obj.thetaMax;
                end
            
                if y1Next(1) == obj.thetaMin
                    y1Next(2) = 0;
                    obj.alpha1(k) = 0;
                elseif y1Next(1) == obj.thetaMax
                    y1Next(2) = 0;
                    obj.alpha1(k) = 0;
                else
                    obj.alpha1(k) = tau_now/obj.I2z;
                end
            
                obj.Y1(:,k) = y1Next;
            end

            % Then store results to obj.results
            obj.alpha3(1) = obj.alpha3(2);
            obj.alpha2(1) = obj.alpha2(2);
            obj.alpha1(1) = obj.alpha1(2);

            obj.theta1 = obj.Y1(1,:);
            obj.theta2 = obj.Y2(1,:);
            obj.theta3 = obj.Y3(1,:);
    
            obj.omega1 = obj.Y2(2,:);
            obj.omega2 = obj.Y3(2,:);
            obj.omega3 = obj.Y1(2,:);

            obj.results.Y1 = obj.Y1;
            obj.results.Y2 = obj.Y2;
            obj.results.Y3 = obj.Y3;
            obj.results.alpha1 = obj.alpha1;
            obj.results.alpha2 = obj.alpha2;
            obj.results.alpha3 = obj.alpha3;
            obj.results.theta1 = obj.theta1(1,:);
            obj.results.theta2 = obj.theta2(1,:);
            obj.results.theta3 = obj.theta3(1,:);
            obj.results.omega1 = obj.theta1(1,:);
            obj.results.omega2 = obj.theta2(1,:);
            obj.results.omega3 = obj.theta3(1,:);
            obj.results.tau1_v = arrayfun(@(th3, th2, th1, t) tau1(th3, th2, th1, t), obj.theta3, obj.theta2, obj.theta1, obj.time);
            obj.results.tau2_v = arrayfun(@(th3, th2, th1, t) tau2(th3, th2, th1, t), obj.theta3, obj.theta2, obj.theta1, obj.time);
            obj.results.tau3_v = arrayfun(@(th3, th2, th1, t) tau3(th3, th2, th1, t), obj.theta3, obj.theta2, obj.theta1, obj.time);
        end

        function [proximalInertia, middleInertia, distalInertia] = GetPhalanxsInertias(obj, phalanxIds, th1, th2, th3)
            % Check input
            if isempty(phalanxIds)
                warning('No phalanx IDs provided.');
                return;
            end

            proximalInertia = 0;
            middleInertia = 0;
            distalInertia = 0;
        
            % Loop over each phalanx ID
            for i = 1:numel(phalanxIds)
                phalanxId = phalanxIds{i};

                % Example processing: retrieve or display information
                % fprintf('Processing Phalanx ID: %s\n', phalanxId);

                % myMass = obj.dataAccumulator.GetTotalMass(phalanxId);
                % fprintf("Total Mass [kg]: %.4f \n", myMass);

                % DTO_COM_L3 = dataAccumulator.GetCenterOfMassDTO(phalanxId);
                % fprintf("Center of Mass (local frame): X: %.4f, Y: %.4f, Z:  %.4f \n", DTO_COM_L3.X, DTO_COM_L3.Y, DTO_COM_L3.Z);

                if contains(phalanxId, "distal")
                    % inertia based on phalanx frame origin
                    inertiaNet = obj.dataAccumulator.GetInertiaMatrix(phalanxId);     % .NET 2D array
                    inertiaMat = double(inertiaNet);                               % Convert to MATLAB matrix
                    distalInertia = inertiaMat(3, 3);                              % Extract scalar Izz
                end

                if contains(phalanxId, "middle")
                    % Rotation origin for middle phalanx
                    O2 = [0; 0];  % Assume middle joint is at origin in local frame
                    
                    % Compute position of distal joint (O3) using length L2 and rotation th2
                    O3 = O2 + obj.R(th2) * obj.L2;  % obj.R is likely a 2x2 rotation matrix function
                
                    % Resolve names
                    fingerBaseName = erase(phalanxId, "middle");              % e.g., 'Index'
                    distalFingerName = strcat("distal", fingerBaseName);      % e.g., 'distalIndex'
                
                    % Get COM of the distal phalanx in its local frame and convert to global
                    DTO_COM_L3 = obj.dataAccumulator.GetCenterOfMassDTO(distalFingerName);
                    tmpCOM_L3 = O3 + obj.R(th2 + th3) * [DTO_COM_L3.X; DTO_COM_L3.Y];
                
                    % Inertia of distal phalanx about its own local frame (assumed at joint O3)
                    I3_Net = obj.dataAccumulator.GetInertiaMatrix(distalFingerName);
                    I3_Mat = double(I3_Net);
                    I3zz = I3_Mat(3, 3);
                
                    % Mass of distal phalanx
                    mass_L3 = obj.dataAccumulator.GetTotalMass(distalFingerName);
                
                    % Distance from COM of distal phalanx to the middle joint axis (O2)
                    r = tmpCOM_L3 - O2;
                    d_squared = r(1)^2 + r(2)^2;
                
                    % Shift I3zz using parallel axis theorem to middle joint
                    I3zz_shifted = I3zz + mass_L3 * d_squared;
                
                    % Inertia of the middle phalanx (about O2, assumed already correct in GetInertiaMatrix)
                    I2 = obj.dataAccumulator.GetInertiaMatrix(phalanxId);
                    I2zz = I2(3,3);
                
                    % Total effective inertia at the middle joint
                    middleInertia = I2zz + I3zz_shifted;
                
                    % Output or store the result
                    % fprintf("Effective middle inertia at joint [Z]: %.6f\n", middleInertia);
                end

                if contains(phalanxId, "proximal")
                    % Rotation origin for proximal phalanx
                    O1 = [0; 0];  % assume joint 1 is origin of global or local frame
                
                    % Positions of joint 2 and 3
                    O2 = O1 + obj.R(th1) * obj.L1;
                    O3 = O2 + obj.R(th1 + th2) * obj.L2;
                
                    % Extract base name (e.g., 'Index') and build phalanx names
                    fingerBaseName = erase(phalanxId, "proximal");
                    middleFingerName = strcat("middle", fingerBaseName);
                    distalFingerName = strcat("distal", fingerBaseName);
                
                    % --- DISTAL PHALANX CONTRIBUTION ---
                
                    DTO_COM_L3 = obj.dataAccumulator.GetCenterOfMassDTO(distalFingerName);
                    tmpCOM_L3 = O3 + obj.R(th1 + th2 + th3) * [DTO_COM_L3.X; DTO_COM_L3.Y];
                
                    I3_net = obj.dataAccumulator.GetInertiaMatrix(distalFingerName);
                    I3_mat = double(I3_net);
                    I3zz = I3_mat(3, 3);
                
                    mass_L3 = obj.dataAccumulator.GetTotalMass(distalFingerName);
                    r3 = tmpCOM_L3 - O1;
                    d3_squared = r3(1)^2 + r3(2)^2;
                    I3zz_shifted = I3zz + mass_L3 * d3_squared;
                
                    % --- MIDDLE PHALANX CONTRIBUTION ---
                
                    DTO_COM_L2 = obj.dataAccumulator.GetCenterOfMassDTO(middleFingerName);
                    tmpCOM_L2 = O2 + obj.R(th1 + th2) * [DTO_COM_L2.X; DTO_COM_L2.Y];
                
                    I2_net = obj.dataAccumulator.GetInertiaMatrix(middleFingerName);
                    I2_mat = double(I2_net);
                    I2zz = I2_mat(3, 3);
                
                    mass_L2 = obj.dataAccumulator.GetTotalMass(middleFingerName);
                    r2 = tmpCOM_L2 - O1;
                    d2_squared = r2(1)^2 + r2(2)^2;
                    I2zz_shifted = I2zz + mass_L2 * d2_squared;
                
                    % --- PROXIMAL PHALANX CONTRIBUTION ---
                
                    I1_net = obj.dataAccumulator.GetInertiaMatrix(phalanxId);
                    I1_mat = double(I1_net);
                    I1zz = I1_mat(3, 3);  % inertia about its own joint (O1)
                
                    % --- TOTAL EFFECTIVE INERTIA at Joint 1 (proximal)
                    proximalInertia = I1zz + I2zz_shifted + I3zz_shifted;
                
                    % fprintf("Effective proximal inertia at joint [Z]: %.6f\n", proximalInertia);
                end

            end
        end

        function plotResults(obj)
            tau1 = @(th3, th2, th1, t) torqueLink1_vFinal(th2, th1, obj.Tmag(t), obj.COM_L1, obj.AP, obj.P, obj.P0, obj.P1, obj.P2, obj.P3, obj.L0, obj.L1, obj.L2, obj.g, obj.m1, obj.R);
            tau2 = @(th3, th2, th1, t) torqueLink2_vFinal(th3, th2, th1, obj.Tmag(t), obj.COM_L2, obj.P0, obj.P1, obj.P2, obj.P3, obj.L0, obj.L1, obj.L2, obj.g, obj.m2, obj.R);
            tau3 = @(th3, th2, th1, t) torqueLink3_vFinal(th3, th2, th1, obj.Tmag(t), obj.COM_L3, obj.P2, obj.P3, obj.L0, obj.L1, obj.L2, obj.g, obj.m3, obj.R);
            
            tau3_v = arrayfun(@(th3, th2, th1, t) tau3(th3, th2, th1, t), obj.theta3, obj.theta2, obj.theta1, obj.time);
            
            % plots
            figure('Name','Third Phalanx Dynamics','Color','w');
            subplot(4,1,1), plot(obj.time, tau3_v,'LineWidth',1.2),  grid on
            ylabel('\tau_{3z}  [N·m]'),   title('Applied Torque')
            
            subplot(4,1,2), plot(obj.time, rad2deg(obj.theta3),'LineWidth',1.2), grid on
            ylabel('\theta_{3z}  [deg]'),  title('Angular Position')
            
            subplot(4,1,3), plot(obj.time, rad2deg(obj.omega3),'LineWidth',1.2), grid on
            ylabel('\omega_{3z}  [deg/s]'), title('Angular Velocity')
            
            subplot(4,1,4), plot(obj.time, rad2deg(obj.alpha3),'LineWidth',1.2), grid on
            ylabel('\alpha_{3z}  [deg/s^2]'), xlabel('Time  [s]')
            title('Angular Acceleration (net)')
            
            %plotSimpleFinger(theta3, tf, COM_L2, COM_L3, P2, P3, L2, PFin, R, desiredFPS);
            
            
            tau2_v = arrayfun(@(th3, th2, th1 ,t) tau2(th3, th2, th1, t), obj.theta3, obj.theta2, obj.theta1, obj.time);
            
            % plots
            figure('Name','Second Phalanx Dynamics','Color','w');
            subplot(4,1,1), plot(obj.time, tau2_v,'LineWidth',1.2),  grid on
            ylabel('\tau_{2z}  [N·m]'),   title('Applied Torque')
            
            subplot(4,1,2), plot(obj.time, rad2deg(obj.theta2),'LineWidth',1.2), grid on
            ylabel('\theta_{2z}  [deg]'),  title('Angular Position')
            
            subplot(4,1,3), plot(obj.time, rad2deg(obj.omega2),'LineWidth',1.2), grid on
            ylabel('\omega_{2z}  [deg/s]'), title('Angular Velocity')
            
            subplot(4,1,4), plot(obj.time, rad2deg(obj.alpha2),'LineWidth',1.2), grid on
            ylabel('\alpha_{2z}  [deg/s^2]'), xlabel('Time  [s]')
            title('Angular Acceleration (net)')
            
            %plotDoubleFinger(theta2, theta3, tf, COM_L1, COM_L2, COM_L3, L1, L2, L3, P0, P1, P2, P3, PFin, R, desiredFPS);
            
            
            tau1_v = arrayfun(@(th3, th2, th1 ,t) tau1(th3, th2, th1, t), obj.theta3, obj.theta2, obj.theta1, obj.time);
            
            % plots
            figure('Name','First Phalanx Dynamics','Color','w');
            subplot(4,1,1), plot(obj.time, tau1_v,'LineWidth',1.2),  grid on
            ylabel('\tau_{1z}  [N·m]'),   title('Applied Torque')
            
            subplot(4,1,2), plot(obj.time, rad2deg(obj.theta1),'LineWidth',1.2), grid on
            ylabel('\theta_{1z}  [deg]'),  title('Angular Position')
            
            subplot(4,1,3), plot(obj.time, rad2deg(obj.omega2),'LineWidth',1.2), grid on
            ylabel('\omega_{1z}  [deg/s]'), title('Angular Velocity')
            
            subplot(4,1,4), plot(obj.time, rad2deg(obj.alpha2),'LineWidth',1.2), grid on
            ylabel('\alpha_{1z}  [deg/s^2]'), xlabel('Time  [s]')
            title('Angular Acceleration (net)')
            
            plotTripleFinger(obj.theta1, obj.theta2, obj.theta3, obj.tf, obj.COM_L1, obj.COM_L2, obj.COM_L3, obj.L0, obj.L1, obj.L2, obj.L3, obj.AP, obj.P, obj.P0, obj.P1, obj.P2, obj.P3, obj.PFin, obj.R, obj.desiredFPS);
        end

        function config = loadConfigFile(obj, filename)
            % Load JSON file into a MATLAB struct
            fid = fopen(filename, 'r');
            raw = fread(fid, inf);
            str = char(raw');
            fclose(fid);
            
            config = jsondecode(str);
        end

        function plotTripleFinger(obj, theta_array1, theta_array2, theta_array3, maxDuration, desiredFPS)
            if nargin < 6 || isempty(desiredFPS)
                desiredFPS = obj.desiredFPS;
            end
            if nargin < 5 || isempty(maxDuration)
                maxDuration = obj.tf;
            end
            if nargin < 4 || isempty(theta_array3)
                theta_array3 = obj.theta3;
            end
            if nargin < 3 || isempty(theta_array2)
                theta_array2 = obj.theta2;
            end
            if nargin < 2 || isempty(theta_array1)
                theta_array1 = obj.theta1;
            end

            % O0 = [0; 0];
            % O1 = O0 + L0;

            % === FRAME SELECTION LOGIC ===
            totalFrames = length(theta_array2);
            maxFramesToShow = round(maxDuration * desiredFPS);  % e.g., 10s * 30 fps = 300 frames

            if totalFrames > maxFramesToShow
                indices = round(linspace(1, totalFrames, maxFramesToShow));
            else
                indices = 1:totalFrames;
            end

            % === Create a new figure for animation ===
            figAnim = figure('Name', 'Double Finger Link Animation', 'Color', 'w');

            for i = 1:length(indices)
                figure(figAnim);
                cla;

                theta1 = theta_array1(indices(i));
                theta2 = theta_array2(indices(i));
                theta3 = theta_array3(indices(i));

                O0 = [0; 0];
                O1 = O0 + obj.L0;
                O2 = O1 + obj.R(theta1) * obj.L1;
                O3 = O2 + obj.R(theta1 + theta2) * obj.L2;

                % Rotated points (relative to O2)
                COM_L1_1 = (obj.R(theta1) * obj.COM_L1) + O1;
                COM_L2_2 = (obj.R(theta1 + theta2) * obj.COM_L2) + O2;
                COM_L3_3 = (obj.R(theta3 + theta2 + theta1) * obj.COM_L3) + O3;
                P_1      = (obj.R(theta1) * obj.P)      + O1;
                P0_1     = (obj.R(theta1) * obj.P0)     + O1;
                P1_2     = (obj.R(theta2 + theta1) * obj.P1)     + O2;
                P2_2     = (obj.R(theta2 + theta1) * obj.P2)     + O2;
                P3_3     = (obj.R(theta1 + theta3 + theta2) * obj.P3)     + O3;
                PFin_3   = (obj.R(theta1 + theta3 + theta2) * obj.PFin)   + O3;

                hold on;
                axis equal;
                grid on;
                xlabel('X [m]');
                ylabel('Y [m]');
                title(sprintf('Finger Link Geometry (Frame %d/%d)', i, length(indices)));

                % Plot links and markers
                plot([O0(1), O1(1)], [O0(2), O1(2)], 'k-', 'LineWidth', 2);
                plot([O1(1), O2(1)], [O1(2), O2(2)], 'k-', 'LineWidth', 2);
                plot([O2(1), O3(1)], [O2(2), O3(2)], 'b-', 'LineWidth', 2);
                plot([O3(1), PFin_3(1)], [O3(2), PFin_3(2)], 'b-', 'LineWidth', 2);
                plot(O0(1), O0(2), 'ko', 'MarkerFaceColor', 'k');
                plot(O1(1), O1(2), 'ko', 'MarkerFaceColor', 'k');
                plot(O2(1), O2(2), 'ko', 'MarkerFaceColor', 'k');
                plot(O3(1), O3(2), 'ko', 'MarkerFaceColor', 'k');
                plot(COM_L1_1(1), COM_L1_1(2), 'ro', 'MarkerFaceColor', 'r');
                plot(COM_L2_2(1), COM_L2_2(2), 'ro', 'MarkerFaceColor', 'r');
                plot(COM_L3_3(1), COM_L3_3(2), 'ro', 'MarkerFaceColor', 'r');
                plot(obj.AP(1), obj.AP(2), 'go', 'MarkerFaceColor', 'g');
                plot(P_1(1), P_1(2), 'go', 'MarkerFaceColor', 'g');
                plot(P0_1(1), P0_1(2), 'go', 'MarkerFaceColor', 'g');
                plot(P2_2(1), P2_2(2), 'go', 'MarkerFaceColor', 'g');
                plot(P1_2(1), P1_2(2), 'go','MarkerFaceColor', 'g');
                plot(P3_3(1), P3_3(2), 'go', 'MarkerFaceColor', 'g');

                % Labels
                text(O0(1), O0(2), ' O0', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
                text(O1(1), O1(2), ' O1', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
                text(O2(1), O2(2), ' O2', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
                text(O3(1), O3(2), ' O3', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
                text(COM_L1_1(1), COM_L1_1(2), ' COM\_L1', 'VerticalAlignment', 'top');
                text(COM_L2_2(1), COM_L2_2(2), ' COM\_L2', 'VerticalAlignment', 'top');
                text(COM_L3_3(1), COM_L3_3(2), ' COM\_L3', 'VerticalAlignment', 'top');
                text(obj.AP(1), obj.AP(2), ' AP', 'VerticalAlignment', 'top');
                text(P_1(1), P_1(2), ' P\_1', 'VerticalAlignment', 'top');
                text(P0_1(1), P0_1(2), ' P0\_1', 'VerticalAlignment', 'top');
                text(P2_2(1), P2_2(2), ' P2\_2', 'VerticalAlignment', 'top');
                text(P1_2(1), P1_2(2), ' P1\_2', 'VerticalAlignment', 'top');
                text(P3_3(1), P3_3(2), ' P3\_3', 'VerticalAlignment', 'top');

                axis([ -0.01 0.17 -0.01 0.1 ]);

                drawnow;
                pause(1 / desiredFPS);  % Smooth animation
            end

        end
    
        function obj = setBufferManagerLibraryPath(obj, assemblyPath)
            try
                NET.addAssembly(assemblyPath);
                obj.bufferManager = BufferPrint.BufferManager();
                obj.bufferManager.CreateOrOpenSharedMemory();

                % Get .NET references to Inventor app and assembly doc
                inventorApp = obj.bufferManager.GetInventorAppInstance();
                assemblyDoc = obj.bufferManager.GetAssemblyDocument();
                
                % Create the InventorObjectData object
                invObjData = BufferPrint.InventorObjectData(inventorApp, assemblyDoc);
                
                % Call SetInventorObjectData
                obj.bufferManager.SetInventorObjectData(invObjData);

                obj.digitalTwinVisual = true;

                obj.dataAccumulator = obj.bufferManager.GetSubassemblyAccumulator();
                disp('[INFO] BufferManager DLL loaded and shared memory initialized.');
            catch ME
                warning(ME.identifier ,'[ERROR] Failed to load BufferManager or initialize shared memory: %s', ME.message);
                obj.digitalTwinVisual = false;
            end
        end

        function drawStepDigitalTwin(obj, action, jointId, angle)

            msg = obj.bufferManager.WriteToBuffer(action, jointId, single(angle));

            % Print feedback
            disp("Buffer Response:" + char(msg.ToString()));
        end

        function delete(obj)
            % Código a ejecutar antes de la finalización del objeto
            disp('Limpio recursos antes de que se vaya!');
            
            obj.bufferManager.CloseSharedMemory();
        end
    end
end
