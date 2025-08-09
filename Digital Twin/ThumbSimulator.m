classdef ThumbSimulator
    properties
        % === Raw configuration ===
        config                          % Original JSON structure loaded as struct
        bufferManager
        dataAccumulator
        originTransformation
        O1_Mat
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
        O0 double                       % Origin
        P0 double
        % P double
        P1 double
        P2 double
        P3 double
        PFin double
    
        % === Segment lengths ===
        L1 double
        L2 double
        L3 double

        % === gears radios ===
        radioProximalThumb double
        radioPuley double
    
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

        %
        err3
        err2
        err1
    
        % === Simulation results ===
        results struct = struct()       % Container for simulation outputs (Y1, Y2, Y3, alpha, etc.)
    end

    methods
        function obj = ThumbSimulator(configFile)
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
            obj.R = @(phi)[cos(phi), -sin(phi), 0;
               sin(phi),  cos(phi), 0;
               0       ,  0       , 1];

            % Geometry (converted to column vectors)
            % obj.L0 = obj.columnVec(c.basePhalanx.L0);
            % obj.AP = obj.columnVec(c.basePhalanx.AP);
            
            obj.P0 = obj.columnVec(c.points.P0);
            % obj.P  = obj.columnVec(c.points.P);
            obj.P1 = obj.columnVec(c.points.P1);
            obj.P2 = obj.columnVec(c.points.P2);
            obj.P3 = obj.columnVec(c.points.P3);
            obj.PFin = obj.columnVec(c.points.PFin);
        
            obj.L0 = obj.columnVec(c.lengths_m.L0);
            obj.L1 = obj.columnVec(c.lengths_m.L1);
            obj.L2 = obj.columnVec(c.lengths_m.L2);
            obj.L3 = obj.columnVec(c.lengths_m.L3);

            obj.radioProximalThumb = c.radioProximalThumb;
            obj.radioPuley = c.radioPuley;
        
            % obj.m1 = c.masses_kg.m1;
            % obj.m2 = c.masses_kg.m2;
            % obj.m3 = c.masses_kg.m3;
        
            % obj.COM_L1 = obj.columnVec(c.centersOfMass_m.COM_L1);
            % obj.COM_L2 = obj.columnVec(c.centersOfMass_m.COM_L2);
            % obj.COM_L3 = obj.columnVec(c.centersOfMass_m.COM_L3);
        
            % obj.I1z = c.inertias_kgm2.I1z;
            % obj.I2z = c.inertias_kgm2.I2z;
            % obj.I3z = c.inertias_kgm2.I3z;

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

            % === Set Mass and com ===
            DTO_COM_L1 = obj.dataAccumulator.GetCenterOfMassDTO("proximalThumb");
            obj.COM_L1 = [DTO_COM_L1.X; DTO_COM_L1.Y; DTO_COM_L1.Z];
            
            DTO_COM_L2 = obj.dataAccumulator.GetCenterOfMassDTO("middleThumb");
            obj.COM_L2 = [DTO_COM_L2.X; DTO_COM_L2.Y; DTO_COM_L2.Z];

            DTO_COM_L3 = obj.dataAccumulator.GetCenterOfMassDTO("distalThumb");
            obj.COM_L3 = [DTO_COM_L3.X; DTO_COM_L3.Y; DTO_COM_L3.Z];

            obj.m1 = obj.dataAccumulator.GetTotalMass("proximalThumb");
            obj.m2 = obj.dataAccumulator.GetTotalMass("middleThumb");
            obj.m3 = obj.dataAccumulator.GetTotalMass("distalThumb");

            % === Torque functions ===
            torqueMotor = TMag;
            % torqueMotor = @(t) 0;

            tau1 = @(th3, th2, th1, t, specialTransform) torqueLink1_thumb(th1, obj.COM_L1, obj.m1, obj.g, torqueMotor(t), obj.radioPuley, obj.radioProximalThumb, obj.R, specialTransform);
            tau2 = @(th3, th2, th1, t, specialTransform) torqueLink2_thumb(th3, th2, th1, TMag(t), obj.COM_L2, obj.P0, obj.P1, obj.P2, obj.P3, obj.L0, obj.L1, obj.L2, obj.g, obj.m2, obj.R, specialTransform);
            tau3 = @(th3, th2, th1, t, specialTransform) torqueLink3_thumb(th3, th2, th1, TMag(t), obj.COM_L3, obj.P2, obj.P3, obj.L0, obj.L1, obj.L2, obj.g, obj.m3, obj.R, specialTransform);
            % tau3 = @(th3, th2, th1, t, specialTransform) torqueLink3_thumb(th3, th2, th1, TMag(t), obj.COM_L3, obj.P1, obj.P2, obj.P3, obj.L0, obj.L1, obj.L2, obj.g, obj.m3, obj.R, specialTransform);

            getTorque1Only = @(th1, t, specialTransform) obj.getFirstOutput(0, 0, th1, t, tau1, specialTransform);
            getTorque2Only = @(th3, th2, th1, t, specialTransform) obj.getFirstOutput(th3, th2, th1, t, tau2, specialTransform);
            getTorque3Only = @(th3, th2, th1, t, specialTransform) obj.getFirstOutput(th3, th2, th1, t, tau3, specialTransform);
        
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

            obj.err3 = zeros(1,obj.N-1);   % distal (θ3, ω3)
            obj.err2 = zeros(1,obj.N-1);   % middle
            obj.err1 = zeros(1,obj.N-1);   % proximal

            % === Inertia Calculation ===
            
            f1 = @(t, y, I, specialTransform) [ ...
                y(2);
                getTorque1Only(y(1), t, specialTransform) / I ...
            ];
        
            f2 = @(t, th_3, y, th_1, I, specialTransform) [ ...
                y(2);
                % getTorque2Only(th_3, y(1), th_1, t) / obj.I2z ...
                getTorque2Only(th_3, y(1), th_1, t, specialTransform) / I ...
            ];
        
            f3 = @(t, y, th_2, th_1, I, specialTransform) [ ...
                y(2);
                % getTorque3Only(y(1), th_2, th_1, t) / obj.I3z ...
                getTorque3Only(y(1), th_2, th_1, t, specialTransform) / I ...
            ];


            specialTrans = [];
            for i = 1:numel(phanlanxIds)
                phalanxId = phanlanxIds{i};
                
                if contains(phalanxId, "proximal")
                    specialTrans = obj.dataAccumulator.GetSpecialTransformationDTO(phalanxId);
                end
            end
        
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
                [proxInertiaZ, middInertiaZ, distInertiaZ] = obj.GetPhalanxsInertias(phanlanxIds, y1(1), y2(1), y3(1));
                
                obj.I3z = distInertiaZ;
                obj.I2z = middInertiaZ;
                obj.I1z = proxInertiaZ;

                % CALCULATIONS THIRD PHALANX

                k1 = f3(t             , y3              , y2(1), y1(1), obj.I3z, specialTrans);
                k2 = f3(t+obj.dt/2    , y3+obj.dt/2*k1  , y2(1), y1(1), obj.I3z, specialTrans);
                k3 = f3(t+obj.dt/2    , y3+obj.dt/2*k2  , y2(1), y1(1), obj.I3z, specialTrans);
                k4 = f3(t+obj.dt      , y3+obj.dt  *k3  , y2(1), y1(1), obj.I3z, specialTrans);
            
                y3_big = y3 + obj.dt/6*(k1 + 2*k2 + 2*k3 + k4);   % <- yₙ₊₁ via 1×dt

                % ------------------------------------------------------------
                % (B) TWO successive half-steps  (h = dt/2)  for error estimate
                % ------------------------------------------------------------
                h = obj.dt/2;
            
                % ----- first half step -----------------------------
                k1a = f3(t      , y3              , y2(1), y1(1), obj.I3z, specialTrans);
                k2a = f3(t+h/2  , y3+h/2*k1a      , y2(1), y1(1), obj.I3z, specialTrans);
                k3a = f3(t+h/2  , y3+h/2*k2a      , y2(1), y1(1), obj.I3z, specialTrans);
                k4a = f3(t+h    , y3+h   *k3a     , y2(1), y1(1), obj.I3z, specialTrans);
                y3_half = y3 + h/6*(k1a + 2*k2a + 2*k3a + k4a);
            
                % ----- second half step ----------------------------
                k1b = f3(t+h    , y3_half         , y2(1), y1(1), obj.I3z, specialTrans);
                k2b = f3(t+3*h/2, y3_half+h/2*k1b , y2(1), y1(1), obj.I3z, specialTrans);
                k3b = f3(t+3*h/2, y3_half+h/2*k2b , y2(1), y1(1), obj.I3z, specialTrans);
                k4b = f3(t+2*h  , y3_half+h  *k3b , y2(1), y1(1), obj.I3z, specialTrans);
                y3_two = y3_half + h/6*(k1b + 2*k2b + 2*k3b + k4b);   % y via 2×(dt/2)
            
                % ----- England p.168 local error for *first* dt step --------------
                obj.err3(k-1) = norm(y3_two - y3_big, 2) / 30;
            
                % Replace y3Next with the more accurate two-half-step value ---------
                y3Next = y3_two;
            
                % ---------- HARD‑STOP LOGIC ---------------------------------
                [tau_now, T3_2] = tau3(y3Next(1), y2(1), y1(1), obj.time(k), specialTrans);                % torque at this step
            
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
                k1 = f2(t             , y3Next(1) , y2              , y1(1), obj.I2z, specialTrans);
                k2 = f2(t+obj.dt/2    , y3Next(1) , y2+obj.dt/2*k1  , y1(1), obj.I2z, specialTrans);
                k3 = f2(t+obj.dt/2    , y3Next(1) , y2+obj.dt/2*k2  , y1(1), obj.I2z, specialTrans);
                k4 = f2(t+obj.dt      , y3Next(1) , y2+obj.dt  *k3  , y1(1), obj.I2z, specialTrans);
            
                y2_big = y2 + obj.dt/6*(k1 + 2*k2 + 2*k3 + k4);

                % --- two half-steps (h = dt/2) --------------------------------
                h = obj.dt/2;
                
                % half-step 1
                k1a = f2(t      , y3Next(1) , y2              , y1(1), obj.I2z, specialTrans);
                k2a = f2(t+h/2  , y3Next(1) , y2+h/2*k1a      , y1(1), obj.I2z, specialTrans);
                k3a = f2(t+h/2  , y3Next(1) , y2+h/2*k2a      , y1(1), obj.I2z, specialTrans);
                k4a = f2(t+h    , y3Next(1) , y2+h   *k3a     , y1(1), obj.I2z, specialTrans);
                y2_half = y2 + h/6*(k1a + 2*k2a + 2*k3a + k4a);
                
                % half-step 2
                k1b = f2(t+h    , y3Next(1) , y2_half         , y1(1), obj.I2z, specialTrans);
                k2b = f2(t+3*h/2, y3Next(1) , y2_half+h/2*k1b , y1(1), obj.I2z, specialTrans);
                k3b = f2(t+3*h/2, y3Next(1) , y2_half+h/2*k2b , y1(1), obj.I2z, specialTrans);
                k4b = f2(t+2*h  , y3Next(1) , y2_half+h  *k3b , y1(1), obj.I2z, specialTrans);
                y2_two = y2_half + h/6*(k1b + 2*k2b + 2*k3b + k4b);
                
                % --- England p.168 error (first dt step) ----------------------
                obj.err2(k-1) = norm(y2_two - y2_big, 2) / 30;
                
                % --- use the more accurate value ------------------------------
                y2Next = y2_two;
            
                % ---------- HARD‑STOP LOGIC ---------------------------------
                [tau_now, T2_1] = tau2(y3Next(1), y2Next(1), y1(1), obj.time(k), specialTrans);
            
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
                k1 = f1(t             , y1             , obj.I1z, specialTrans);
                k2 = f1(t+obj.dt/2    , y1+obj.dt/2*k1 , obj.I1z, specialTrans);
                k3 = f1(t+obj.dt/2    , y1+obj.dt/2*k2 , obj.I1z, specialTrans);
                k4 = f1(t+obj.dt      , y1+obj.dt  *k3 , obj.I1z, specialTrans);

                y1_big = y1 + obj.dt/6*(k1 + 2*k2 + 2*k3 + k4);

                % --- two half-steps (h = dt/2) --------------------------------
                h = obj.dt/2;
                
                % half-step 1
                k1a = f1(t      , y1              , obj.I1z, specialTrans);
                k2a = f1(t+h/2  , y1+h/2*k1a      , obj.I1z, specialTrans);
                k3a = f1(t+h/2  , y1+h/2*k2a      , obj.I1z, specialTrans);
                k4a = f1(t+h    , y1+h   *k3a     , obj.I1z, specialTrans);
                y1_half = y1 + h/6*(k1a + 2*k2a + 2*k3a + k4a);
                
                % half-step 2
                k1b = f1(t+h    , y1_half         , obj.I1z, specialTrans);
                k2b = f1(t+3*h/2, y1_half+h/2*k1b , obj.I1z, specialTrans);
                k3b = f1(t+3*h/2, y1_half+h/2*k2b , obj.I1z, specialTrans);
                k4b = f1(t+2*h  , y1_half+h  *k3b , obj.I1z, specialTrans);
                y1_two = y1_half + h/6*(k1b + 2*k2b + 2*k3b + k4b);
                
                % --- England p.168 error (first dt step) ----------------------
                obj.err1(k-1) = norm(y1_two - y1_big, 2) / 30;
                
                % --- use the more accurate value ------------------------------
                y1Next = y1_two;

                % ---------- HARD‑STOP LOGIC ---------------------------------
                [tau_now, T0] = tau1(y3Next(1), y2Next(1), y1Next(1), obj.time(k), specialTrans);

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
                    obj.alpha1(k) = tau_now / obj.I1z;
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
    
            obj.omega1 = obj.Y1(2,:);
            obj.omega2 = obj.Y2(2,:);
            obj.omega3 = obj.Y3(2,:);

            obj.results.Y1 = obj.Y1;
            obj.results.Y2 = obj.Y2;
            obj.results.Y3 = obj.Y3;
            obj.results.alpha1 = obj.alpha1(1,:);
            obj.results.alpha2 = obj.alpha2(1,:);
            obj.results.alpha3 = obj.alpha3(1,:);
            obj.results.theta1 = obj.theta1(1,:);
            obj.results.theta2 = obj.theta2(1,:);
            obj.results.theta3 = obj.theta3(1,:);
            obj.results.omega1 = obj.omega1(1,:);
            obj.results.omega2 = obj.omega2(1,:);
            obj.results.omega3 = obj.omega3(1,:);
            obj.results.tau1_v = arrayfun(@(th3, th2, th1, t) tau1(th3, th2, th1, t, specialTrans), obj.theta3, obj.theta2, obj.theta1, obj.time);
            obj.results.tau2_v = arrayfun(@(th3, th2, th1, t) tau2(th3, th2, th1, t, specialTrans), obj.theta3, obj.theta2, obj.theta1, obj.time);
            obj.results.tau3_v = arrayfun(@(th3, th2, th1, t) tau3(th3, th2, th1, t, specialTrans), obj.theta3, obj.theta2, obj.theta1, obj.time);

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
                    O2 = [0; 0; 0];  % Assume middle joint is at origin in local frame
                    
                    % Compute position of distal joint (O3) using length L2 and rotation th2
                    O3 = O2 + obj.R(th2) * obj.L2;  % obj.R is likely a 2x2 rotation matrix function
                
                    % Resolve names
                    fingerBaseName = erase(phalanxId, "middle");              % e.g., 'Index'
                    distalFingerName = strcat("distal", fingerBaseName);      % e.g., 'distalIndex'
                
                    % Get COM of the distal phalanx in its local frame and convert to global
                    DTO_COM_L3 = obj.dataAccumulator.GetCenterOfMassDTO(distalFingerName);
                    tmpCOM_L3 = O3 + obj.R(th2 + th3) * [DTO_COM_L3.X; DTO_COM_L3.Y; DTO_COM_L3.Z];
                
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
                    % Extract base name (e.g., 'Index') and build phalanx names
                    fingerBaseName = erase(phalanxId, "proximal");
                    middleFingerName = strcat("middle", fingerBaseName);
                    distalFingerName = strcat("distal", fingerBaseName);
                    
                    % --- Fixed transformation from proximal to middle frame ---
                    O1_2_STrans = obj.dataAccumulator.GetSpecialTransformationDTO(phalanxId);
                    R_mp = [O1_2_STrans.XX, O1_2_STrans.XY, O1_2_STrans.XZ;
                        O1_2_STrans.YX, O1_2_STrans.YY, O1_2_STrans.YZ;
                        O1_2_STrans.ZX, O1_2_STrans.ZY, O1_2_STrans.ZZ];  % Rotation from proximal to middle base
                    r_mp = [O1_2_STrans.Trans_x; O1_2_STrans.Trans_y; O1_2_STrans.Trans_z];
                    
                    % --- Joint positions ---
                    O1 = [0; 0; 0];
                    O2 = O1 + R_mp * r_mp;
                    O3 = O2 + R_mp * obj.R(th2) * obj.L2;
                    
                    % --- Joint rotations ---
                    R2 = obj.R(th2);
                    R3 = obj.R(th3);
                    
                    % === DISTAL PHALANX CONTRIBUTION ===
                    R_d_to_p = R_mp * R2 * R3;
                    DTO_COM_L3 = obj.dataAccumulator.GetCenterOfMassDTO(distalFingerName);
                    COM_L3_local = [DTO_COM_L3.X; DTO_COM_L3.Y; DTO_COM_L3.Z];
                    tmpCOM_L3 = O3 + R_d_to_p * COM_L3_local;
                    r3 = tmpCOM_L3 - O1;
                    I3_local = double(obj.dataAccumulator.GetInertiaMatrix(distalFingerName));
                    mass_L3 = obj.dataAccumulator.GetTotalMass(distalFingerName);
                    I3_rot = R_d_to_p * I3_local * R_d_to_p';
                    I3_shift = mass_L3 * ((r3' * r3) * eye(3) - (r3 * r3'));
                    I3_total = I3_rot + I3_shift;
                    
                    % === MIDDLE PHALANX CONTRIBUTION ===
                    R_m_to_p = R_mp * R2;
                    DTO_COM_L2 = obj.dataAccumulator.GetCenterOfMassDTO(middleFingerName);
                    COM_L2_local = [DTO_COM_L2.X; DTO_COM_L2.Y; DTO_COM_L2.Z];
                    tmpCOM_L2 = O2 + R_m_to_p * COM_L2_local;
                    r2 = tmpCOM_L2 - O1;
                    I2_local = double(obj.dataAccumulator.GetInertiaMatrix(middleFingerName));
                    mass_L2 = obj.dataAccumulator.GetTotalMass(middleFingerName);
                    I2_rot = R_m_to_p * I2_local * R_m_to_p';
                    I2_shift = mass_L2 * ((r2' * r2) * eye(3) - (r2 * r2'));
                    I2_total = I2_rot + I2_shift;
                    
                    % === PROXIMAL PHALANX CONTRIBUTION ===
                    I1_local = double(obj.dataAccumulator.GetInertiaMatrix(phalanxId));
                    
                    % === TOTAL EFFECTIVE INERTIA (in proximal frame) ===
                    proximalInertia = I1_local(3,3) + I2_total(3,3) + I3_total(3,3);
                end
            end
        end

        function plotFingerResults(obj, time, Thumb)
            % === Plot results for third phalanx ===
            figure('Name','Third Phalanx Dynamics','Color','w');
            subplot(4,1,1), plot(time, Thumb.tau3,'LineWidth',1.2), grid on
            ylabel('\tau_{3z} [N·m]'), title('Applied Torque')
            yticks(-1.8:0.25:1.8)
        
            subplot(4,1,2), plot(time, rad2deg(Thumb.th3),'LineWidth',1.2), grid on
            ylabel('\theta_{3z} [deg]'), title('Angular Position')
            yticks(0:10:120)
        
            subplot(4,1,3), plot(time, Thumb.o3,'LineWidth',1.2), grid on
            ylabel('\omega_{3z} [deg/s]'), title('Angular Velocity')
            yticks(-3:0.5:5)
        
            subplot(4,1,4), plot(time, Thumb.a3,'LineWidth',1.2), grid on
            ylabel('\alpha_{3z} [deg/s^2]'), xlabel('Time [s]')
            title('Angular Acceleration (net)')
            yticks(-3:0.5:3)
        
            % === Plot results for second phalanx ===
            figure('Name','Second Phalanx Dynamics','Color','w');
            subplot(4,1,1), plot(time, Thumb.tau2,'LineWidth',1.2), grid on
            ylabel('\tau_{2z} [N·m]'), title('Applied Torque')
            yticks(-1.5:0.25:1.5)
        
            subplot(4,1,2), plot(time, rad2deg(Thumb.th2),'LineWidth',1.2), grid on
            ylabel('\theta_{2z} [deg]'), title('Angular Position')
            yticks(0:10:120)
        
            subplot(4,1,3), plot(time, Thumb.o2,'LineWidth',1.2), grid on
            ylabel('\omega_{2z} [deg/s]'), title('Angular Velocity')
            yticks(-3:0.25:3)
        
            subplot(4,1,4), plot(time, Thumb.a2,'LineWidth',1.2), grid on
            ylabel('\alpha_{2z} [deg/s^2]'), xlabel('Time [s]')
            title('Angular Acceleration (net)')
            yticks(-1:0.25:2)
        
            % === Plot results for first phalanx ===
            figure('Name','First Phalanx Dynamics','Color','w');
            subplot(4,1,1), plot(time, Thumb.tau1,'LineWidth',1.2), grid on
            ylabel('\tau_{1z} [N·m]'), title('Applied Torque')
            yticks(-160:40:160)
        
            subplot(4,1,2), plot(time, rad2deg(Thumb.th1),'LineWidth',1.2), grid on
            ylabel('\theta_{1z} [deg]'), title('Angular Position')
            yticks(0:10:120)
        
            subplot(4,1,3), plot(time, Thumb.o1,'LineWidth',1.2), grid on
            ylabel('\omega_{1z} [deg/s]'), title('Angular Velocity')
            yticks(-5:1:5)
        
            subplot(4,1,4), plot(time, Thumb.a1,'LineWidth',1.2), grid on
            ylabel('\alpha_{1z} [deg/s^2]'), xlabel('Time [s]')
            title('Angular Acceleration (net)')
            yticks(-45:2.5:20)
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

            specialTransformation = obj.dataAccumulator.GetSpecialTransformationDTO("proximalThumb");

            % === FRAME SELECTION LOGIC ===
            totalFrames = length(theta_array2);
            maxFramesToShow = round(maxDuration * desiredFPS);  % e.g., 10s * 30 fps = 300 frames

            if totalFrames > maxFramesToShow
                indices = round(linspace(1, totalFrames, maxFramesToShow));
            else
                indices = 1:totalFrames;
            end

            figAnim = figure('Name', 'Double Finger Link Animation', 'Color', 'w');

            for i = 1:length(indices)
                % === Setup figure ===
                figure(figAnim);
                cla;
                
                % === Joint angles ===
                theta1 = theta_array1(indices(i));
                theta2 = theta_array2(indices(i));
                theta3 = theta_array3(indices(i));
            
                % === Transformations ===
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
                R_0to1        = R_0to1_base * obj.R(theta1);
                
                R_0to2_base   = R_0to1 * R_1to2;   % middle at zero θ₂
                R_0to2        = R_0to2_base * obj.R(theta2);
                
                R_0to3        = R_0to2 * obj.R(theta3);  % distal after θ₃
            
                % === Joint Origins ===
                O0 = [0; 0; 0];
                O1 = O0;
                O2 = O1 + R_0to1 * t_1to2;
                O3 = O2 + R_0to2 * obj.L2;
            
                % === Points in local frames → global space ===
                P0 = O1 + R_0to1 * obj.P0;
                P1 = O2 + R_0to2 * obj.P1;
                P2 = O2 + R_0to2 * obj.P2;
                P3 = O3 + R_0to3 * obj.P3;
                PFin = O3 + R_0to3 * obj.PFin;

                COM_L1_global = O1 + R_0to1 * obj.COM_L1;
                COM_L2_global = O2 + R_0to2 * obj.COM_L2;
                COM_L3_global = O3 + R_0to3 * obj.COM_L3;

                hold on;
                axis equal;
                grid on;
                xlabel('X [m]');
                ylabel('Y [m]');
                title(sprintf('Finger Link Geometry (Frame %d/%d)', i, length(indices)));

                % === Project to YZ plane ===
                % --- helper: project a 3-D vector onto the plotting plane
                projYZ = @(v) [ v(3)  ;   ...  %  X-coord  ←  Z-component
                                v(2) ];       %  Y-coord  ←  Y-component

                P = [ projYZ(P0) projYZ(P1) projYZ(P2) projYZ(P3) projYZ(PFin)];
                O = [ projYZ(O0) projYZ(O1) projYZ(O2) projYZ(O3) ];
                C = [ projYZ(COM_L1_global) , projYZ(COM_L2_global) , projYZ(COM_L3_global) ];

                plot(O(1,:), O(2,:), 'rs', 'MarkerFaceColor','r');
                % Plot links between origins (e.g., kinematic chain)
                plot(O(1,[1 2]), O(2,[1 2]), 'b-', 'LineWidth', 1.5);  % O0 to O1
                plot(O(1,[2 3]), O(2,[2 3]), 'b-', 'LineWidth', 1.5);  % O1 to O2
                plot(O(1,[3 4]), O(2,[3 4]), 'b-', 'LineWidth', 1.5);  % O2 to O3
                PFin_YZ = [PFin(3); PFin(2)];   % X ← Z, Y ← Y
                plot([O(1,4), PFin_YZ(1)], [O(2,4), PFin_YZ(2)], 'b-', 'LineWidth', 2);

                % joints (black)
                plot(P(1,:),P(2,:),'ko','MarkerFaceColor','k');
                
                plot(C(1,:), C(2,:), 'gd', 'MarkerFaceColor','g', 'MarkerSize',6);

                scaleFactor = 0.15;
                W1 = [0; -obj.m1 * obj.g; 0] * scaleFactor;
                W2 = [0; -obj.m2 * obj.g; 0] * scaleFactor;
                W3 = [0; -obj.m3 * obj.g; 0] * scaleFactor;

                % Weights
                % Arrow length scale (optional visual tuning)
                scale = 0.25;
                
                % Define arrow tail and tip positions
                tail1 = COM_L1_global;
                tip1  = COM_L1_global + W1 * scale;
                
                tail2 = COM_L2_global;
                tip2  = COM_L2_global + W2 * scale;
                
                tail3 = COM_L3_global;
                tip3  = COM_L3_global + W3 * scale;

                labelPoint = @(pos, name, offset) ...
                    text(pos(1)+offset(1), pos(2)+offset(2), name, ...
                         'FontSize', 8, 'FontWeight','bold', 'Color','k'); 
            
                % Offset to avoid text-overlapping
                offset = [0.001; 0.001];  % adjust if needed
                
                % Label Origins
                labelPoint(O(:,1), 'O0', offset);
                labelPoint(O(:,2), 'O1', offset);
                labelPoint(O(:,3), 'O2', offset);
                labelPoint(O(:,4), 'O3', offset);
                
                % Label Link Points
                labelPoint(P(:,1), 'P0', offset);
                labelPoint(P(:,2), 'P1', offset);
                labelPoint(P(:,3), 'P2', offset);
                labelPoint(P(:,4), 'P3', offset);
                labelPoint(P(:,5), 'PFin', offset);
                
                % Label COMs
                labelPoint(C(:,1), 'COM1', offset);
                labelPoint(C(:,2), 'COM2', offset);
                labelPoint(C(:,3), 'COM3', offset);

                % Plot arrows as lines
                plot([tail1(3), tip1(3)], [tail1(2), tip1(2)], 'm-', 'LineWidth', 1.5);
                plot([tail2(3), tip2(3)], [tail2(2), tip2(2)], 'm-', 'LineWidth', 1.5);
                plot([tail3(3), tip3(3)], [tail3(2), tip3(2)], 'm-', 'LineWidth', 1.5);

                drawArrowhead(obj, [tip1(3);tip1(2)], W1, 'm');
                drawArrowhead(obj, [tip2(3);tip2(2)], W2, 'm');
                drawArrowhead(obj, [tip3(3);tip3(2)], W3, 'm');

                axis auto;
                drawnow;
                pause(1/desiredFPS);
            end

        end

        function drawArrowhead(obj, tip, direction, color)
            direction = direction(1:2);  % Truncate to 2D
            dir = direction / norm(direction);  % Normalize
            perp = [-dir(2); dir(1)];           % Perpendicular in 2D
        
            len = 0.002;  % Length of the arrowhead
            width = 0.001; % Width of the arrowhead
        
            base = tip - dir * len;
            left = base + perp * width;
            right = base - perp * width;
        
            fill([tip(1), left(1), right(1)], ...
                 [tip(2), left(2), right(2)], color, 'EdgeColor', color);
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

                obj.originTransformation = obj.bufferManager.GetOriginTransformation();

                %
                % UCSs = obj.dataAccumulator.GetUCSs("proximalThumb");

                % O1_Mat = [];
                % for i = 1:length(UCSs)
                %     ucsData = UCSs.Item(i - 1);  % .NET indexing starts from 0
                % 
                %     fprintf('UCS Name: %s\n', ucsData.Name);
                %     % disp(ucsData.UCSMat);  % Uncomment to print matrix
                % 
                %     if strcmp(char(ucsData.Name), 'UCS2')        % using char()
                %         tmpMat = ucsData.UCSMat;
                % 
                %         methods(tmpMat)
                %         properties(tmpMat)
                % 
                %         T = zeros(4, 4);
                % 
                %         for k = 1:3      % Rows
                %             for j = 1:4  % Columns
                %                 T(k, j) = tmpMat.Cell(k, j);  % 1-based indexing
                %             end
                %         end
                %         T(4, :) = [0, 0, 0, 1];
                % 
                %         O1_Mat = T;
                %         break;
                %     end
                % end
                % 
                % if isempty(O1_Mat)
                %     error("UCS1 not found in the UCS list.");
                % end
                % 
                % origin1 = zeros(4,4);   % Origin of UCS1 in global coordinates
                % origin1(1,1) = obj.originTransformation.XX;
                % origin1(1,2) = obj.originTransformation.XY;
                % origin1(1,3) = obj.originTransformation.XZ;
                % origin1(1,4) = obj.originTransformation.Trans_x;
                % origin1(2,1) = obj.originTransformation.YX;
                % origin1(2,2) = obj.originTransformation.YY;
                % origin1(2,3) = obj.originTransformation.YZ;
                % origin1(2,4) = obj.originTransformation.Trans_y;
                % origin1(3,1) = obj.originTransformation.ZX;
                % origin1(3,2) = obj.originTransformation.ZY;
                % origin1(3,3) = obj.originTransformation.ZZ;
                % origin1(3,4) = obj.originTransformation.Trans_z;
                % origin1(4,1) = 0;
                % origin1(4,2) = 0;
                % origin1(4,3) = 0;
                % origin1(4,4) = 1;
                % 
                % origin2 = O1_Mat(1:3, 4);  % Origin of UCS2 in global coordinates
                % 
                % obj.L1 = origin2 - origin1(1:3, 4);  % Vector from UCS1 to UCS2
                %

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
