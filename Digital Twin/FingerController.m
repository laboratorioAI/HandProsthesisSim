classdef FingerController
    properties
        Simulator       % FingerSimulator instance
        FingerName      % e.g., 'RIn'
        JointIds        % cell array of 3 joint ID strings
        PhalanxIds      % cell array of 3 phalanx ID strings
    end

    methods
        function obj = FingerController(fingerName, configPath)
            obj.FingerName = fingerName;
            obj.Simulator = FingerSimulator(configPath);
            obj.JointIds = obj.assignJointIds(fingerName);
            obj.PhalanxIds = obj.assignPhalanxIds(fingerName);

            % visual digital twin initialization (Inventor)
            obj.Simulator = obj.Simulator.setBufferManagerLibraryPath('D:\Repositorios_Politecnica\TesisProj\BufferManager\obj\Debug\BufferManager.dll');
        end

        function jointIds = assignJointIds(~, fingerName)
            switch fingerName
                case "RIn"
                    jointIds = {"RInPxJ", "RInMdJ", "RInDsJ"};
                case "RMd"
                    jointIds = {"RMdPxJ", "RMdMdJ", "RMdDsJ"};
                case "RRn"
                    jointIds = {"RRnPxJ", "RRnMdJ", "RRnDsJ"};
                case "RPk"
                    jointIds = {"RPkPxJ", "RPkMdJ", "RPkDsJ"};
                case "RTh"
                    jointIds = {"RThB__", "RThPxJ", "RThMdJ", "RThDsJ"};  % if thumb has only 3 joints
                case "LIn"
                    jointIds = {"LInPxJ", "LInMdJ", "LInDsJ"};
                case "LMd"
                    jointIds = {"LMdPxJ", "LMdMdJ", "LMdDsJ"};
                case "LRn"
                    jointIds = {"LRnPxJ", "LRnMdJ", "LRnDsJ"};
                case "LPk"
                    jointIds = {"LPkPxJ", "LPkMdJ", "LPkDsJ"};
                case "LTh"
                    jointIds = {"LThB__", "LThPxJ", "LThMdJ", "LThDsJ"};
                otherwise
                    error("Unknown finger name: %s", fingerName);
            end
        end

        function phanlanxIds = assignPhalanxIds(~, fingerName)
            switch fingerName
                case "RIn"
                    phanlanxIds = {"proximalIndex", "middleIndex", "distalIndex"};
                case "RMd"
                    phanlanxIds = {"proximalMiddle", "middleMiddle", "distalMiddle"};
                case "RRn"
                    phanlanxIds = {"proximalRing", "middleRing", "distalRing"};
                case "RPk"
                    phanlanxIds = {"proximalPicky", "middlePicky", "distalPicky"};
                case "RTh"
                    phanlanxIds = {"RThB__", "RThPxJ", "RThMdJ", "RThDsJ"};  % if thumb has only 3 joints
                case "LIn"
                    phanlanxIds = {"LInPxJ", "LInMdJ", "LInDsJ"};
                case "LMd"
                    phanlanxIds = {"LMdPxJ", "LMdMdJ", "LMdDsJ"};
                case "LRn"
                    phanlanxIds = {"LRnPxJ", "LRnMdJ", "LRnDsJ"};
                case "LPk"
                    phanlanxIds = {"LPkPxJ", "LPkMdJ", "LPkDsJ"};
                case "LTh"
                    phanlanxIds = {"LThB__", "LThPxJ", "LThMdJ", "LThDsJ"};
                otherwise
                    error("Unknown phanlanx name: %s", fingerName);
            end
        end

        function obj = run(obj, TMag, th1, th2, th3, o1, o2, o3, a1, a2, a3, N, dt)
            obj.Simulator = obj.Simulator.runSimulation(TMag, th1, th2, th3, o1, o2, o3, a1, a2, a3, N, dt, obj.PhalanxIds);

            % print results
            % obj.Simulator.results
        end

        function printJoints(obj)
            fprintf('Finger %s has joints:\n', obj.FingerName);
            disp(obj.JointIds);
        end

        function digitalTwinImpresion(obj, thetaPoximal, thetaMiddle, thetaDistal, thetaThumb)
            if obj.FingerName == "RTh" || obj.FingerName == "LTh"
                obj.Simulator.drawStepDigitalTwin("writte",obj.JointIds{1},thetaPoximal);
                obj.Simulator.drawStepDigitalTwin("writte",obj.JointIds{2},thetaMiddle);
                obj.Simulator.drawStepDigitalTwin("writte",obj.JointIds{3},thetaDistal);
                obj.Simulator.drawStepDigitalTwin("writte",obj.JointIds{4},thetaThumb);
            else
                obj.Simulator.drawStepDigitalTwin("writte",obj.JointIds{1},thetaPoximal);
                obj.Simulator.drawStepDigitalTwin("writte",obj.JointIds{2},thetaMiddle);
                obj.Simulator.drawStepDigitalTwin("writte",obj.JointIds{3},thetaDistal);
            end
        end
    end

end