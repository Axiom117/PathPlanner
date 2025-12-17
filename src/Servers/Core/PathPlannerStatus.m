classdef PathPlannerStatus < handle
% PathPlannerStatus - Status Monitor and Manual Control Module
    %
    % This module acts as the bridge between the digital world (the 'config' object) 
    % and the physical controller hardware. It has two core responsibilities:
    %
    % 1. Monitoring (Synchronous): It queries the hardware for its real-time
    %    position (e.g., getStatus). It then writes this "physical truth" 
    %    back into the shared 'config' module, ensuring the system's 
    %    "Single Source of Truth" is always synchronized with reality.
    %
    % 2. Operating (Asynchronous): It sends simple, non-blocking manual movement 
    %    commands (e.g., stepMove, homePosition) to the hardware and listens 
    %    for asynchronous completion notifications (e.g., 'STEP_COMPLETED').
    
    properties (Access = private)
        comm                            % Communication module reference
    end

    properties (Access = public)
        config                          % Configuration module reference
    end
    
    events
        StatusUpdate                    % Status operation updates
    end
    
    methods (Access = public)
        function obj = PathPlannerStatus(comm, config)
            % Constructor - Initialize status module with dependencies
            
            obj.comm = comm;
            obj.config = config;

            % Listen to communication events for async message handling
            % All async messages marked 'MessageReceived' will be catched
            % and processed using handleMessage method
            addlistener(obj.comm, 'MessageReceived', @obj.handleMessage);
        end
        
        function getStatus(obj)
            % Retrieve current status of manipulators
            % Sends synchronous GET_STATUS command and parses response
            
            % Get the selected manipulator IDs from the Config manager
            id1 = obj.config.manipulatorID1;
            id2 = obj.config.manipulatorID2;
            
            % Send synchronous status request
            command = sprintf('GET_STATUS, %s, %s', id1, id2);
            response = obj.comm.sendCommandSync(command);
            
            % Check for error response
            if contains(response, 'ERROR')
                error('PathPlannerStatus:GetStatusFailed', ...
                    'GetStatus failed: %s', response);
            end
            
            % Parse response format: STATUS, id1, X1, Y1, Z1, id2, X2, Y2, Z2
            tokens = strsplit(response, ',');

            % Remove leading and trailing whitespace
            tokens = strtrim(tokens);
            
            if isempty(tokens) || ~strcmp(tokens{1}, 'STATUS')
                error('PathPlannerStatus:InvalidResponse', ...
                    'Invalid status response header: %s', response);
            end
            
            % Dynamically parse status data trunks
            numTokens = length(tokens);
            i = 2;

            % Update positional parameters in config
            while i <= numTokens - 3
                currentID = tokens{i};
                valX = str2double(tokens{i+1});
                valY = str2double(tokens{i+2});
                valZ = str2double(tokens{i+3});

                % Match ID with the corresponding properties in Config
                if strcmp(currentID, id1)
                    obj.config.XMC1 = valX;
                    obj.config.YMC1 = valY;
                    obj.config.ZMC1 = valZ;
                
                elseif strcmp(currentID, id2)
                    obj.config.XMC2 = valX;
                    obj.config.YMC2 = valY;
                    obj.config.ZMC2 = valZ;

                else
                    msg = fprintf('Warning: Received status for unknown ID: %s\n', currentID);
                    notify(obj, 'StatusUpdate', PathPlannerEventData(msg))
                end
                
                % Move to the next trunk
                i = i + 4;
            end
            
            notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData('Status retrieved successfully'));
            
        end

        function [pose, elapsedTime] = getPose(obj)
            % Use forward kinematics model to compute the resulting pose
            % given the input of manipulator displacements

            modelFK = obj.config.modelFK;

            % Get current manipulator status and reflect to config
            obj.getStatus();

            % Build joint input vector [x1 y1 z1 x2 y2 z2] and convert μm to m
            [pose, elapsedTime] = onCalcFK(obj, modelFK);
            
            % Update the pose status in config
            obj.config.X0 = pose(1);
            obj.config.Y0 = pose(2);
            obj.config.Z0 = pose(3);
            obj.config.Phi0 = pose(4);
            obj.config.Theta0 = pose(5);
            obj.config.Psi0 = pose(6);
        end
        
        function success = stepMove(obj, id1, id2, X, Y, Z)
            % Execute incremental manipulator movement
            % Sends asynchronous STEP command for specified displacement
            
            % Validate input parameters
            if nargin < 6
                error('PathPlannerStatus:InsufficientArgs', ...
                    'StepMove requires manipulator IDs and X,Y,Z displacement values');
            end
            
            % Format step movement command
            command = sprintf('START_STEP, %s, %s, %.2f, %.2f, %.2f', ...
                id1, id2, X, Y, Z);
            
            try
                % Send asynchronous step command
                obj.comm.sendCommand(command);
                success = true;
                
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData('Step movement command sent'));
                
            catch ME
                success = false;
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData(['StepMove failed: ' ME.message], false));
            end
        end
        
        function success = stepMoveBatch(obj, id1, d1, id2, d2)
            % Execute simultaneous incremental movement for multiple manipulators
            %
            % Inputs:
            %   id1, id2 - Manipulator identifiers
            %   d1, d2   - Displacement vectors [x, y, z] in microns
            
            % Validate input dimensions
            if length(d1) < 3 || length(d2) < 3
                error('PathPlannerStatus:InvalidArgs', 'Displacement vectors must have 3 elements');
            end
            
            % Format the batch command (Protocol v1.1 updated)
            % Format: START_STEP, id1, x1, y1, z1, id2, x2, y2, z2
            command = sprintf('START_STEP, %s, %.0f, %.0f, %.0f, %s, %.0f, %.0f, %.0f', ...
                id1, d1(1), d1(2), d1(3), ...
                id2, d2(1), d2(2), d2(3));
            
            try
                % Send asynchronous command
                obj.comm.sendCommand(command);

                success = true;
                
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData(sprintf('Batch step command sent for %s and %s', id1, id2)));
                
            catch ME
                success = false;
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData(['Batch step failed: ' ME.message], false));
            end
        end
    end
    
    methods (Access = private)
        function handleMessage(obj, ~, eventData)
            % Handle asynchronous messages from communication module
            % Processes status-related controller responses
            %
            % Inputs:
            %   eventData - PathPlannerEventData containing message
            
            message = eventData.Message;
            
            % Process step completion notifications
            if contains(message, 'STEP_COMPLETED')
                obj.handleStepCompleted(message);
                
            elseif contains(message, 'STATUS,')
                obj.handleStatusUpdate(message);
            end
        end
        
        function handleStepCompleted(obj, message)
            % Handle step movement completion notification
            %
            % Inputs:
            %   message - Step completion message from controller
            
            notify(obj, 'StatusUpdate', ...
                PathPlannerEventData(['Step movement completed: ' message]));
        end
        
        function handleStatusUpdate(obj, message)
            % Handle unsolicited status update messages from controller
            % These may be sent during operations for monitoring
            %
            % Inputs:
            %   message - Status update message from controller
            
            try
                % Parse status message for validation
                tokens = strsplit(message, ',');
                tokens = strtrim(tokens);
                
                if length(tokens) >= 9 && strcmp(tokens{1}, 'STATUS')
                    notify(obj, 'StatusUpdate', ...
                        PathPlannerEventData('Unsolicited status update received'));
                else
                    notify(obj, 'StatusUpdate', ...
                        PathPlannerEventData('Received malformed status update'));
                end
                
            catch ME
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData(['Failed to parse status update: ' message], false));
            end
        end
    end
end