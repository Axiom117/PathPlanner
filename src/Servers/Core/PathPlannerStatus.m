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
            %
            % Use default IDs from configuration if not provided

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
            
            if length(tokens) >= 9 && strcmp(tokens{1}, 'STATUS')
                % Update positional parameters in config

                % Update for manipulator 1
                obj.config.XMC1 = str2double(tokens{3});
                obj.config.YMC1 = str2double(tokens{4});
                obj.config.ZMC1 = str2double(tokens{5});

                % Update for manipulator 2
                obj.config.XMC2 = str2double(tokens{7});
                obj.config.YMC2 = str2double(tokens{8});
                obj.config.ZMC2 = str2double(tokens{9});
                
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData('Status retrieved successfully'));
                    
            else
                error('PathPlannerStatus:InvalidResponse', ...
                    'Invalid status response format. Expected 9 tokens, got %d', length(tokens));
            end
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
        
        function success = stepMoveSingle(obj, id, X, Y, Z)
            % Execute incremental movement for single manipulator
            % Convenience method for single manipulator step movements
            %
            % Inputs:
            %   id      - Manipulator identifier
            %   X, Y, Z - Incremental displacement [μm]
            %
            % Returns:
            %   success - Boolean indicating command transmission success
            
            % Validate input parameters
            if nargin < 5
                error('PathPlannerStatus:InsufficientArgs', ...
                    'stepMoveSingle requires manipulator ID and X,Y,Z displacement values');
            end
            
            % Format single manipulator step command
            command = sprintf('START_STEP, %s, %.2f, %.2f, %.2f', id, X, Y, Z);
            
            try
                % Send asynchronous step command
                obj.comm.sendCommand(command);
                success = true;
                
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData(['Single step movement command sent for ' id]));
                
            catch ME
                success = false;
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData(['Single StepMove failed: ' ME.message], false));
            end
        end
        
        function success = homePosition(obj, id1, id2)
            % Move manipulators to home position (0,0,0)
            % Convenience method for returning to origin
            
            % Use default IDs if not provided
            if nargin < 2
                id1 = obj.config.manipulatorID1;
                id2 = obj.config.manipulatorID2;
            end
            
            try 
                % Calculate displacement to home position (0,0,0)
                deltaX1 = -obj.config.XMC1;
                deltaY1 = -obj.config.YMC1;
                deltaZ1 = -obj.config.ZMC1;
                
                deltaX2 = -obj.config.XMC2;
                deltaY2 = -obj.config.YMC2;
                deltaZ2 = -obj.config.ZMC2;
                
                % Send step movement to home position
                % Note: This assumes both manipulators can move independently
                success1 = obj.stepMoveSingle(id1, deltaX1, deltaY1, deltaZ1);
                success2 = obj.stepMoveSingle(id2, deltaX2, deltaY2, deltaZ2);
                
                success = success1 && success2;
                
                if success
                    notify(obj, 'StatusUpdate', ...
                        PathPlannerEventData('Home position commands sent'));
                else
                    notify(obj, 'StatusUpdate', ...
                        PathPlannerEventData('Home position command failed', false));
                end
                
            catch ME
                success = false;
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData(['Home position failed: ' ME.message], false));
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