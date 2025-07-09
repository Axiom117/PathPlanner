classdef PathPlannerTrajectory < handle
    % PathPlannerTrajectory - Trajectory management module
    % 
    % This module handles trajectory planning, data transmission, and execution
    % monitoring. It coordinates with the communication module for data transfer
    % and provides event notifications for trajectory status updates.
    
    properties (Access = private)
        status                          % Status management module
        comm                            % Communication module reference
        trajectoryReady = false         % Trajectory readiness flag
        lastTrajectoryData              % Cache of last calculated trajectory
        isExecuting = false             % Path execution status flag
    end

    properties (Access = public)
        config                          % Configuration module reference
    end
    
    properties (Access = public)
        trajectoryData = []             % Joint trajectory data (Nx6 matrix)
        trajectoryTime = []             % Time vector for trajectory
    end
    
    events
        StatusUpdate                    % General status updates
        TrajectoryReady                 % Trajectory planning completed
        TrajectoryExecuted              % Trajectory execution completed
        PathDataReceived                % Controller confirmed data receipt
        PathExecutionStarted            % Path execution initiated
        PathExecutionFailed             % Path execution failed
    end
    
    methods (Access = public)
        function obj = PathPlannerTrajectory(comm, config, status)
            % Constructor - Initialize trajectory module with dependencies
            %
            % Inputs:
            %   comm   - PathPlannerComm instance
            %   config - PathPlannerConfig instance
            
            obj.comm = comm;
            obj.config = config;
            obj.status = status;
            
            % Listen to communication events for async message handling
            addlistener(obj.comm, 'MessageReceived', @obj.handleMessage);
        end
        
        function success = planPath(obj)
            % Plan trajectory using inverse kinematics solver
            % Calls external onCalcIK function with current configuration
            %
            % Inputs:
            %   id1, id2 - (optional) Manipulator identifiers
            %   model    - (optional) IK model name
            %
            % Returns:
            %   success - Boolean indicating planning success
            
            targetPoints = 100;
            try
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData('Computing trajectory using inverse kinematics...'));
                
                % Refresh the current pose information by calling the getPose() method
                % from the referenced status module
                [~,~] = obj.status.getPose();

                % Execute external IK calculation function
                % Pass config object to provide access to all parameters
                [qTime, qData, elapsedTime] = onCalcIK(obj);

                originalPoints = size(qData, 1);
                
                if originalPoints > targetPoints
                    [downsampledData, downsampledTime, ~] = obj.downsampleTrajectory(qData, qTime, targetPoints);
                else
                    downsampledData = qData;
                end

                % Cache trajectory data for future operations
                obj.lastTrajectoryData = struct();
                obj.lastTrajectoryData.id1 = obj.config.manipulatorID1;
                obj.lastTrajectoryData.id2 = obj.config.manipulatorID2;
                obj.lastTrajectoryData.qTime = downsampledTime;
                obj.lastTrajectoryData.qData = downsampledData;
                obj.lastTrajectoryData.elapsedTime = elapsedTime;
                
                % Update public properties
                obj.trajectoryData = qData;
                obj.trajectoryTime = qTime;
                obj.trajectoryReady = true;
                success = true;
                
                % Notify trajectory completion with data
                eventData = PathPlannerEventData(...
                    sprintf('Trajectory computed successfully in %.3f seconds', elapsedTime), ...
                    true, obj.lastTrajectoryData);
                notify(obj, 'TrajectoryReady', eventData);
                
            catch ME
                obj.trajectoryReady = false;
                success = false;
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData(['Trajectory planning failed: ' ME.message], false));
            end
        end
        
        function success = sendPath(obj)
            % Transmit planned trajectory to controller server
            % Converts trajectory data to controller-compatible format
            %
            % Inputs:
            %   id1, id2 - (optional) Manipulator identifiers
            %
            % Returns:
            %   success - Boolean indicating transmission success
            
            % Use default IDs
            id1 = obj.config.manipulatorID1;
            id2 = obj.config.manipulatorID2;

            try
                % Validate prerequisites
                if ~obj.comm.getConnectionStatus()
                    error('No active connection to controller server');
                end
                
                if ~obj.trajectoryReady || isempty(obj.lastTrajectoryData)
                    error('No trajectory data available for transmission');
                end
                
                % convert from m to um for the controller input
                qData = obj.lastTrajectoryData.qData * 1e6;

                % Prepare PATH_DATA command header
                pathDataStr = sprintf('PATH_DATA, %s, %s', id1, id2);
                
                % Validate trajectory data structure
                if size(qData, 2) < 6
                    error('Invalid trajectory data: expected 6 columns, found %d', size(qData, 2));
                end
                
                % Serialize trajectory data points
                for i = 1:size(qData, 1)
                    pathDataStr = sprintf('%s, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f', ...
                        pathDataStr, qData(i, 1:6));
                end
                
                % Transmit to controller via communication module
                obj.comm.sendCommand(pathDataStr);
                success = true;
                
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData('Trajectory data transmission initiated'));
                
            catch ME
                success = false;
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData(['Trajectory transmission error: ' ME.message], false));
            end
        end
        
        function [downsampledData, downsampledTime, selectedIndices] = downsampleTrajectory(~, qData, qTime, targetPoints)
            % Uniform interval sampling
            numPoints = size(qData, 1);

            if targetPoints >= 2
                % Always include first and last points
                if targetPoints == 2
                    indices = [1, numPoints];
                else
                    % Distribute remaining points uniformly
                    middlePoints = targetPoints - 2;
                    step = (numPoints - 1) / (middlePoints + 1);
                    middleIndices = round(step * (1:middlePoints)) + 1;
                    indices = [1, middleIndices, numPoints];
                end
            else
                % Pure uniform sampling
                step = (numPoints - 1) / (targetPoints - 1);
                indices = round(1 + step * (0:targetPoints-1));
            end
            
            % Ensure unique indices and sort
            indices = unique(indices);
            indices = indices(indices >= 1 & indices <= numPoints);

            selectedIndices = indices;
            downsampledData = qData(selectedIndices, :);
            downsampledTime = qTime(selectedIndices);
        end

        function success = executePath(obj)
            % Initiate asynchronous trajectory execution on controller
            % Execution status monitored via async message handling
            %
            % Inputs:
            %   id1, id2 - (optional) Manipulator identifiers
            %
            % Returns:
            %   success - Boolean indicating command transmission success
            
            % Use default IDs
            id1 = obj.config.manipulatorID1;
            id2 = obj.config.manipulatorID2;
            
            try
                % Validate prerequisites
                if ~obj.comm.getConnectionStatus()
                    error('No active connection to controller server');
                end
                
                if obj.isExecuting
                    error('Path execution already in progress');
                end
                
                % Set execution flag and send command
                obj.isExecuting = true;
                command = sprintf('START_PATH, %s, %s', id1, id2);
                obj.comm.sendCommand(command);
                
                success = true;
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData('Path execution initiated'));
                
            catch ME
                obj.isExecuting = false;
                success = false;
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData(['Path execution error: ' ME.message], false));
            end
        end
        
        %% Status Query Methods
        function ready = isReady(obj)
            % Check if trajectory is ready for execution
            % Returns: ready - Boolean indicating trajectory readiness
            
            ready = obj.trajectoryReady;
        end
        
        function data = getLastData(obj)
            % Get last calculated trajectory data
            % Returns: data - Structure containing trajectory information
            
            data = obj.lastTrajectoryData;
        end
        
        function executing = isExecutingPath(obj)
            % Check if path execution is currently in progress
            % Returns: executing - Boolean indicating execution status
            
            executing = obj.isExecuting;
        end
        
        function clearTrajectory(obj)
            % Clear cached trajectory data and reset status
            
            obj.trajectoryData = [];
            obj.trajectoryTime = [];
            obj.lastTrajectoryData = [];
            obj.trajectoryReady = false;
            obj.isExecuting = false;
            
            notify(obj, 'StatusUpdate', ...
                PathPlannerEventData('Trajectory data cleared'));
        end
    end
    
    methods (Access = private)
        function handleMessage(obj, ~, eventData)
            % Handle asynchronous messages from communication module
            % Processes trajectory-related controller responses
            %
            % Inputs:
            %   eventData - PathPlannerEventData containing message
            
            message = eventData.Message;
            
            % Process different message types
            if contains(message, 'PATH_TRACKING_STARTED')
                obj.handlePathTrackingStarted();
                
            elseif contains(message, 'PATH_COMPLETED') && obj.isExecuting
                obj.handlePathCompleted(message);
                
            elseif contains(message, 'PATH_DATA_RECEIVED')
                obj.handlePathDataReceived();
                
            elseif contains(message, 'ERROR') && obj.isExecuting
                obj.handlePathError(message);
            end
        end
        
        function handlePathTrackingStarted(obj)
            % Handle path tracking started notification from controller
            
            notify(obj, 'PathExecutionStarted', ...
                PathPlannerEventData('Path execution initiated on controller'));
        end
        
        function handlePathCompleted(obj, response)
            % Handle path completion notification from controller
            %
            % Inputs:
            %   response - Controller response message
            
            obj.isExecuting = false;
            
            % Extract manipulator IDs from response for detailed message
            tokens = strsplit(response, ',');
            if length(tokens) >= 3
                message = sprintf('Path execution completed for %s and %s', ...
                    strtrim(tokens{2}), strtrim(tokens{3}));
            else
                message = 'Path execution completed successfully';
            end
            
            notify(obj, 'TrajectoryExecuted', PathPlannerEventData(message));
        end
        
        function handlePathDataReceived(obj)
            % Handle trajectory data receipt confirmation from controller
            
            notify(obj, 'PathDataReceived', ...
                PathPlannerEventData('Trajectory data confirmed received by controller'));
        end
        
        function handlePathError(obj, message)
            % Handle path execution error from controller
            %
            % Inputs:
            %   message - Error message from controller
            
            obj.isExecuting = false;
            notify(obj, 'PathExecutionFailed', ...
                PathPlannerEventData(['Path execution error: ' message], false));
        end
    end
end