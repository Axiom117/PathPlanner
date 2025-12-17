classdef PathPlannerTrajectory < handle
% PathPlannerTrajectory - The central "nervous system" for trajectory management
    %
    % This module acts as the bridge between the planning logic ("Brain") and the
    % robot controller ("Muscles"). It orchestrates the entire motion lifecycle through
    % three key responsibilities:
    %
    % 1. Planning (The Brain): 
    %    - Invokes the Inverse Kinematics (IK) solver (`onCalcIK`) to generate
    %      joint trajectories from the current pose to the target.
    %
    % 2. Transmission (The Messenger):
    %    - Formats calculated trajectory data (converting units from meters to micrometers).
    %    - Downsamples data if necessary for efficiency.
    %    - Transmits the payload to the robot controller via the communication module.
    %
    % 3. Monitoring (The Watchdog):
    %    - Issues execution commands (`START_PATH`) to the controller.
    %    - Asynchronously listens for feedback to track status: start, completion, or failure.
    %
    % By coordinating the Configuration, Status, and Communication modules, this class
    % drives the physical execution of paths and broadcasts real-time events to the UI.
    
    properties (Access = private)
        status                          % Status management module
        comm                            % Communication module reference
        trajectoryReady = false         % Trajectory readiness flag
        lastTrajectoryData              % Cache of last calculated trajectory
        isExecuting = false             % Path execution status flag

        isPTPMode = false               % Flag for Point-to-Point mode (Translation only)
        ptpDeltas = []                  % Cached increments for PTP move [dx1, dy1, dz1, dx2, dy2, dz2] (um)
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
            
            try
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData('Computing trajectory using inverse kinematics...'));
                
                % Refresh the current pose information by calling the getPose() method
                % from the referenced status module
                obj.status.getPose();
                
                % Get current and target roll angle Phi
                currentPhi = obj.config.Phi0;
                targetPhi = obj.config.PhiTarget;
                
                % Get current and target Position with unit conversion
                currentPos = [obj.config.X0, obj.config.Y0, obj.config.Z0];
                targetPos = [obj.config.XTarget, obj.config.YTarget, obj.config.ZTarget].* 10^3;

                % Calculate Deltas
                deltaPhi = abs(targetPhi - currentPhi);
                deltaDist = norm(targetPos - currentPos);
                
                % Detect PTP mode
                if deltaPhi < 0.01
                    obj.isPTPMode = true;
                    notify(obj, 'StatusUpdate', ...
                        PathPlannerEventData('Motion type: Translation only. Switching to PTP Mode (Single Step).'));
                else
                    obj.isPTPMode = false;
                end

                % Execute external IK calculation function
                % Pass config object to provide access to all parameters
                [qTime, qData, elapsedTime] = onCalcIK(obj);
               
                % Branch Logic based on Mode
                if obj.isPTPMode
                    % --- PTP Mode Logic ---
                    % Get Target Joint Positions (Last point of IK result) -> Convert to microns
                    targetJointsUm = qData(end, :) * 1e6;
                    
                    % Get Current Joint Positions (From Config) -> Microns
                    % [XMC1, YMC1, ZMC1, XMC2, YMC2, ZMC2]
                    currentJointsUm = [obj.config.XMC1, obj.config.YMC1, obj.config.ZMC1, ...
                                       obj.config.XMC2, obj.config.YMC2, obj.config.ZMC2];
                                   
                    % Calculate Incremental Steps (Delta)
                    obj.ptpDeltas = round(targetJointsUm - currentJointsUm);
                    
                    % For PTP, we don't need trajectory data for transmission, 
                    % but we keep qData for plotting/reference.
                    obj.lastTrajectoryData = []; 
                    
                else
                    % --- Trajectory Mode Logic (Original) ---
                    resPhi = 1; 
                    resDist = 0.1;
                    
                    pointsPhi = ceil(deltaPhi / resPhi);
                    pointsDist = ceil(deltaDist / resDist);
                    targetPoints = max([pointsPhi, pointsDist, 10]); % Min 10 points
                    
                    notify(obj, 'StatusUpdate', ...
                        PathPlannerEventData(sprintf('Dynamic sampling: Points=%d', targetPoints)));
                    
                    originalPoints = size(qData, 1);
                    if originalPoints > targetPoints
                        [downsampledData, downsampledTime, ~] = obj.downsampleTrajectory(qData, qTime, targetPoints);
                    else
                        downsampledData = qData;
                        downsampledTime = qTime;
                    end
                    
                    % Cache trajectory data
                    obj.lastTrajectoryData = struct();
                    obj.lastTrajectoryData.id1 = obj.config.manipulatorID1;
                    obj.lastTrajectoryData.id2 = obj.config.manipulatorID2;
                    obj.lastTrajectoryData.qTime = downsampledTime;
                    obj.lastTrajectoryData.elapsedTime = elapsedTime;
                    
                    % Unit conversion & Rounding
                    obj.lastTrajectoryData.qData = round(downsampledData * 1e6);
                end
                
                % Common Finalization
                obj.trajectoryData = qData;
                obj.trajectoryTime = qTime;
                obj.trajectoryReady = true;
                success = true;
                
                if obj.isPTPMode
                    modeStr = "PTP";
                else
                    modeStr = "Trajectory";
                end
                
                msg = sprintf('Planning successful (Mode: %s)', modeStr);
                
                notify(obj, 'TrajectoryReady', ...
                    PathPlannerEventData(msg, true, obj.lastTrajectoryData));
                
            catch ME
                obj.trajectoryReady = false;
                success = false;
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData(['Planning failed: ' ME.message], false));
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
            
            % Prepare IDs and path data

            if obj.isPTPMode
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData('PTP Mode: Skipping path transmission. Ready to execute step.'))
                success = true;
                notify(obj, 'PathDataReceived', ...
                    PathPlannerEventData('PTP Ready'));
                return;
            end

            % Standard Send Logic
            ids = {obj.config.manipulatorID1, obj.config.manipulatorID2};

            try
                % Validate prerequisites
                if ~obj.comm.getConnectionStatus()
                    error('No active connection to controller server');
                end
                
                if ~obj.trajectoryReady || isempty(obj.lastTrajectoryData)
                    error('No trajectory data available for transmission');
                end
                
                qData = obj.lastTrajectoryData.qData;
                
                % Validate trajectory data structure
                if size(qData, 2) < 6
                    error('Invalid trajectory data: expected 6 columns, found %d', size(qData, 2));
                end
                
                % Send trajectory points separately
                % Define the data index for each manipulator
                colMap = {[1, 2, 3], [4, 5, 6]};

                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData('Starting sequential trajectory transmission...'));
                
                for k = 1:length(ids)
                    currentID = ids{k};
                    currentCols = colMap{k};

                    % Extract the N x 3 data corresponding to current ID
                    manipulatorData = qData(:, currentCols);

                    % Construct PATH_DATA command string
                    % Format: PATH_DATA, ID, X, Y, Z, X, Y, Z...
                    pathDataStr = sprintf('PATH_DATA, %s', currentID);

                    % Serialize trajectory data points
                    for i = 1:size(manipulatorData, 1)
                        % Attention: don't forget to add comma beforehead
                        pathDataStr = sprintf('%s, %.0f, %.0f, %.0f', ...
                            pathDataStr, manipulatorData(i, 1), ...
                            manipulatorData(i, 2), ...
                            manipulatorData(i, 3));
                    end

                    % Transmit data string synchronously via communication
                    % module and wait for confirmation
                    timeout = 3;
                    response = obj.comm.sendCommandSync(pathDataStr, timeout);
    
                    % Validate response
                    % Controller should reply: PATH_DATA_RECEIVED, <ID>
                    expectedToken = 'PATH_DATA_RECEIVED';
                    
                    if contains(response, expectedToken)
                        notify(obj, 'StatusUpdate', ...
                            PathPlannerEventData(sprintf('Trajectory for %s sent and confirmed.', currentID)));
                    else
                        error('Failed to send path for %s. Server response: %s', currentID, response);
                    end
                end

                % All iterations finalized and no error
                success = true;
                notify(obj, 'PathDataReceived', ...
                    PathPlannerEventData('All trajectory data transmitted and confirmed.'));

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
            
            % Get manipulator IDs
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

                if obj.isPTPMode
                    % PTP Mode: Batch Step Execution
                    % ptpDeltas format: [dx1, dy1, dz1, dx2, dy2, dz2]
                    d = obj.ptpDeltas;
                    
                    if isempty(d) || length(d) < 6
                        error('Invalid PTP deltas');
                    end

                    notify(obj, 'StatusUpdate', ...
                        PathPlannerEventData('Executing PTP Batch Step Move...'));
                
                    % Send STEP command all at once
                    success = obj.status.stepMoveBatch(id1, d(1:3), id2, d(4:6));

                    if ~success
                        obj.isExecuting = false;
                        error('Failed to send batch step command');
                    end

                else
                    % Trajectory Mode: Path Tracking
                    command = sprintf('START_PATH_CP, %s, %s', id1, id2);
                    obj.comm.sendCommand(command);
                    success = true;
                
                    notify(obj, 'StatusUpdate', ...
                        PathPlannerEventData('Path execution initiated'));
                end

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