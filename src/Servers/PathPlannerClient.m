classdef PathPlannerClient < handle
    % PathPlannerClient - TCP client with integrated configuration management
    % Implements PathPlanner API v1.1 with automatic parameter loading and
    % support for external FK/IK calculation functions and asynchronous execution
    %
    % Features:
    %   - Automatic configuration loading from ConfigManager
    %   - Event-driven architecture for status updates
    %   - Asynchronous path execution with real-time monitoring
    %   - Clean workspace management
    %   - Comprehensive error handling and retry mechanisms
    %
    % Usage:
    %   client = PathPlannerClient();                    % Auto-load all configs
    %   client.connect();
    %   success = client.PlanPath('MC1', 'MC2', 'model_2R_RCM_IK');
    
    properties (Access = private)
        tcpClient               % TCP client object for server communication
        messageListener         % Timer for monitoring incoming async messages
        trajectoryReady = false % Flag indicating if trajectory is ready for execution
        lastTrajectoryData      % Cache of last calculated trajectory data
        isExecutingPath = false % Flag indicating active path execution
        configManager           % Configuration manager instance
    end
    
    properties (SetAccess = private)
        isConnected = false     % Connection status (readable externally)
    end
    
    properties (Access = public)
        % Communication parameters (loaded from configuration)
        controllerHost          % Controller server host IP address
        controllerPort          % Controller server port number
        connectionTimeout       % TCP connection timeout [s]
        responseTimeout         % Command response timeout [s]
        maxRetryAttempts        % Maximum number of connection retry attempts
        
        % Manipulator identification (loaded from configuration)
        manipulatorID1          % Primary manipulator identifier
        manipulatorID2          % Secondary manipulator identifier
        
        % Current end-effector state (in mm and degrees)
        X0 = 0                  % Current X position [mm]
        Y0 = 0                  % Current Y position [mm]
        Z0 = 0                  % Current Z position [mm]
        Phi0 = 0                % Current Phi angle [degrees]
        Theta0 = 0              % Current Theta angle [degrees]
        Psi0 = 0                % Current Psi angle [degrees]
        
        % Target pose references (for onCalcIK compatibility)
        XTarget                 % Reference to target X field
        YTarget                 % Reference to target Y field
        ZTarget                 % Reference to target Z field
        PhiTarget               % Reference to target Phi field
        
        % Trajectory data storage
        trajectoryData = []     % Joint trajectory data (Nx6 matrix)
        trajectoryTime = []     % Time vector for trajectory
    end
    
    events
        StatusUpdate            % General status and communication updates
        TrajectoryReady         % Fired when trajectory planning completes
        TrajectoryExecuted      % Fired when trajectory execution completes
        ConnectionStateChanged  % Fired on connection/disconnection events
        PathDataReceived
        PathExecutionStarted    % Fired when path execution begins
        PathExecutionFailed     % Fired when path execution encounters errors
        ConfigurationLoaded     % Fired when configuration parameters are loaded
    end
    
    methods (Access = public)
        function obj = PathPlannerClient()
            % Constructor - Initialize client with automatic configuration loading
            % Automatically loads all configuration parameters and initializes the client
            
            % Initialize configuration and load all parameters
            obj.initializeConfiguration();
            
            % Initialize connection state
            obj.isConnected = false;
            obj.trajectoryReady = false;
            obj.isExecutingPath = false;
            
            % Notify successful initialization
            notify(obj, 'ConfigurationLoaded', ...
                PathPlannerEventData('PathPlannerClient initialized with configuration parameters'));
        end
        
        function success = connect(obj)
            % Establish TCP connection to controller server
            % Uses configured timeout and retry parameters
            %
            % Returns:
            %   success - Boolean indicating successful connection
            
            try
                % Create TCP client with configured timeout settings
                obj.tcpClient = tcpclient(obj.controllerHost, obj.controllerPort, ...
                    'Timeout', obj.responseTimeout, 'ConnectTimeout', obj.connectionTimeout);
                
                obj.isConnected = true;
                obj.startMessageListener();
                
                % Notify connection events
                notify(obj, 'ConnectionStateChanged');
                notify(obj, 'StatusUpdate', PathPlannerEventData(...
                    sprintf('Connected to controller at %s:%d', obj.controllerHost, obj.controllerPort)));
                
                % Verify connection with heartbeat
                if obj.sendHeartbeat()
                    success = true;
                else
                    obj.disconnect();
                    success = false;
                    notify(obj, 'StatusUpdate', PathPlannerEventData('Heartbeat verification failed'));
                end
                
            catch ME
                obj.isConnected = false;
                success = false;
                notify(obj, 'StatusUpdate', PathPlannerEventData(['Connection failed: ' ME.message]));
                
                % Attempt retry if configured
                if obj.maxRetryAttempts > 0
                    obj.attemptReconnection();
                end
            end
        end
        
        function disconnect(obj)
            % Close TCP connection and cleanup resources
            if obj.isConnected
                obj.stopMessageListener();
                
                if ~isempty(obj.tcpClient) && isvalid(obj.tcpClient)
                    delete(obj.tcpClient);
                end
                obj.tcpClient = [];
                obj.isConnected = false;
                obj.isExecutingPath = false;
                
                notify(obj, 'ConnectionStateChanged');
                notify(obj, 'StatusUpdate', PathPlannerEventData('Disconnected from controller server'));
            end
        end
        
        function [status1, status2] = GetStatus(obj, id1, id2)
            % Retrieve current status of manipulators
            % Uses configured manipulator IDs if not specified
            %
            % Inputs:
            %   id1, id2 - (optional) Manipulator identifiers
            %
            % Returns:
            %   status1, status2 - Structures containing position data [μm]
            
            if nargin < 2
                id1 = obj.manipulatorID1;
                id2 = obj.manipulatorID2;
            end
            
            command = sprintf('GET_STATUS, %s, %s', id1, id2);
            response = obj.sendCommandSync(command);
            
            if contains(response, 'ERROR')
                error('GetStatus failed: %s', response);
            end
            
            % Parse response: STATUS, id1, X1, Y1, Z1, id2, X2, Y2, Z2
            tokens = strsplit(response, ',');
            tokens = strtrim(tokens);
            
            if length(tokens) >= 9 && strcmp(tokens{1}, 'STATUS')
                status1 = struct('id', tokens{2}, ...
                    'X', str2double(tokens{3}), ...
                    'Y', str2double(tokens{4}), ...
                    'Z', str2double(tokens{5}));
                
                status2 = struct('id', tokens{6}, ...
                    'X', str2double(tokens{7}), ...
                    'Y', str2double(tokens{8}), ...
                    'Z', str2double(tokens{9}));
            else
                error('Invalid status response format. Expected 9 tokens, got %d', length(tokens));
            end
        end
        
        function success = StepMove(obj, id1, id2, X, Y, Z)
            % Execute incremental manipulator movement
            %
            % Inputs:
            %   id1, id2     - Manipulator identifiers
            %   X, Y, Z      - Incremental displacement [μm]
            %
            % Returns:
            %   success - Boolean indicating successful execution
            
            command = sprintf('START_STEP, %s, %s, %.2f, %.2f, %.2f', id1, id2, X, Y, Z);
            
            try
                obj.sendCommand(command);
                success = true;
                notify(obj, 'StatusUpdate', PathPlannerEventData('Step movement command sent'));
            catch ME
                success = false;
                notify(obj, 'StatusUpdate', PathPlannerEventData(['StepMove failed: ' ME.message]));
            end
        end
        
        function success = PlanPath(obj, id1, id2, model)
            % Plan trajectory using inverse kinematics solver
            % Uses configured simulation parameters and external IK function
            %
            % Inputs:
            %   id1, id2 - (optional) Manipulator identifiers
            %   model    - (optional) IK model name
            %
            % Returns:
            %   success - Boolean indicating successful trajectory generation
            
            % Use configured defaults if parameters not specified
            if nargin < 2
                id1 = obj.manipulatorID1;
                id2 = obj.manipulatorID2;
            end
            
            try
                notify(obj, 'StatusUpdate', PathPlannerEventData('Computing trajectory using inverse kinematics...'));
                
                % Execute external IK calculation function
                [qTime, qData, elapsedTime] = onCalcIK(obj, id1, id2, model);
                
                disp(qData);

                % Cache trajectory data for execution
                obj.lastTrajectoryData = struct();
                obj.lastTrajectoryData.id1 = id1;
                obj.lastTrajectoryData.id2 = id2;
                obj.lastTrajectoryData.qTime = qTime;
                obj.lastTrajectoryData.qData = qData;
                obj.lastTrajectoryData.elapsedTime = elapsedTime;
                
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
                notify(obj, 'StatusUpdate', PathPlannerEventData(['Trajectory planning failed: ' ME.message]));
            end
        end
        
        function success = SendPath(obj, id1, id2)
            % Transmit planned trajectory to controller server
            % Converts trajectory data to controller-compatible format
            %
            % Inputs:
            %   id1, id2 - (optional) Manipulator identifiers
            %
            % Returns:
            %   success - Boolean indicating successful data transmission
            
            if nargin < 2
                id1 = obj.manipulatorID1;
                id2 = obj.manipulatorID2;
            end
            
            try
                if ~obj.isConnected
                    error('No active connection to controller server');
                end
                
                if ~obj.trajectoryReady || isempty(obj.lastTrajectoryData)
                    error('No trajectory data available for transmission');
                end
                
                % Prepare PATH_DATA command
                pathDataStr = sprintf('PATH_DATA, %s, %s', id1, id2);
                qData = obj.lastTrajectoryData.qData;
                
                % Validate trajectory data dimensions
                if size(qData, 2) < 6
                    error('Invalid trajectory data: expected 6 columns, found %d', size(qData, 2));
                end
                
                % Convert units if necessary (meters to micrometers)
                if max(abs(qData(:))) < 1
                    qData = qData * 1e6;
                    notify(obj, 'StatusUpdate', PathPlannerEventData('Converting trajectory units: m → μm'));
                end
                
                % Serialize trajectory data
                for i = 1:size(qData, 1)
                    pathDataStr = sprintf('%s, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f', ...
                        pathDataStr, qData(i, 1:6));
                end
                
                % Transmit to controller
                obj.sendCommand(pathDataStr);
                success = true;
                notify(obj, 'StatusUpdate', PathPlannerEventData('Trajectory data transmission initiated'));
                
            catch ME
                success = false;
                notify(obj, 'StatusUpdate', PathPlannerEventData(['Trajectory transmission error: ' ME.message]));
            end
        end
        
        function success = ExecutePath(obj, id1, id2)
            % Initiate asynchronous trajectory execution on controller
            % Execution status is monitored via event notifications
            %
            % Inputs:
            %   id1, id2 - (optional) Manipulator identifiers
            %
            % Returns:
            %   success - Boolean indicating successful command transmission
            
            if nargin < 2
                id1 = obj.manipulatorID1;
                id2 = obj.manipulatorID2;
            end
            
            try
                if ~obj.isConnected
                    error('No active connection to controller server');
                end
                
                if obj.isExecutingPath
                    error('Path execution already in progress');
                end
                
                obj.isExecutingPath = true;
                
                % Send asynchronous execution command
                command = sprintf('START_PATH, %s, %s', id1, id2);
                obj.sendCommand(command);
                
                success = true;
                notify(obj, 'StatusUpdate', PathPlannerEventData('Path execution initiated'));
                
            catch ME
                obj.isExecutingPath = false;
                success = false;
                notify(obj, 'StatusUpdate', PathPlannerEventData(['Path execution error: ' ME.message]));
            end
        end
        
        function success = sendHeartbeat(obj)
            % Send heartbeat message to verify connection integrity
            response = obj.sendCommandSync('HEARTBEAT', 2); % timeout after 2 seconds
            success = contains(response, 'HEARTBEAT_OK');

            if ~success
                notify(obj, 'StatusUpdate', PathPlannerEventData('Heartbeat failed'));
            end
        end
        
        % Status query methods
        function ready = isTrajectoryReady(obj)
            ready = obj.trajectoryReady;
        end
        
        function data = getLastTrajectoryData(obj)
            data = obj.lastTrajectoryData;
        end
        
        function executing = isPathExecuting(obj)
            executing = obj.isExecutingPath;
        end
        
        function params = getConfigurationSummary(obj)
            % Get summary of current configuration parameters
            params = struct();
            params.communication = struct('host', obj.controllerHost, 'port', obj.controllerPort);
            params.manipulators = struct('id1', obj.manipulatorID1, 'id2', obj.manipulatorID2);
        end
        
        function delete(obj)
            % Destructor - Clean up resources and close connections
            obj.disconnect();
        end
    end
    
    methods (Access = private)
        function initializeConfiguration(obj)
            % Initialize configuration manager and load all parameters
            try
                obj.configManager = ConfigManager.getInstance();
                
                % Load all configuration categories
                obj.configManager.loadAllConfigs();
                obj.loadParametersFromConfig();
                
            catch ME
                warning('ConfigManager:LoadFailed', ...
                    'Configuration loading failed: %s. Using default parameters.', ME.message);
                obj.setDefaultParameters();
            end
        end
        
        function loadParametersFromConfig(obj)
            % Load parameters from configuration manager into object properties
            
            % Communication parameters
            obj.controllerHost = obj.configManager.getParam('communication', 'controllerHost');
            obj.controllerPort = obj.configManager.getParam('communication', 'controllerPort');
            obj.connectionTimeout = obj.configManager.getParam('communication', 'connectionTimeout');
            obj.responseTimeout = obj.configManager.getParam('communication', 'responseTimeout');
            obj.maxRetryAttempts = obj.configManager.getParam('communication', 'maxRetryAttempts');
            
            % Controller parameters
            obj.manipulatorID1 = obj.configManager.getParam('controller', 'manipulatorID1');
            obj.manipulatorID2 = obj.configManager.getParam('controller', 'manipulatorID2');
            
            % Initial pose
            obj.X0 = obj.configManager.getParam('simulation', 'X0');
            obj.Y0 = obj.configManager.getParam('simulation', 'Y0');
            obj.Z0 = obj.configManager.getParam('simulation', 'Z0');
            obj.Phi0 = obj.configManager.getParam('simulation', 'Phi0');
            obj.Theta0 = obj.configManager.getParam('simulation', 'Theta0');
            obj.Psi0 = obj.configManager.getParam('simulation', 'Psi0');
        end
        
        function setDefaultParameters(obj)
            % Set fallback default parameters if configuration loading fails
            obj.controllerHost = '127.0.0.1';
            obj.controllerPort = 5000;
            obj.connectionTimeout = 10;
            obj.responseTimeout = 1;
            obj.maxRetryAttempts = 3;
            obj.manipulatorID1 = 'MC1';
            obj.manipulatorID2 = 'MC2';
        end
        
        function attemptReconnection(obj)
            % Attempt automatic reconnection with configured retry limits
            for attempt = 1:obj.maxRetryAttempts
                notify(obj, 'StatusUpdate', PathPlannerEventData(...
                    sprintf('Reconnection attempt %d/%d', attempt, obj.maxRetryAttempts)));
                
                pause(1);  % Brief delay between attempts
                
                if obj.connect()
                    notify(obj, 'StatusUpdate', PathPlannerEventData('Automatic reconnection successful'));
                    return;
                end
            end
            
            notify(obj, 'StatusUpdate', PathPlannerEventData('Automatic reconnection failed'));
        end
        
        function sendCommand(obj, command)
            % Send command asynchronously - all responses handled by messageListener
            if ~obj.isConnected
                error('No active connection to controller server');
            end
            
            try
                % Ensure message listener is running
                obj.ensureMessageListenerRunning();
                
                % Send command
                write(obj.tcpClient, uint8(command));
                notify(obj, 'StatusUpdate', PathPlannerEventData(['→ ' command]));
                
            catch ME
                notify(obj, 'StatusUpdate', PathPlannerEventData(['Command error: ' ME.message]));
                rethrow(ME);
            end
        end

        function response = sendCommandSync(obj, command, timeoutSeconds)
            % Send command and wait for synchronous response
            % Use this for commands that require instant response
            % (GET_STATUS, HEARTBEAT, etc)
            %
            % Inputs:
            %   command - Command string to send
            %   timeoutSeconds - (optional) Timeout in seconds, default 5
            %
            % Returns:
            %   response - Response string from server
            
            if nargin < 3
                timeoutSeconds = obj.responseTimeout;
            end

            if ~obj.isConnected
                error('No active connection to controller server');
            end
            
            try
                % Temporarily stop message listener in case of confliction
                wasListenerRunning = false;
                if ~isempty(obj.messageListener) && isvalid(obj.messageListener)
                    wasListenerRunning = strcmp(obj.messageListener.Running, 'on');
                    if wasListenerRunning, stop(obj.messageListener); end
                end
                
                % Clean up cache
                if obj.tcpClient.NumBytesAvailable > 0
                    read(obj.tcpClient, obj.tcpClient.NumBytesAvailable, 'uint8');
                end

                % Transmit command
                write(obj.tcpClient, uint8(command));
                notify(obj, 'StatusUpdate', PathPlannerEventData(['→ ' command]));
                
                % Wait for response
                startTime = tic;
                response = '';

                while toc(startTime) < timeoutSeconds
                    if obj.tcpClient.NumBytesAvailable > 0
                        responseBytes = read(obj.tcpClient, obj.tcpClient.NumBytesAvailable, 'uint8');
                        response = strtrim(char(responseBytes));
                        notify(obj, 'StatusUpdate', PathPlannerEventData(['← ' response ' (sync)']));
                        break;
                    end
                    pause(0.01); % 10ms interval
                end

                if isempty(response)
                    response = sprintf('ERROR, 105, Timeout waiting for response to: %s', command);
                end
                     
            catch ME
                response = sprintf('ERROR, 104, Exception during sync command: %s', ME.message);
                notify(obj, 'StatusUpdate', PathPlannerEventData(['Sync command error: ' ME.message]));
            end
            
            % Restart message listener
            if wasListenerRunning
                try
                    start(obj.messageListener);
                catch
                    obj.startMessageListener(); % Recreate object once failed starting
                end
            end
        end
        
        function startMessageListener(obj)
            % Initialize asynchronous message monitoring timer
            if ~isempty(obj.messageListener) && isvalid(obj.messageListener)
                stop(obj.messageListener);
                delete(obj.messageListener);
            end
            
            obj.messageListener = timer(...
                'ExecutionMode', 'fixedRate', ...
                'Period', 0.05, ...
                'TimerFcn', @(~,~)obj.checkIncomingMessages(), ...
                'ErrorFcn', @(~,~)obj.handleListenerError(), ...
                'Name', 'PathPlannerMessageListener');
            
            start(obj.messageListener);
            notify(obj, 'StatusUpdate', PathPlannerEventData('Message listener started'));
        end
        
        function ensureMessageListenerRunning(obj)
            if obj.isConnected
                if isempty(obj.messageListener) || ~isvalid(obj.messageListener)
                    obj.startMessageListener();
                elseif ~strcmp(obj.messageListener.Running, 'on')
                    try
                        start(obj.messageListener);
                    catch
                        obj.startMessageListener();
                    end
                end
            end
        end

        function stopMessageListener(obj)
            % Stop and clean up message monitoring timer
            if ~isempty(obj.messageListener) && isvalid(obj.messageListener)
                stop(obj.messageListener);
                delete(obj.messageListener);
                obj.messageListener = [];
            end
        end
        
        function checkIncomingMessages(obj)
            % Monitor and process all asynchronous messages from controller
            try
                if obj.isConnected && obj.tcpClient.NumBytesAvailable > 0
                    responseBytes = read(obj.tcpClient, obj.tcpClient.NumBytesAvailable, 'uint8');
                    response = strtrim(char(responseBytes));
                    
                    % Log all received messages
                    notify(obj, 'StatusUpdate', PathPlannerEventData(['← ' response]));
                    
                    % Handle response with multiple lines
                    responseLines = splitlines(string(response));
                    
                    for i = 1:length(responseLines)
                        line = char(strtrim(responseLines(i)));
                        if isempty(line)
                            continue;
                        end
                        
                        obj.processAsyncMessage(line);
                    end
                end
            catch ME
                if ~contains(ME.message, 'Invalid or deleted object')
                    notify(obj, 'StatusUpdate', PathPlannerEventData(['Message monitoring error: ' ME.message]));
                end
            end
        end

        function processAsyncMessage(obj, message)
            % Process different types of asynchronous messages
            
            % Path execution related messages
            if contains(message, 'PATH_TRACKING_STARTED')
                obj.handlePathTrackingStarted();
                
            elseif contains(message, 'PATH_COMPLETED') && obj.isExecutingPath
                obj.handlePathCompleted(message);
                
            elseif contains(message, 'PATH_DATA_RECEIVED')
                obj.handlePathDataReceived();
                
            elseif contains(message, 'STEP_COMPLETED')
                obj.handleStepCompleted(message);
                
            elseif contains(message, 'ERROR')
                obj.handleAsyncError(message);
                
            elseif contains(message, 'HEARTBEAT_OK')
                obj.handleHeartbeatResponse();
                
            elseif contains(message, 'STATUS,')
                obj.handleStatusUpdate(message);
                
            else
                % Unknown message type
                notify(obj, 'StatusUpdate', PathPlannerEventData(['Unknown async message: ' message]));
            end
        end
        
        function handleListenerError(obj)
            notify(obj, 'StatusUpdate', PathPlannerEventData('Message listener encountered an error'));
        end
        
        function handlePathTrackingStarted(obj)
            notify(obj, 'PathExecutionStarted', PathPlannerEventData('Path execution initiated on controller'));
        end

        function handlePathCompleted(obj, response)
            obj.isExecutingPath = false;
            
            % Extract manipulator IDs from response
            tokens = strsplit(response, ',');
            if length(tokens) >= 3
                message = sprintf('Path execution completed for %s and %s', strtrim(tokens{2}), strtrim(tokens{3}));
            else
                message = 'Path execution completed successfully';
            end
            
            notify(obj, 'TrajectoryExecuted', PathPlannerEventData(message));
        end

        function handlePathDataReceived(obj)
            notify(obj, 'PathDataReceived', PathPlannerEventData('Trajectory data confirmed received by controller'));
            notify(obj, 'StatusUpdate', PathPlannerEventData('Controller ready to execute trajectory'));
        end
        
        function handleStepCompleted(obj, message)
            notify(obj, 'StatusUpdate', PathPlannerEventData(['Step movement completed: ' message]));
        end
        
        function handleAsyncError(obj, message)
            if obj.isExecutingPath
                obj.isExecutingPath = false;
                notify(obj, 'PathExecutionFailed', PathPlannerEventData(['Path execution error: ' message]));
            else
                notify(obj, 'StatusUpdate', PathPlannerEventData(['Controller error: ' message]));
            end
        end
        
        function handleHeartbeatResponse(obj)
            % Heartbeat response for checking connection condition
            notify(obj, 'StatusUpdate', PathPlannerEventData('Heartbeat confirmed'));
        end
        function handlePathError(obj, response)
            obj.isExecutingPath = false;
            notify(obj, 'PathExecutionFailed', PathPlannerEventData(['Path execution error: ' response]));
        end

        function handleStatusUpdate(obj, message)
            try
                tokens = strsplit(message, ',');
                tokens = strtrim(tokens);
                
                if length(tokens) >= 9 && strcmp(tokens{1}, 'STATUS')
                    notify(obj, 'StatusUpdate', PathPlannerEventData('Status update received'));
                end
            catch
                notify(obj, 'StatusUpdate', PathPlannerEventData(['Failed to parse status: ' message]));
            end
        end
    end
end