classdef PathPlannerComm < handle
%
% This module handles all TCP communication with the controller server,
% including connection management, command transmission, and asynchronous
% message monitoring.
%
% It provides both synchronous and asynchronous communication methods:
%
%   Synchronous Communication: Used for "request-response" (blocking) cases.
%     When MATLAB sends a command (e.g., 'HEARTBEAT'), it halts execution
%     and waits for a specific response from the server before continuing.
%
%   Asynchronous Communication: Used for "non-blocking" cases.
%     When MATLAB sends a command (e.g., 'START_MOTION'), it is not
%     blocked (stuck) waiting for a response. Instead, it can continue
%     executing other tasks.
%     Meanwhile, the server can actively push messages at any time
%     (e.g., 'REACHED_TARGET' or 'ERROR'), which are received by this
%     module's background message listener.
%
    
    properties (Access = private)
        tcpClient                       % TCP client object
        messageListener                 % Timer for async message monitoring
        config                          % Configuration module reference
        isConnected = false             % Internal connection status
        isReconnecting = false          % Flag to prevent recursive reconnection
    end
    
    events
        StatusUpdate                    % Communication status updates
        ConnectionStateChanged          % Connection state changes
        MessageReceived                 % Async messages from controller
    end
    
    methods (Access = public)
        function obj = PathPlannerComm(config)
            % Constructor - Initialize communication module with configuration
            %
            % Inputs:
            %   config - PathPlannerConfig instance
            
            obj.config = config;
        end
        
        function success = connect(obj, suppressReconnection)
            % Establish TCP connection to controller server
            % Returns: success - Boolean indicating connection status
            
            if nargin < 2
                suppressReconnection = false;
            end

            try
                % Create TCP client with configured settings and connect to
                % host server
                obj.tcpClient = tcpclient(obj.config.controllerHost, obj.config.controllerPort, ...
                    'Timeout', obj.config.responseTimeout, ...
                    'ConnectTimeout', obj.config.connectionTimeout);
                
                obj.isConnected = true;
                
                % Start the asynchronous listener with timer
                obj.startMessageListener();
                
                % Notify successful connection
                notify(obj, 'StatusUpdate', PathPlannerEventData(...
                    sprintf('Connected to controller at %s:%d', ...
                    obj.config.controllerHost, obj.config.controllerPort)));
                notify(obj, 'ConnectionStateChanged');
                
                % Verify connection with heartbeat
                if obj.sendHeartbeat()
                    success = true;
                else
                    obj.disconnect();
                    success = false;
                    notify(obj, 'StatusUpdate', ...
                        PathPlannerEventData('Heartbeat verification failed', false));
                end
                
            catch ME
                obj.isConnected = false;
                success = false;
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData(['Connection failed: ' ME.message], false));

                % Only attempt automatic reconnection if:
                % 1. Not already in a reconnection process
                % 2. Reconnection is not suppressed
                % 3. Max retry attempts is configured > 0
                % Attempt automatic reconnection if configured
                if ~obj.isReconnecting && ~suppressReconnection && obj.config.maxRetryAttempts > 0
                    obj.attemptReconnection();
                end
            end
        end
        
        function disconnect(obj)
            % Disconnect from controller server and cleanup resources
            
            try
                % Must stop timer in case it attempts to access terminated
                % tcpClient instance during next connection
                obj.stopMessageListener();
                
                % Terminate TCP client
                if ~isempty(obj.tcpClient) && isvalid(obj.tcpClient)
                    delete(obj.tcpClient);
                end
                obj.tcpClient = [];
                obj.isConnected = false;
                
                % Notify disconnection
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData('Disconnected from controller server'));
                notify(obj, 'ConnectionStateChanged');

            catch ME
                obj.isConnected = false;
                obj.tcpClient = [];
                warning(ME.identifier,'%s', ME.message);
            end
        end
        
        function success = sendHeartbeat(obj)
            % Send heartbeat message to verify connection integrity
            % Returns: success - Boolean indicating heartbeat response
            
            response = obj.sendCommandSync('HEARTBEAT', 2);
            success = contains(response, 'HEARTBEAT_OK');
            
            if ~success
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData('Heartbeat failed', false));
            end
        end
        
        function sendCommand(obj, command)
            % Send command asynchronously to controller
            % Response handling is done through message listener
            
            if ~obj.isConnected
                error('PathPlannerComm:NotConnected', ...
                    'No active connection to controller server');
            end
            
            try
                obj.ensureMessageListenerRunning();
                write(obj.tcpClient, uint8(command));
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData(['→ ' command]));
                
            catch ME
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData(['Command error: ' ME.message], false));
                rethrow(ME);
            end
        end
        
        function response = sendCommandSync(obj, command, timeoutSeconds)
            % Send command and wait for synchronous response
            % Use for commands requiring immediate response (GET_STATUS, HEARTBEAT)
            
            if nargin < 3
                timeoutSeconds = obj.config.responseTimeout;
            end
            
            if ~obj.isConnected
                error('PathPlannerComm:NotConnected', ...
                    'No active connection to controller server');
            end
            
            try
                % Temporarily pause message listener to avoid conflicts
                wasListenerRunning = obj.pauseMessageListener();
                
                % Clear any pending data in buffer
                if obj.tcpClient.NumBytesAvailable > 0
                    read(obj.tcpClient, obj.tcpClient.NumBytesAvailable, 'uint8');
                end
                
                % Send command
                write(obj.tcpClient, uint8(command));
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData(['→ ' command]));
                
                % Wait for response with timeout
                response = obj.waitForResponse(timeoutSeconds);
                
                % Resume message listener if it was running
                if wasListenerRunning
                    obj.resumeMessageListener();
                end
                
            catch ME
                response = sprintf('ERROR, 104, Exception during sync command: %s', ME.message);
                notify(obj, 'StatusUpdate', ...
                    PathPlannerEventData(['Sync command error: ' ME.message], false));
            end
        end
        
        function connected = getConnectionStatus(obj)
            % Get current connection status
            % Returns: connected - Boolean connection status
            
            connected = obj.isConnected;
        end
        
        function delete(obj)
            % Destructor - Clean up resources
            obj.disconnect();
        end
    end
    
    methods (Access = private)
        function response = waitForResponse(obj, timeoutSeconds)
            % Wait for response from server with timeout
            
            startTime = tic;
            response = '';
            
            while toc(startTime) < timeoutSeconds
                % Check is there any bytes from stream readable
                if obj.tcpClient.NumBytesAvailable > 0
                    responseBytes = read(obj.tcpClient, obj.tcpClient.NumBytesAvailable, 'uint8');
                    response = strtrim(char(responseBytes));
                    notify(obj, 'StatusUpdate', ...
                        PathPlannerEventData(['← ' response ' (sync)']));
                    break;
                end
                pause(0.01); % 10ms polling interval
            end
            
            if isempty(response)
                response = sprintf('ERROR, 105, Timeout waiting for response');
            end
        end
        
        function wasRunning = pauseMessageListener(obj)
            % Temporarily pause message listener
            % Returns: wasRunning - Boolean indicating if listener was active
            
            wasRunning = false;
            if ~isempty(obj.messageListener) && isvalid(obj.messageListener)
                wasRunning = strcmp(obj.messageListener.Running, 'on');
                if wasRunning
                    stop(obj.messageListener);
                end
            end
        end
        
        function resumeMessageListener(obj)
            % Resume message listener after pause
            
            if ~isempty(obj.messageListener) && isvalid(obj.messageListener)
                try
                    start(obj.messageListener);
                catch
                    % If restart fails, recreate the listener
                    obj.startMessageListener();
                end
            end
        end
        
        function startMessageListener(obj)
            % Initialize and start asynchronous message monitoring timer
            
            % Clean up existing listener
            if ~isempty(obj.messageListener) && isvalid(obj.messageListener)
                stop(obj.messageListener);
                delete(obj.messageListener);
            end
            
            % Create new timer for message monitoring
            obj.messageListener = timer(...
                'ExecutionMode', 'fixedRate', ...
                'Period', 0.05, ... % 50ms period (20Hz)
                'TimerFcn', @(~,~)obj.checkIncomingMessages(), ...
                'ErrorFcn', @(~,~)obj.handleListenerError(), ...
                'Name', 'PathPlannerMessageListener');
            
            start(obj.messageListener);
            notify(obj, 'StatusUpdate', ...
                PathPlannerEventData('Message listener started'));
        end
        
        function stopMessageListener(obj)
            % Stop and cleanup message monitoring timer
            
            try
                if ~isempty(obj.messageListener) && isvalid(obj.messageListener)
                    stop(obj.messageListener);
                    delete(obj.messageListener);
                    obj.messageListener = [];
                end
            catch ME
                warning(ME.identifier,'%s', ME.message);
            end

            obj.messageListener = [];
        end
        
        function ensureMessageListenerRunning(obj)
            % Ensure message listener is active when connection exists
            
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
        
        function checkIncomingMessages(obj)
            % Monitor and process incoming asynchronous messages
            
            try
                % Check if connected and if there is data in buffer 
                if obj.isConnected && obj.tcpClient.NumBytesAvailable > 0
                    responseBytes = read(obj.tcpClient, obj.tcpClient.NumBytesAvailable, 'uint8');
                    response = strtrim(char(responseBytes));
                    
                    % Log received message
                    notify(obj, 'StatusUpdate', ...
                        PathPlannerEventData(['← ' response]));
                    
                    % Handle multi-line responses and store as an array
                    responseLines = splitlines(string(response));
                    
                    for i = 1:length(responseLines)
                        % parse each line
                        line = char(strtrim(responseLines(i)));
                        if ~isempty(line)
                            % Emit message received event for other modules
                            notify(obj, 'MessageReceived', ...
                                PathPlannerEventData(line));
                        end
                    end
                end
            catch ME
                % Only report errors that aren't due to object deletion
                if ~contains(ME.message, 'Invalid or deleted object')
                    notify(obj, 'StatusUpdate', ...
                        PathPlannerEventData(['Message monitoring error: ' ME.message], false));
                end
            end
        end
        
        function handleListenerError(obj)
            % Handle errors in message listener timer
            
            notify(obj, 'StatusUpdate', ...
                PathPlannerEventData('Message listener encountered an error', false));
        end
        
        function attemptReconnection(obj)
            % Attempt automatic reconnection with configured retry limits
            % This method prevents infinite recursion by using the suppressReconnection flag
            
            % Prevent recursive reconnection attempts
            if obj.isReconnecting
                return;
            end

            obj.isReconnecting = true;

            notify(obj, 'StatusUpdate', PathPlannerEventData(...
                sprintf('Starting automatic reconnection (max %d attempts)', obj.config.maxRetryAttempts)));
            
            for attempt = 1:obj.config.maxRetryAttempts
                notify(obj, 'StatusUpdate', PathPlannerEventData(...
                    sprintf('Reconnection attempt %d/%d', attempt, obj.config.maxRetryAttempts)));
                
                pause(1); % Brief delay between attempts
                
                if obj.connect()
                    notify(obj, 'StatusUpdate', ...
                        PathPlannerEventData('Automatic reconnection successful'));
                    obj.isReconnecting = false;
                    return;
                end
            end
            
            % All attempts failed
            obj.isReconnecting = false;
            notify(obj, 'StatusUpdate', ...
                PathPlannerEventData('Automatic reconnection failed', false));
        end
    end
end