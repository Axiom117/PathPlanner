classdef PathPlannerClient < handle
% PathPlannerClient - Facade (Coordinator) for the modular path planning system.
    % 
    % This class implements the Facade design pattern, acting as the central 
    % coordinator ("Commander") for the entire system.
    %
    % It provides a single, simplified interface for the MATLAB App (UI),
    % decoupling the UI from the complex, underlying business logic. It achieves
    % this by coordinating all operations between the internal configuration, 
    % communication, trajectory, and status modules.
    %
    % This design simplifies the UI code (which only needs to talk to this 
    % class) and greatly improves maintainability, as internal modules can be 
    % modified or replaced without impacting the UI.

    properties (Access = private)
        param             % Parameter management module
        comm              % Communication module
        syncOps           % Synchronous operations module
        asyncTask         % Asynchronous task management module
    end
    
    properties (SetAccess = private)
        isConnected = false % Connection status (read-only external access)
    end
    
    % Events - maintain original event interface
    events
        StatusUpdate            % General status and communication updates
        TrajectoryReady         % Fired when trajectory planning completes
        TrajectoryExecuted      % Fired when trajectory execution completes
        ConnectionStateChanged  % Fired on connection/disconnection events
        PathDataReceived        % Fired when controller confirms data receipt
        PathExecutionStarted    % Fired when path execution begins
        PathExecutionFailed     % Fired when path execution encounters errors
        ConfigurationLoaded     % Fired when configuration parameters are loaded
    end
    
    methods (Access = public)
        function obj = PathPlannerClient()
            % Constructor - Initialize all modules and setup event forwarding
            
            obj.initializeModules();
            obj.setupEventForwarding();
            
            % Notify initialization complete
            notify(obj, 'ConfigurationLoaded', ...
                PathPlannerEventData('PathPlannerClient initialized with modular architecture'));
        end
        
        %% Connection Management
        function success = connect(obj)
            % Establish connection to controller server
            % Returns: success - Boolean indicating connection status
            
            % Delegate connect method to comm module
            success = obj.comm.connect();
            obj.isConnected = success;
            
            if success
                notify(obj, 'ConnectionStateChanged');
            end
        end
        
        function disconnect(obj)
            % Disconnect from controller server and cleanup resources
            
            % Delegate disconnet method to comm module
            obj.comm.disconnect();
            obj.isConnected = false;
            notify(obj, 'ConnectionStateChanged');
        end
        
        %% Status Operations
        function [status1, status2] = GetStatus(obj, varargin)
            % Retrieve current status of manipulators
            % Inputs: id1, id2 (optional) - Manipulator identifiers
            % Returns: status1, status2 - Structures with position data [μm]
            
            % Delegate getStatus method to synchronous operations module
            [status1, status2] = obj.syncOps.getStatus(varargin{:});
        end
        
        function [pose, elapsedTime] = GetPose(obj, varargin)
            % Send request for status to the controller side and call the FK
            % to solve for the current pose from the returned positional
            % information
            
            % Delegate getPose method to synchronous operations module
            [pose, elapsedTime] = obj.syncOps.getPose(varargin{:});
        end

        %% Trajectory Operations
        function success = PlanPath(obj, varargin)
            % Plan trajectory using inverse kinematics solver
            % Inputs: id1, id2, model (optional) - Manipulator IDs and IK model
            % Returns: success - Boolean indicating planning success
            
            % Delegate planPath to asynchronous task module, varargin allows this
            % method to take any number of the input parameters.
            success = obj.asyncTask.planPath(varargin{:});
        end
        
        function success = SendPath(obj, varargin)
            % Transmit planned trajectory to controller
            % Inputs: id1, id2 (optional) - Manipulator identifiers
            % Returns: success - Boolean indicating transmission success
            
            % Delegate sendPath to asynchronous task module
            success = obj.asyncTask.sendPath(varargin{:});
        end
        
        function success = StartPath(obj, varargin)
            % Initiate trajectory execution on controller
            % Inputs: id1, id2 (optional) - Manipulator identifiers
            % Returns: success - Boolean indicating command success
            
            success = obj.asyncTask.startPath(varargin{:});
        end
        
        %% Utility Methods
        function success = sendHeartbeat(obj)
            % Send heartbeat to verify connection integrity
            % Returns: success - Boolean indicating heartbeat response
            
            success = obj.comm.sendHeartbeat();
        end
        
        function ready = isTrajectoryReady(obj)
            % Check if trajectory is ready for execution
            % Returns: ready - Boolean indicating trajectory readiness
            
            ready = obj.asyncTask.isReady();
        end
        
        function data = getLastTrajectoryData(obj)
            % Get last calculated trajectory data
            % Returns: data - Structure containing trajectory information
            
            data = obj.asyncTask.getLastData();
        end
        
        function executing = isPathExecuting(obj)
            % Check if path execution is currently in progress
            % Returns: executing - Boolean indicating execution status
            
            executing = obj.asyncTask.isExecuting();
        end
        
        function params = getParameterSummary(obj)
            % Get summary of current configuration parameters
            % Returns: params - Structure with configuration summary
            
            params = obj.param.getSummary();
        end

        %% Configuration Access Methods
        function paramObj = getParamObj(obj)
            % Get direct access to parameter object (use with caution)
            % Returns: param - Parameter module reference
            
            paramObj = obj.param;
        end
        
        function delete(obj)
            % Destructor - Clean up resources and close connections
            try
                if obj.isConnected
                    obj.disconnect();
                end
                % Clear all modules
                if ~isempty(obj.asyncTask)
                    delete(obj.asyncTask);
                    obj.asyncTask = [];
                end
                
                if ~isempty(obj.syncOps)
                    delete(obj.syncOps);
                    obj.syncOps = [];
                end
                
                if ~isempty(obj.comm)
                    delete(obj.comm);
                    obj.comm = [];
                end
                
                if ~isempty(obj.param)
                    delete(obj.param);
                    obj.param = [];
                end
            catch ME
                warning(ME.identifier, '%s', ME.message);
            end
        end
    end

    methods (Access = private)
        function initializeModules(obj)
            % Dependency assembling and injection
            % Initialize all functional modules in correct dependency order
            
            % Configuration module has no dependencies
            obj.param = PathPlannerParam();
            
            % Communication module depends on configuration
            obj.comm = PathPlannerComm(obj.param);
            
            % Status modules depend on communication and config
            obj.syncOps = PathPlannerSyncOps(obj.comm, obj.param);
            
            % Trajectory module depends on comm, config, and status
            obj.asyncTask = PathPlannerAsyncTask(obj.comm, obj.param, obj.syncOps);
        end
        
        function setupEventForwarding(obj)
            % Event aggregation
            % Setup event forwarding from modules to main class
            % This maintains the original event interface for backward compatibility
            
            % Communication module events
            addlistener(obj.comm, 'StatusUpdate', ...
                @(src,evt) notify(obj, 'StatusUpdate', evt));
            addlistener(obj.comm, 'ConnectionStateChanged', ...
                @(src,evt) notify(obj, 'ConnectionStateChanged', evt));
            
            % Trajectory module events
            addlistener(obj.asyncTask, 'StatusUpdate', ...
                @(src,evt) notify(obj, 'StatusUpdate', evt));
            addlistener(obj.asyncTask, 'TrajectoryReady', ...
                @(src,evt) notify(obj, 'TrajectoryReady', evt));
            addlistener(obj.asyncTask, 'TrajectoryExecuted', ...
                @(src,evt) notify(obj, 'TrajectoryExecuted', evt));
            addlistener(obj.asyncTask, 'PathDataReceived', ...
                @(src,evt) notify(obj, 'PathDataReceived', evt));
            addlistener(obj.asyncTask, 'PathExecutionStarted', ...
                @(src,evt) notify(obj, 'PathExecutionStarted', evt));
            addlistener(obj.asyncTask, 'PathExecutionFailed', ...
                @(src,evt) notify(obj, 'PathExecutionFailed', evt));
            
            % Status module events
            addlistener(obj.syncOps, 'StatusUpdate', ...
                @(src,evt) notify(obj, 'StatusUpdate', evt));
            
            % Configuration module events
            addlistener(obj.param, 'ConfigurationLoaded', ...
                @(src,evt) notify(obj, 'ConfigurationLoaded', evt));
        end
    end
end