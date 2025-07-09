classdef PathPlannerClient_v1 < handle
    % PathPlannerClient - Main coordinator class for modular path planning system
    % 
    % This class serves as the primary interface for path planning operations,
    % coordinating between configuration, communication, trajectory, and status modules.
    % It maintains backward compatibility with the original API while providing
    % a cleaner, modular architecture.
    %
    % Usage:
    %   client = PathPlannerClient();
    %   client.connect();
    %   success = client.PlanPath('MC1', 'MC2', 'model_2R_RCM_IK');
    %   client.SendPath();
    %   client.ExecutePath();
    
    properties (Access = private)
        config              % Configuration management module
        comm               % Communication module
        trajectory         % Trajectory management module
        status             % Status management module
    end
    
    properties (SetAccess = private)
        isConnected = false % Connection status (read-only external access)
    end
    
    % Dependent properties - delegate to config module
    properties (Dependent)
        XMC1, YMC1, ZMC1, XMC2, YMC2, ZMC2  % Manipulator positions [mm]
        X0, Y0, Z0                          % Current position [mm]
        Phi0, Theta0, Psi0                  % Current orientation [degrees]
        XTarget, YTarget, ZTarget           % Target position [mm]
        PhiTarget, ThetaTarget, PsiTarget   % Target orientation [degrees]
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
        function obj = PathPlannerClient_v1()
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
            
            success = obj.comm.connect();
            obj.isConnected = success;
            
            if success
                notify(obj, 'ConnectionStateChanged');
            end
        end
        
        function disconnect(obj)
            % Disconnect from controller server and cleanup resources
            
            obj.comm.disconnect();
            obj.isConnected = false;
            notify(obj, 'ConnectionStateChanged');
        end
        
        %% Status Operations
        function [status1, status2] = GetStatus(obj, varargin)
            % Retrieve current status of manipulators
            % Inputs: id1, id2 (optional) - Manipulator identifiers
            % Returns: status1, status2 - Structures with position data [μm]
            
            [status1, status2] = obj.status.getStatus(varargin{:});
        end
        
        function [pose, elapsedTime] = GetPose(obj, varargin)
            % Send request for status to the controller side and call the FK
            % to solve for the current pose from the returned positional
            % information

            [pose, elapsedTime] = obj.status.getPose(varargin{:});
        end

        %% Trajectory Operations
        function success = PlanPath(obj, varargin)
            % Plan trajectory using inverse kinematics solver
            % Inputs: id1, id2, model (optional) - Manipulator IDs and IK model
            % Returns: success - Boolean indicating planning success
            
            success = obj.trajectory.planPath(varargin{:});
        end
        
        function success = SendPath(obj, varargin)
            % Transmit planned trajectory to controller
            % Inputs: id1, id2 (optional) - Manipulator identifiers
            % Returns: success - Boolean indicating transmission success
            
            success = obj.trajectory.sendPath(varargin{:});
        end
        
        function success = ExecutePath(obj, varargin)
            % Initiate trajectory execution on controller
            % Inputs: id1, id2 (optional) - Manipulator identifiers
            % Returns: success - Boolean indicating command success
            
            success = obj.trajectory.executePath(varargin{:});
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
            
            ready = obj.trajectory.isReady();
        end
        
        function data = getLastTrajectoryData(obj)
            % Get last calculated trajectory data
            % Returns: data - Structure containing trajectory information
            
            data = obj.trajectory.getLastData();
        end
        
        function executing = isPathExecuting(obj)
            % Check if path execution is currently in progress
            % Returns: executing - Boolean indicating execution status
            
            executing = obj.trajectory.isExecuting();
        end
        
        function params = getConfigurationSummary(obj)
            % Get summary of current configuration parameters
            % Returns: params - Structure with configuration summary
            
            params = obj.config.getSummary();
        end

        %% Configuration Access Methods
        function setTargetPose(obj, XTarget, YTarget, ZTarget, PhiTarget)
            % Set target pose values in configuration
            % Inputs: XTarget, YTarget, ZTarget, PhiTarget - Target pose values
            
            if nargin >= 2, obj.config.XTarget = XTarget * 1e-3; end
            if nargin >= 3, obj.config.YTarget = YTarget * 1e-3; end
            if nargin >= 4, obj.config.ZTarget = ZTarget * 1e-3; end
            if nargin >= 5, obj.config.PhiTarget = PhiTarget; end
        end
        
        function config = getConfig(obj)
            % Get direct access to configuration object (use with caution)
            % Returns: config - Configuration module reference
            
            config = obj.config;
        end
        
        function delete(obj)
            % Destructor - Clean up resources and close connections
            try
                if obj.isConnected
                    obj.disconnect();
                end
                % 清理所有模块
                if ~isempty(obj.trajectory)
                    delete(obj.trajectory);
                    obj.trajectory = [];
                end
                
                if ~isempty(obj.status)
                    delete(obj.status);
                    obj.status = [];
                end
                
                if ~isempty(obj.comm)
                    delete(obj.comm);
                    obj.comm = [];
                end
                
                if ~isempty(obj.config)
                    delete(obj.config);
                    obj.config = [];
                end
            catch ME
                warning(ME.identifier, '%s', ME.message);
            end
        end
    end

    % Property get methods
    methods        
        function value = get.XMC1(obj), value = obj.config.XMC1; end

        function value = get.YMC1(obj), value = obj.config.XMC1; end

        function value = get.ZMC1(obj), value = obj.config.XMC1; end

        function value = get.XMC2(obj), value = obj.config.XMC1; end

        function value = get.YMC2(obj), value = obj.config.XMC1; end

        function value = get.ZMC2(obj), value = obj.config.XMC1; end

        function value = get.X0(obj), value = obj.config.X0; end
        
        function value = get.Y0(obj), value = obj.config.Y0; end
        
        function value = get.Z0(obj), value = obj.config.Z0; end
        
        function value = get.Phi0(obj), value = obj.config.Phi0; end
        
        function value = get.Theta0(obj), value = obj.config.Theta0; end
        
        function value = get.Psi0(obj), value = obj.config.Psi0; end
        
        function value = get.XTarget(obj), value = obj.config.XTarget; end
        
        function value = get.YTarget(obj), value = obj.config.YTarget; end
        
        function value = get.ZTarget(obj), value = obj.config.ZTarget; end
        
        function value = get.PhiTarget(obj), value = obj.config.PhiTarget; end

        function value = get.ThetaTarget(obj), value = obj.config.ThetaTarget; end

        function value = get.PsiTarget(obj), value = obj.config.PsiTarget; end
    end

    methods (Access = private)
        function initializeModules(obj)
            % Initialize all functional modules in correct dependency order
            
            % Configuration module has no dependencies
            obj.config = PathPlannerConfig();
            
            % Communication module depends on configuration
            obj.comm = PathPlannerComm(obj.config);
            
            % Status modules depend on communication and config
            obj.status = PathPlannerStatus(obj.comm, obj.config);
            
            % Trajectory module depends on comm, config, and status
            obj.trajectory = PathPlannerTrajectory(obj.comm, obj.config, obj.status);
        end
        
        function setupEventForwarding(obj)
            % Setup event forwarding from modules to main class
            % This maintains the original event interface for backward compatibility
            
            % Communication module events
            addlistener(obj.comm, 'StatusUpdate', ...
                @(src,evt) notify(obj, 'StatusUpdate', evt));
            addlistener(obj.comm, 'ConnectionStateChanged', ...
                @(src,evt) notify(obj, 'ConnectionStateChanged', evt));
            
            % Trajectory module events
            addlistener(obj.trajectory, 'StatusUpdate', ...
                @(src,evt) notify(obj, 'StatusUpdate', evt));
            addlistener(obj.trajectory, 'TrajectoryReady', ...
                @(src,evt) notify(obj, 'TrajectoryReady', evt));
            addlistener(obj.trajectory, 'TrajectoryExecuted', ...
                @(src,evt) notify(obj, 'TrajectoryExecuted', evt));
            addlistener(obj.trajectory, 'PathDataReceived', ...
                @(src,evt) notify(obj, 'PathDataReceived', evt));
            addlistener(obj.trajectory, 'PathExecutionStarted', ...
                @(src,evt) notify(obj, 'PathExecutionStarted', evt));
            addlistener(obj.trajectory, 'PathExecutionFailed', ...
                @(src,evt) notify(obj, 'PathExecutionFailed', evt));
            
            % Status module events
            addlistener(obj.status, 'StatusUpdate', ...
                @(src,evt) notify(obj, 'StatusUpdate', evt));
            
            % Configuration module events
            addlistener(obj.config, 'ConfigurationLoaded', ...
                @(src,evt) notify(obj, 'ConfigurationLoaded', evt));
        end
    end
end