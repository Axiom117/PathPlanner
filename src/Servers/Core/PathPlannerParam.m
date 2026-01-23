classdef PathPlannerParam < handle
% PathPlannerConfig - Configuration Management Module ("Single Source of Truth")
    %
    % This module acts as the central "data warehouse" for the entire system,
    % loading and managing all shared parameters.
    %
    % As a 'handle' class, it functions as a Single Source of Truth. It is
    % injected into and shared by all other modules (Comm, Trajectory, Status),
    % ensuring that any configuration change is instantly visible system-wide.
    %
    % It provides robust startup by attempting to load parameters from an external
    % ConfigManager. If this fails, it safely falls back to hard-coded default
    % values, guaranteeing the system always launches with a valid configuration.
    
    properties (Access = public)
        %% 1. Runtime State (Dynamic Variables)

        % Current end-effector state (mm, degrees)
        X0 = 0; Y0 = 0; Z0 = 0;
        Phi0 = 0; Theta0 = 0; Psi0 = 0;
        
        % Target pose references (mm, degrees)
        XTarget = 0; YTarget = 0; ZTarget = 0;
        PhiTarget = 0; ThetaTarget = 0; PsiTarget = 0;
        
        % Current Manipulator hardware state (mm)
        XMC1 = 0; YMC1 = 0; ZMC1 = 0;
        XMC2 = 0; YMC2 = 0; ZMC2 = 0;

        % Path Planning and Execution Parameters
        interval = 0.1                % Path tracking interval [s]

        %% 2. Core Configuration (Loaded from ConfigFile, rarely changed)

        % Communication parameters
        controllerHost = '127.0.0.1'    % Controller server IP address
        controllerPort = 5000           % Controller server port number
        connectionTimeout = 10          % TCP connection timeout [s]
        responseTimeout = 1             % Command response timeout [s]
        maxRetryAttempts = 3            % Maximum connection retry attempts
        
        % Manipulator identification
        manipulatorID1 = 'MC1'          % Primary manipulator identifier
        manipulatorID2 = 'MC2'          % Secondary manipulator identifier

        % Solver
        modelIK
        modelFK
    end
    
    properties (Access = private)
        configManager                   % ConfigManager instance
    end
    
    events
        ConfigurationLoaded             % Fired when configuration is loaded
        ParameterUpdated                % Fired when a critical parameter is changed
    end
    
    methods (Access = public)
        function obj = PathPlannerParam()

            % Constructor - Load configuration from ConfigManager or use defaults
            obj.loadConfiguration();
            
        end

        %% Unified Access Interface (The "Central Hub" Logic)
        function val = getParam(obj, paramName, configCategory)
            % getParam - The universal getter
            % 
            % Logic Priority:
            % 1. Check if 'paramName' is a public property of this class (Runtime/Core Config).
            % 2. If it is NOT a property, check 'ConfigManager' logic (Extended Config).
            %
            % Usage:
            %   val = param.getParam('XTarget');              % Get Runtime Property
            %   val = param.getParam('controllerHost');       % Get Core Config
            %   val = param.getParam('logLevel', 'system');   % Get extra config from file
            
            if isprop(obj, paramName)
                % Priority 1: Return the class property (Accessing Runtime variables)
                val = obj.(paramName);
            else
                % Priority 2: Fallback to underlying ConfigManager (Accessing hidden/static configs)
                if ~isempty(obj.configManager)
                    if nargin < 3
                         % If category not provided, search might rely on unique names or fail
                         % Ideally, ConfigMap requires category.
                         error('PathPlannerParam:MissingCategory', ...
                             'Parameter "%s" is not a runtime property. To fetch from ConfigManager, provide a category name.', paramName);
                    end
                    val = obj.configManager.getParam(configCategory, paramName);
                else
                    error('PathPlannerParam:NotFound', 'Parameter "%s" not found.', paramName);
                end
            end
        end

        function setParam(obj, paramName, value, configCategory)
            % setParam - The universal setter
            %
            % Usage:
            %   param.setParam('XTarget', 100);               % Update Runtime Property
            %   param.setParam('debugMode', true, 'system');  % Update background config
            
            if isprop(obj, paramName)
                % Case 1: Updating a Runtime Property
                obj.(paramName) = value;
                
                % Optional: Notify if needed, or simply update state
                % notify(obj, 'ParameterUpdated', PathPlannerEventData(paramName));
            else
                % Case 2: Updating a ConfigManager value (Runtime override of config file)
                if ~isempty(obj.configManager) && nargin >= 4
                    obj.configManager.setParam(configCategory, paramName, value);
                else
                     warning('PathPlannerParam:SetFailed', ...
                         'Cannot set "%s". It is not a property and ConfigManager usage is invalid.', paramName);
                end
            end
        end

        %% Configuration Loading Logic

        function loadConfiguration(obj)
            % Initialize ConfigManager and hydrate Core Properties
            
            try
                % Attempt to use ConfigManager in singleton mode
                obj.configManager = ConfigManager.getInstance();
                
                % Load all configurations from config files to manager cache
                obj.configManager.loadAllConfigs();

                % Copy data from manager cache to public properties
                obj.loadParametersFromConfig();
                
                notify(obj, 'ConfigurationLoaded', ...
                    PathPlannerEventData('Configuration loaded from ConfigManager'));
                    
            catch ME
                % Fall back to default parameters once any step fails
                warning('PathPlannerParam:LoadFailed', ...
                    'Configuration loading failed: %s. Using default parameters.', ME.message);
                
                notify(obj, 'ConfigurationLoaded', ...
                    PathPlannerEventData('Default configuration applied', true));
            end
        end
        
        function params = getSummary(obj)
            % Get summary of current configuration parameters
            % Returns: params - Structure with organized configuration data
            
            params = struct();
            
            % Communication settings
            params.communication = struct( ...
                'host', obj.controllerHost, ...
                'port', obj.controllerPort, ...
                'connectionTimeout', obj.connectionTimeout, ...
                'responseTimeout', obj.responseTimeout, ...
                'maxRetryAttempts', obj.maxRetryAttempts);
            
            % Manipulator settings
            params.manipulators = struct( ...
                'id1', obj.manipulatorID1, ...
                'id2', obj.manipulatorID2);
            
            % Initial pose
            params.initialPose = struct( ...
                'X0', obj.X0, 'Y0', obj.Y0, 'Z0', obj.Z0, ...
                'Phi0', obj.Phi0, 'Theta0', obj.Theta0, 'Psi0', obj.Psi0);
            
            % Target references
            params.targets = struct( ...
                'XTarget', obj.XTarget, 'YTarget', obj.YTarget, ...
                'ZTarget', obj.ZTarget, 'PhiTarget', obj.PhiTarget, ...
                'ThetaTarget', obj.ThetaTarget, 'PsiTarget', obj.PsiTarget);
            
            % Path tracking parameters
            params.pathTracking = struct( ...
                'interval', obj.interval);
        end
        
        function updateParameter(obj, category, paramName, value)
            % Update a specific configuration parameter in both public
            % properties and config manager cache
            
            try
                if isprop(obj, paramName)
                    obj.(paramName) = value;
                    
                    % Also update in ConfigManager if available
                    if ~isempty(obj.configManager)
                        obj.configManager.setParam(category, paramName, value);
                    end
                else
                    warning('PathPlannerConfig:InvalidParameter', ...
                        'Parameter %s does not exist', paramName);
                end
            catch ME
                warning('PathPlannerConfig:UpdateFailed', ...
                    'Failed to update parameter %s: %s', paramName, ME.message);
            end
        end
    end
    
    methods (Access = private)
        function loadParametersFromConfig(obj)
            % Load parameters from ConfigManager into object properties
            
            try
                % Communication parameters
                obj.controllerHost = obj.safeGetConfig('communication', 'controllerHost', 'defaultHost');
                obj.controllerPort = obj.safeGetConfig('communication', 'controllerPort', 1234);
                obj.connectionTimeout = obj.safeGetConfig('communication', 'connectionTimeout', 30);
                obj.responseTimeout = obj.safeGetConfig('communication', 'responseTimeout', 30);
                obj.maxRetryAttempts = obj.safeGetConfig('communication', 'maxRetryAttempts', 3);
                
                % Controller parameters
                obj.manipulatorID1 = obj.safeGetConfig('controller', 'manipulatorID1', 'defaultID1');
                obj.manipulatorID2 = obj.safeGetConfig('controller', 'manipulatorID2', 'defaultID2');
                % Solver parameters
                obj.modelIK = obj.safeGetConfig('simulation', 'modelIK', 'defaultModelIK');
                obj.modelFK = obj.safeGetConfig('simulation', 'modelFK', 'defaultModelFK');
                
                % Initial manipulator positions
                obj.XMC1 = obj.safeGetConfig('controller', 'XMC1', 0);
                obj.YMC1 = obj.safeGetConfig('controller', 'YMC1', 0);
                obj.ZMC1 = obj.safeGetConfig('controller', 'ZMC1', 0);
                obj.XMC2 = obj.safeGetConfig('controller', 'XMC2', 0);
                obj.YMC2 = obj.safeGetConfig('controller', 'YMC2', 0);
                obj.ZMC2 = obj.safeGetConfig('controller', 'ZMC2', 0);

                % Initial pose parameters
                obj.X0 = obj.safeGetConfig('simulation', 'X0', 0);
                obj.Y0 = obj.safeGetConfig('simulation', 'Y0', 0);
                obj.Z0 = obj.safeGetConfig('simulation', 'Z0', 0);
                obj.Phi0 = obj.safeGetConfig('simulation', 'Phi0', 0);
                obj.Theta0 = obj.safeGetConfig('simulation', 'Theta0', 0);
                obj.Psi0 = obj.safeGetConfig('simulation', 'Psi0', 0);
                
            catch ME
                warning('PathPlannerConfig:ParameterLoadFailed', ...
                    'Failed to load some parameters: %s', ME.message);
            end
        end

        function val = safeGetConfig(obj, category, name, defaultVal)
            % Helper to safely get from ConfigManager without crashing initialization
            try
                val = obj.configManager.getParam(category, name);
            catch
                val = defaultVal;
            end
        end
    end
end