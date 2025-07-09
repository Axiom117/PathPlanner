classdef PathPlannerConfig < handle
    % PathPlannerConfig - Configuration management module
    % 
    % This module handles loading and managing all configuration parameters
    % for the PathPlanner system. It attempts to load from ConfigManager
    % and falls back to default values if loading fails.
    %
    % Properties include communication settings, manipulator IDs, and
    % initial pose parameters.
    
    properties (Access = public)
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
        modelIK, modelFK
        
        % Current state of two connected manipulators (in mm)
        XMC1 = 0;
        YMC1 = 0;
        ZMC1 = 0;
        XMC2 = 0;
        YMC2 = 0;
        ZMC2 = 0;

        % Current end-effector state (in mm and degrees)
        X0 = 0                          % Current X position [mm]
        Y0 = 0                          % Current Y position [mm]
        Z0 = 0                          % Current Z position [mm]
        Phi0 = 0                        % Current Phi angle [degrees]
        Theta0 = 0                      % Current Theta angle [degrees]
        Psi0 = 0                        % Current Psi angle [degrees]
        
        % Target pose references (for onCalcIK compatibility)
        XTarget = 0                     % Reference to target X field
        YTarget = 0                     % Reference to target Y field
        ZTarget = 0                     % Reference to target Z field
        PhiTarget = 0                   % Reference to target Phi field
        ThetaTarget = 0                 % Reference to target Theta field
        PsiTarget = 0                   % Reference to target Psi field
    end
    
    properties (Access = private)
        configManager                   % ConfigManager instance
    end
    
    events
        ConfigurationLoaded             % Fired when configuration is loaded
    end
    
    methods (Access = public)
        function obj = PathPlannerConfig()
            % Constructor - Load configuration from ConfigManager or use defaults
            
            obj.loadConfiguration();
        end
        
        function loadConfiguration(obj)
            % Load configuration parameters from ConfigManager
            % Falls back to default values if loading fails
            
            try
                % Attempt to use ConfigManager
                obj.configManager = ConfigManager.getInstance();
                obj.configManager.loadAllConfigs();
                obj.loadParametersFromConfig();
                
                notify(obj, 'ConfigurationLoaded', ...
                    PathPlannerEventData('Configuration loaded from ConfigManager'));
                    
            catch ME
                % Fall back to default parameters
                warning('PathPlannerConfig:LoadFailed', ...
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
        end
        
        function updateParameter(obj, category, paramName, value)
            % Update a specific configuration parameter
            %
            % Inputs:
            %   category  - Parameter category ('communication', 'manipulator', etc.)
            %   paramName - Name of parameter to update
            %   value     - New parameter value
            
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
                obj.controllerHost = obj.configManager.getParam('communication', 'controllerHost');
                obj.controllerPort = obj.configManager.getParam('communication', 'controllerPort');
                obj.connectionTimeout = obj.configManager.getParam('communication', 'connectionTimeout');
                obj.responseTimeout = obj.configManager.getParam('communication', 'responseTimeout');
                obj.maxRetryAttempts = obj.configManager.getParam('communication', 'maxRetryAttempts');
                
                % Controller parameters
                obj.manipulatorID1 = obj.configManager.getParam('controller', 'manipulatorID1');
                obj.manipulatorID2 = obj.configManager.getParam('controller', 'manipulatorID2');

                % Solver parameters
                obj.modelIK = obj.configManager.getParam('simulation', 'modelIK');
                obj.modelFK = obj.configManager.getParam('simulation', 'modelFK');
                
                % Initial manipulator positions
                obj.XMC1 = obj.configManager.getParam('controller', 'XMC1');
                obj.YMC1 = obj.configManager.getParam('controller', 'YMC1');
                obj.ZMC1 = obj.configManager.getParam('controller', 'ZMC1');
                obj.XMC2 = obj.configManager.getParam('controller', 'XMC2');
                obj.YMC2 = obj.configManager.getParam('controller', 'YMC2');
                obj.ZMC2 = obj.configManager.getParam('controller', 'ZMC2');

                % Initial pose parameters
                obj.X0 = obj.configManager.getParam('simulation', 'X0');
                obj.Y0 = obj.configManager.getParam('simulation', 'Y0');
                obj.Z0 = obj.configManager.getParam('simulation', 'Z0');
                obj.Phi0 = obj.configManager.getParam('simulation', 'Phi0');
                obj.Theta0 = obj.configManager.getParam('simulation', 'Theta0');
                obj.Psi0 = obj.configManager.getParam('simulation', 'Psi0');
                
            catch ME
                warning('PathPlannerConfig:ParameterLoadFailed', ...
                    'Failed to load some parameters: %s', ME.message);
            end
        end
    end
end