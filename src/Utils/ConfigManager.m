classdef ConfigManager < handle
    % ConfigManager - Unified configuration management system (Refactored)
    % Provides centralized loading, caching, and access to system parameters
    % from configuration functions stored in the configs folder.
    %
    % Features:
    %   - Function-based configuration files returning struct
    %   - Parameter caching for improved performance
    %   - Workspace integration and management
    %   - Singleton pattern for global access
    %   - Dynamic parameter updates and validation
    %   - No eval() usage - cleaner and safer implementation
    %
    % Usage:
    %   configMgr = ConfigManager();
    %   param = configMgr.getParam('robot', 'cubeLength');
    %   config = configMgr.getConfig('robot');
    %
    % Static usage:
    %   ConfigManager.loadGlobalConfigs();
    %   value = ConfigManager.getGlobalParameter('robot', 'cubeLength');
    
    properties (Constant, Access = private)
        % Default configuration folder name
        CONFIG_FOLDER = 'configs'
        
        % A mapping table that matches standard configuration categories 
        % and their default function names
        DEFAULT_CONFIG_FUNCTIONS = struct(...
            'robot', 'robot_params', ...
            'simulation', 'sim_params', ...
            'controller', 'controller_params', ...
            'communication', 'comm_params')
    end
    
    properties (Access = private)
        configPath          % Absolute path to configuration folder
        loadedConfigs       % Cache of loaded configuration structures
        isInitialized       % Initialization status flag
    end
    
    methods (Access = public)
        function obj = ConfigManager(configPath)
            % Constructor - Initialize configuration manager
            %
            % Input:
            %   configPath - (optional) Custom path to configuration folder
            %                If not provided, uses './configs'
            %
            % The constructor automatically initializes the manager and sets up
            % the configuration path. If the configuration folder doesn't exist,
            % a warning is issued but the manager remains functional.
            
            if nargin < 1
                obj.configPath = fullfile(pwd, obj.CONFIG_FOLDER);
            else
                obj.configPath = configPath;
            end
            
            % Create an empty struct as cache
            obj.loadedConfigs = struct();
            obj.isInitialized = false;
            
            % Add config path to MATLAB path for feval() to find the function
            if exist(obj.configPath, 'dir')
                addpath(obj.configPath);
                obj.isInitialized = true;
            else
                warning('ConfigManager:PathNotFound', ...
                    'Configuration path not found: %s', obj.configPath);
            end
        end
        
        function config = loadConfig(obj, configName, functionName)
            % Load a specific configuration by calling its function
            %
            % Inputs:
            %   configName   - Configuration category name (e.g., 'robot', 'simulation')
            %   functionName - (optional) Custom function name. If not provided, uses default
            %
            % Returns:
            %   config - Structure containing loaded parameters
            %
            % The method calls the configuration function directly and caches
            % the returned structure for subsequent access.
            
            try
                % Determine configuration function name from mapping
                % table
                if nargin < 3
                    if isfield(obj.DEFAULT_CONFIG_FUNCTIONS, configName)
                        % (configName) represents the dynamic field name,
                        % will be substituted into the actual field name
                        functionName = obj.DEFAULT_CONFIG_FUNCTIONS.(configName);
                    else
                        functionName = [configName '_params'];
                    end
                end
                
                % Call configuration function and return the config struct
                config = feval(functionName);
                
                % Cache the loaded configuration using dynamic field name,
                % next time when requesting the config, it will be returned
                % from the loaded struct
                obj.loadedConfigs.(configName) = config;
                
                fprintf('Loaded configuration: %s (%d parameters)\n', ...
                    configName, length(fieldnames(config)));
                
            catch ME
                warning('ConfigManager:LoadFailed', ...
                    'Failed to load configuration %s: %s', configName, ME.message);
                config = struct();
            end
        end
        
        function loadAllConfigs(obj)
            % Load all default configuration files
            % Iterates through all predefined configuration categories and loads them
            
            configNames = fieldnames(obj.DEFAULT_CONFIG_FUNCTIONS);
            successCount = 0;
            
            for i = 1:length(configNames)
                configName = configNames{i};
                config = obj.loadConfig(configName);
                if ~isempty(fieldnames(config))
                    successCount = successCount + 1;
                end
            end
            
            fprintf('Successfully loaded %d/%d configurations\n', ...
                successCount, length(configNames));
        end
        
        function config = getConfig(obj, configName)
            % Retrieve a loaded configuration structure
            
            % Firstly check the cached config
            if isfield(obj.loadedConfigs, configName)
                config = obj.loadedConfigs.(configName);
            else
                % Auto-load if not already cached
                config = obj.loadConfig(configName);
            end
        end
        
        function param = getParam(obj, configName, paramName)
            % Retrieve a specific parameter value with optional default
            
            config = obj.getConfig(configName);
            
            if isfield(config, paramName)
                param = config.(paramName);
            else
                error('ConfigManager:ParameterNotFound', ...
                    'Parameter %s.%s not found and no default provided', configName, paramName);
            end
        end
        
        function setParam(obj, configName, paramName, value)
            % Set parameter value in memory (does not persist to file)
            
            % Create the config if not exist
            if ~isfield(obj.loadedConfigs, configName)
                obj.loadedConfigs.(configName) = struct();
            end
            
            % Update the param in cached config
            obj.loadedConfigs.(configName).(paramName) = value;
            
            fprintf('Updated parameter %s.%s\n', configName, paramName);
        end
        
        function success = reloadConfig(obj, configName)
            % Reload a specific configuration from its function
            
            fprintf('Reloading configuration: %s\n', configName);
            
            % Clear from cache
            if isfield(obj.loadedConfigs, configName)
                obj.loadedConfigs = rmfield(obj.loadedConfigs, configName);
            end
            
            % Reload
            config = obj.loadConfig(configName);
            success = ~isempty(fieldnames(config));
        end
        
        function listConfigs(obj)
            % Display summary of all loaded configurations
            % Shows configuration names and parameter counts for debugging
            
            if isempty(fieldnames(obj.loadedConfigs))
                fprintf('No configurations loaded\n');
                return;
            end
            
            fprintf('Loaded configurations:\n');
            fprintf('%-15s %-10s %s\n', 'Config', 'Params', 'Sample Parameters');
            fprintf('%s\n', repmat('-', 1, 60));
            
            configNames = fieldnames(obj.loadedConfigs);
            for i = 1:length(configNames)
                configName = configNames{i};
                config = obj.loadedConfigs.(configName);
                paramNames = fieldnames(config);
                paramCount = length(paramNames);
                
                % Show first few parameters as examples
                if paramCount > 0
                    numSamples = min(3, paramCount);
                    if paramCount > 3
                        % Pre-allocate with estimated size
                        sampleParams = [strjoin(paramNames(1:numSamples), ', '), '...'];
                    else
                        sampleParams = strjoin(paramNames(1:numSamples), ', ');
                    end
                else
                    sampleParams = '';
                end
                
                fprintf('%-15s %-10d %s\n', configName, paramCount, sampleParams);
            end
        end
        
        function paramCount = exportToWorkspace(obj, configName)
            % Export configuration parameters to MATLAB base workspace
            %
            % This method is useful for making parameters available to Simulink
            % models and legacy scripts that expect workspace variables
            
            if nargin < 2
                % Export all configurations recursively
                configNames = fieldnames(obj.loadedConfigs);
                totalParams = 0;
                for i = 1:length(configNames)
                    paramCount = obj.exportToWorkspace(configNames{i});
                    totalParams = totalParams + paramCount;
                end
                fprintf('Total exported: %d parameters from %d configurations\n', ...
                    totalParams, length(configNames));

                paramCount = totalParams;
                return;
            end
            
            config = obj.getConfig(configName);
            paramNames = fieldnames(config);
            
            for i = 1:length(paramNames)
                paramName = paramNames{i};
                % Load param as variable into base workspace
                assignin('base', paramName, config.(paramName));
            end
            
            fprintf('Exported %d parameters from %s configuration to workspace\n', ...
                length(paramNames), configName);

            paramCount = length(paramNames);
        end
        
        function clearCache(obj)
            % Clear all cached configurations
            % Forces reload on next access
            
            obj.loadedConfigs = struct();
            fprintf('Configuration cache cleared\n');
        end
    end
    
    methods (Static)
        function instance = getInstance()
            % Get singleton instance of ConfigManager
            % Implements singleton pattern for global configuration access
            %
            % Returns:
            %   instance - Singleton ConfigManager instance
            %
            % This method ensures only one ConfigManager instance exists per
            % MATLAB session, providing consistent global configuration state
            
            persistent configManagerInstance;
            
            if isempty(configManagerInstance) || ~isvalid(configManagerInstance)
                configManagerInstance = ConfigManager();
            end
            
            instance = configManagerInstance;
        end
        
        function param = getGlobalParameter(configName, paramName)
            % Static method for convenient global parameter access
            %
            % This method provides a convenient one-line access to any configuration
            % parameter from anywhere in the codebase without creating manager instances
            
            manager = ConfigManager.getInstance();
            param = manager.getParam(configName, paramName);
        end
        
        function loadGlobalConfigs()
            % Static method to load all global configurations
            % Convenient method for initializing all configurations in startup scripts
            %
            % This method uses the singleton instance to load all default
            % configuration files, making them available globally throughout
            % the MATLAB session
            
            manager = ConfigManager.getInstance();
            manager.loadAllConfigs();
        end
        
        function config = getGlobalConfig(configName)
            % Static method to get entire configuration structure
            %
            
            manager = ConfigManager.getInstance();
            config = manager.getConfig(configName);
        end
        
        function reloadGlobalConfig(configName)
            % Static method to reload a global configuration
            %
            
            manager = ConfigManager.getInstance();
            manager.reloadConfig(configName);
        end
    end
end