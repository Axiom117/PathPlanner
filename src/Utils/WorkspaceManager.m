classdef WorkspaceManager < handle
    % WorkspaceManager - Intelligent workspace variable management
    % Provides safe cleanup of configuration variables while preserving user data
    %
    % Features:
    %   - Tracks configuration variables added to workspace
    %   - Selective cleanup that preserves user variables
    %   - Backup and restore functionality
    %   - Integration with ConfigManager
    %
    % Usage:
    %   wm = WorkspaceManager();
    %   % ... add variables to workspace ...
    %   wm.cleanupWorkspace();  % Clean only config variables
    
    properties (Access = private)
        originalVars        % Variables present before configuration loading
        configVars         % Variables added by configuration loading
        backupData         % Backup of original variables (if overwritten)
        isInitialized      % Initialization status
    end
    
    properties (Constant, Access = private)
        % Configuration parameter patterns to identify config variables
        CONFIG_PATTERNS = {...
            '^[a-z][a-zA-Z0-9]*$', ...       % camelCase parameters
            '^[A-Z][a-zA-Z0-9]*$', ...       % PascalCase parameters
            '^[a-zA-Z]+_[a-zA-Z0-9_]*$'...   % snake_case parameters
            '^[XYZ]\d*$', ...                % Coordinate variables (X0, Y1, etc.)
            '^Phi\d*$', '^Theta\d*$', '^Psi\d*$'  % Angle variables
        };
        
        % Variables to always preserve (never clean up)
        PRESERVE_VARS = {'ans', 'obj', 'client', 'configMgr'};
    end
    
    methods (Access = public)
        function obj = WorkspaceManager()
            % Constructor - Initialize workspace tracking
            obj.captureOriginalWorkspace();
            obj.isInitialized = true;
        end
        
        function captureOriginalWorkspace(obj)
            % Capture current workspace state before configuration loading
            obj.originalVars = evalin('base', 'who');
            obj.configVars = {};
            obj.backupData = struct();
        end
        
        function trackConfigExport(obj, configName)
            % Track variables exported by ConfigManager for a specific configuration
            %
            % Input:
            %   configName - Name of configuration that was exported
            
            if ~obj.isInitialized
                return;
            end
            
            % Get current workspace variables
            currentVars = evalin('base', 'who');
            
            % Identify newly added variables
            newVars = setdiff(currentVars, obj.originalVars);
            
            % Filter to likely configuration variables
            configVarsFromExport = obj.filterConfigVariables(newVars);
            
            % Add to tracked config variables
            obj.configVars = union(obj.configVars, configVarsFromExport);
            
            if ~isempty(configVarsFromExport)
                fprintf('WorkspaceManager: Tracking %d variables from %s config\n', ...
                    length(configVarsFromExport), configName);
            end
        end
        
        function cleanupWorkspace(obj)
            % Clean up only configuration variables, preserve user data
            if ~obj.isInitialized || isempty(obj.configVars)
                return;
            end
            
            % Get current workspace variables
            currentVars = evalin('base', 'who');
            
            % Find config variables that still exist
            varsToClean = intersect(obj.configVars, currentVars);
            
            % Remove preserved variables
            varsToClean = setdiff(varsToClean, obj.PRESERVE_VARS);
            
            % Clean up variables
            cleanedCount = 0;
            for i = 1:length(varsToClean)
                try
                    evalin('base', sprintf('clear %s', varsToClean{i}));
                    cleanedCount = cleanedCount + 1;
                catch
                    % Silent failure for individual variables
                end
            end
            
            if cleanedCount > 0
                fprintf('WorkspaceManager: Cleaned %d configuration variables\n', cleanedCount);
            end
            
            % Reset tracking
            obj.configVars = {};
        end
        
        function backupVariable(obj, varName)
            % Backup a variable before it might be overwritten
            %
            % Input:
            %   varName - Name of variable to backup
            
            try
                if evalin('base', sprintf('exist(''%s'', ''var'')', varName))
                    obj.backupData.(varName) = evalin('base', varName);
                end
            catch
                % Silent backup failure
            end
        end
        
        function restoreVariable(obj, varName)
            % Restore a previously backed up variable
            %
            % Input:
            %   varName - Name of variable to restore
            
            if isfield(obj.backupData, varName)
                try
                    assignin('base', varName, obj.backupData.(varName));
                    obj.backupData = rmfield(obj.backupData, varName);
                catch
                    % Silent restore failure
                end
            end
        end
        
        function smartExportConfig(obj, configMgr, configName)
            % Smart export that tracks and manages workspace variables
            %
            % Inputs:
            %   configMgr  - ConfigManager instance
            %   configName - Configuration to export
            
            % Capture state before export
            preExportVars = evalin('base', 'who');
            
            % Export configuration
            configMgr.exportToWorkspace(configName);
            
            % Track what was added
            postExportVars = evalin('base', 'who');
            newVars = setdiff(postExportVars, preExportVars);
            
            % Add to tracked variables
            obj.configVars = union(obj.configVars, newVars);
            
            fprintf('WorkspaceManager: Exported and tracking %d variables from %s\n', ...
                length(newVars), configName);
        end
        
        function listTrackedVariables(obj)
            % Display currently tracked configuration variables
            if isempty(obj.configVars)
                fprintf('No configuration variables currently tracked\n');
                return;
            end
            
            fprintf('Tracked configuration variables (%d):\n', length(obj.configVars));
            for i = 1:length(obj.configVars)
                fprintf('  %s\n', obj.configVars{i});
            end
        end
        
        function reset(obj)
            % Reset workspace manager state
            obj.configVars = {};
            obj.backupData = struct();
            obj.captureOriginalWorkspace();
        end
    end
    
    methods (Access = private)
        function configVars = filterConfigVariables(obj, varNames)
            % Filter variable names to identify likely configuration variables
            %
            % Input:
            %   varNames - Cell array of variable names
            %
            % Returns:
            %   configVars - Filtered list of likely configuration variables
            
            configVars = {};
            
            for i = 1:length(varNames)
                varName = varNames{i};
                
                % Skip preserved variables
                if any(strcmp(varName, obj.PRESERVE_VARS))
                    continue;
                end
                
                % Check against configuration patterns
                isConfigVar = false;
                for j = 1:length(obj.CONFIG_PATTERNS)
                    if ~isempty(regexp(varName, obj.CONFIG_PATTERNS{j}, 'once'))
                        isConfigVar = true;
                        break;
                    end
                end
                
                if isConfigVar
                    configVars{end+1} = varName; %#ok<AGROW>
                end
            end
        end
    end
end