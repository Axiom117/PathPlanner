function [pose_m, elapsedTime] = onCalcFK(obj, model)
    % onCalcFK - Forward kinematics calculation with configuration management
    % Calculates end-effector pose from current manipulator displacements
    %
    % This function:
    %   1. Gets current manipulator status via client
    %   2. Exports required configurations for Simulink
    %   3. Executes FK calculation via solverFK
    %   4. Updates client pose and cleans workspace

    try
        %% 1. Setup configuration for Simulink

        % Create workspaceManager instance and obtain the configManager
        % instance
        workspaceManager = WorkspaceManager();
        configMgr = ConfigManager.getInstance();
        
        % Export required configurations for Simulink FK model
        workspaceManager.smartExportConfig(configMgr, 'robot');
        workspaceManager.smartExportConfig(configMgr, 'simulation');
        
        % Get current manipulator displacements from the configuration with
        % unit conversion (μm to m) 
        qData = [obj.config.XMC1, obj.config.YMC1, obj.config.ZMC1, ...
            obj.config.XMC2, obj.config.YMC2, obj.config.ZMC2] * 1e-6;

        %% 2. Execute forward kinematics calculation
        [pose_m, elapsedTime] = solverFK(qData, model);
        
        %% 3. Cleanup workspace
        workspaceManager.cleanupWorkspace();
        
    catch ME
        % Ensure cleanup on error
        if exist('workspaceManager', 'var')
            workspaceManager.cleanupWorkspace();
        end
        
        error('onCalcFK:CalculationFailed', ...
            'FK calculation failed for model %s: %s', model, ME.message);
    end
end

