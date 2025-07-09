function [qTime, qData, elapsedTime] = onCalcIK(obj)
    % onCalcIK - Enhanced trajectory planning with configuration management
    % Handles parameter loading, pose interpolation, and coordinates IK calculation
    %
    % Inputs:
    %   client - PathPlannerClient instance containing target positions and configuration
    %   id1    - Primary manipulator identifier
    %   id2    - Secondary manipulator identifier  
    %   model  - IK model name for Simulink simulation
    %
    % Outputs:
    %   qTime       - Time vector for trajectory [s]
    %   qData       - Joint trajectory data (Nx6) [m]
    %   elapsedTime - Total computation time [s]
    %
    % This function orchestrates the complete IK workflow:
    %   1. Load configuration parameters and export to workspace for Simulink
    %   2. Get current pose via FK
    %   3. Generate interpolated trajectory
    %   4. Execute IK calculation via solverIK
    %   5. Clean workspace selectively

    startTime = tic;
    
    try
        %% 1. Setup configuration for Simulink compatibility
        configMgr = ConfigManager.getInstance();
        workspaceManager = WorkspaceManager();
        
        % Smart export with tracking for required configurations
        workspaceManager.smartExportConfig(configMgr, 'robot');
        workspaceManager.smartExportConfig(configMgr, 'simulation');
        workspaceManager.smartExportConfig(configMgr, 'controller');
        
        % Get simulation parameters directly from config
        simConfig = configMgr.getConfig('simulation');
        T = simConfig.simTime;
        dt = simConfig.timeStep;
        model = simConfig.modelIK;

        % Notify start of trajectory planning
        notify(obj, 'StatusUpdate', ...
            PathPlannerEventData(sprintf('Starting trajectory planning using model: %s', model)));
        
        %% 2. Get current pose through forward kinematics
        currentPose = [obj.config.X0, obj.config.Y0, obj.config.Z0, ...
            obj.config.Phi0, obj.config.Theta0, obj.config.Psi0];
        
        %% 3. Prepare target pose from client properties
        targetPose = [obj.config.XTarget, obj.config.YTarget, obj.config.ZTarget, ...
            obj.config.PhiTarget, obj.config.ThetaTarget, obj.config.PsiTarget];
        
        %% 4. Generate interpolated trajectory
        trajectoryData = generateTrajectory(currentPose, targetPose, T, dt);
        
        notify(obj, 'StatusUpdate', ...
            PathPlannerEventData(sprintf('Generated %d-point trajectory from current to target pose', ...
            length(trajectoryData.time))));

        %% 5. Execute inverse kinematics calculation
        [qTime, qData, ikElapsedTime] = solverIK(trajectoryData, model);
        
        % Calculate total elapsed time
        elapsedTime = toc(startTime);
        
        %% 6. Selective workspace cleanup
        workspaceManager.cleanupWorkspace();
        
        % Notify successful completion
        notify(obj, 'StatusUpdate', ...
            PathPlannerEventData(sprintf('Trajectory planning completed: %d points in %.3f seconds (IK: %.3f s)', ...
            length(qTime), elapsedTime, ikElapsedTime)));
        
    catch ME
        % Ensure cleanup on error
        if exist('workspaceManager', 'var')
            workspaceManager.cleanupWorkspace();
        end
        
        % Re-throw with enhanced error message
        error('onCalcIK:PlanningFailed', ...
            'Trajectory planning failed for model %s: %s', model, ME.message);
    end
end

function trajectoryData = generateTrajectory(currentPose, targetPose, T, dt)
    % Generate interpolated trajectory from current to target pose

    % Create time vector
    timeVector = (0:dt:T)';
    numPoints = length(timeVector);
    
    % Interpolate poses from current to target
    trajectoryPoses = zeros(numPoints, 6);
    for i = 1:6
        trajectoryPoses(:, i) = interp1([0, T], [currentPose(i), targetPose(i)], ...
                                       timeVector, 'linear');
    end
        
    % Create trajectory structure for solverIK
    trajectoryData = struct();
    trajectoryData.poses = trajectoryPoses;  % [m, rad]
    trajectoryData.time = timeVector;
    trajectoryData.dt = dt;
end