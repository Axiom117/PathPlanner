function [qTime, qData, elapsedTime] = solverIK(trajectoryData, model)
    % solverIK - Pure inverse kinematics calculation using Simulink simulation
    % Focused solely on kinematic computation without data preprocessing
    %
    % Inputs:
    %   trajectoryData - Structure containing:
    %                    .poses - Nx6 matrix of interpolated poses [x y z phi theta psi]
    %                    .time  - Nx1 time vector [s]
    %                    .dt    - Time step [s]
    %   model          - Simulink IK model name
    %
    % Outputs:
    %   qTime       - Time vector for joint displacements [s]
    %   qData       - Nx6 matrix of displacements [x1 y1 z1 x2 y2 z2] [m]
    %   elapsedTime - Simulation execution time [s]
    %
    % This function performs pure kinematic calculation:
    %   1. Validates inputs and loads Simulink model
    %   2. Prepares trajectory data for simulation
    %   3. Executes Simulink-based IK calculation
    %   4. Extracts and returns displacement results

    try
        %% 1. Input validation and model preparation
        % Validate trajectory data structure
        if ~isstruct(trajectoryData) || ~isfield(trajectoryData, 'poses') || ...
           ~isfield(trajectoryData, 'time') || ~isfield(trajectoryData, 'dt')
            error('Invalid trajectoryData structure. Required fields: poses, time, dt');
        end
        
        poses = trajectoryData.poses;
        timeVector = trajectoryData.time;
        dt = trajectoryData.dt;
        
        % Validate dimensions
        if size(poses, 2) ~= 6
            error('Pose trajectory must have 6 columns [x y z phi theta psi]');
        end
        
        if length(timeVector) ~= size(poses, 1)
            error('Time vector length must match number of pose points');
        end
        
        % Ensure model is loaded
        if ~bdIsLoaded(model)
            load_system(model);
        end
        
        %% 2. Prepare simulation data
        % Create timeseries for pose trajectory
        pose_ts = timeseries(poses, timeVector);
        pose_ts.Name = 'pose';
        
        % Export to base workspace for Simulink access
        assignin('base', 'pose', pose_ts);
        
        % Calculate simulation parameters
        T = timeVector(end);  % Total simulation time
        
        %% 3. Configure and execute Simulink simulation
        % Set simulation parameters
        set_param(model, 'StopTime', num2str(T));
        if ~strcmpi(get_param(model, 'FastRestart'), 'on')
            set_param(model, 'FixedStep', num2str(dt));
        end
        set_param(model, 'FastRestart', 'on');
        
        % Execute IK simulation
        tic;
        simOut = sim(model);
        elapsedTime = toc;
        
        %% 4. Extract simulation results
        % Get displacement output from simulation
        q = simOut.get('q');
        qData = q.Data;    % Joint displacements in meters
        qTime = q.time;    % Time vector
        
        % Validate output dimensions
        if size(qData, 2) ~= 6
            error('Simulation output must have 6 columns [x1 y1 z1 x2 y2 z2]');
        end
        
        % Clean up workspace variables created for simulation
        % evalin('base', 'clear pose');
        
    catch ME
        % Clean up on error
        try
            % evalin('base', 'clear pose');
        catch
            % Silent cleanup failure
        end
        
        % Re-throw with context
        error('solverIK:SimulationFailed', ...
            'IK simulation failed for model %s: %s', model, ME.message);
    end
end