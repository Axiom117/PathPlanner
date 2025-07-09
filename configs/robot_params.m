%% robot_params.m - Robot Configuration Function
function config = robot_params()
    % M-REx HYPERCUBE Parameters Configuration
    % Returns a structure containing all robot-related parameters
    
    % Geometric Parameters
    config.cubeLength = 10;      % Cube dimension [mm]
    config.tipDistance = 15;     % Pipette tip distance [mm]
    config.tiltAngle = -pi/4;    % Tilt angle around Y-axis [rad]
    
    % Joint Limits
    config.X_min = -10;          % X minimum limit [mm]
    config.X_max = 10;           % X maximum limit [mm]
    config.Y_min = -10;          % Y minimum limit [mm]
    config.Y_max = 10;           % Y maximum limit [mm]
    config.Z_min = -15;          % Z minimum limit [mm]
    config.Z_max = 15;           % Z maximum limit [mm]
    config.Phi_min = -pi/4;      % Phi minimum limit [rad]
    config.Phi_max = pi/4;       % Phi maximum limit [rad]
    
    % Control Parameters
    config.singularityThreshold = 1e-6;        % Singularity detection threshold
    config.rotationSequence = 'ZYX';           % Rotation sequence
    config.angularVelocityLimit = pi/2;        % Maximum angular velocity [rad/s]
    config.linearVelocityLimit = 50;           % Maximum linear velocity [mm/s]
    
    % Safety Parameters
    config.emergencyStopEnabled = true;        % Enable emergency stop
    config.collisionDetectionEnabled = true;   % Enable collision detection
    config.softLimitsEnabled = true;           % Enable software limits
    
    % Display loaded parameters count
    fprintf('Robot configuration loaded with %d parameters\n', length(fieldnames(config)));
end