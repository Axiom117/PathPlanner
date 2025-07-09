%% sim_params.m - Simulation Configuration Function
function config = sim_params()
    % Simulation Parameters Configuration
    % Returns a structure containing all simulation-related parameters
    
    % Time Parameters
    config.simTime = 1;           % Total simulation time [s]
    config.timeStep = 0.001;        % Time step [s]
    config.solverType = 'ode45';    % Solver type
    
    % Model Names
    config.modelFK = 'model_2R_RCM_FK';    % Forward kinematics model
    config.modelIK = 'model_2R_RCM_IK';    % Inverse kinematics model
    
    % Solver Tolerances
    config.relTol = 1e-6;           % Relative tolerance
    config.absTol = 1e-9;           % Absolute tolerance
    config.maxStep = 0.01;          % Maximum step size
    
    % Initial Pose
    config.X0 = 0;                  % Initial X position [mm]
    config.Y0 = 0;                  % Initial Y position [mm]
    config.Z0 = 0;                  % Initial Z position [mm]
    config.Phi0 = 0;                % Initial Phi angle [rad]
    config.Theta0 = 0;              % Initial Theta angle [rad]
    config.Psi0 = 0;                % Initial Psi angle [rad]
    
    % Visualization Parameters
    config.enableVisualization = true;      % Enable 3D visualization
    config.visualizationRate = 10;          % Visualization update rate [Hz]
    config.showTrajectory = true;           % Show trajectory path
    config.trajectoryLength = 1000;         % Maximum trajectory points
    
    % Logging Parameters
    config.enableLogging = true;            % Enable data logging
    config.logFileName = 'simulation_log';  % Log file name (without extension)
    config.logDirectory = 'logs';           % Log directory
    
    fprintf('Simulation configuration loaded with %d parameters\n', length(fieldnames(config)));
end