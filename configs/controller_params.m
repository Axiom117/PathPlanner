%% controller_params.m - Controller Configuration Function
function config = controller_params()
    % Controller Parameters Configuration
    % Returns a structure containing all controller-related parameters
    
    % Manipulator IDs
    config.manipulatorID1 = 'MC1';
    config.manipulatorID2 = 'MC2';
    
    % Initial Positions [μm]
    config.XMC1 = 0;                % MC1 initial X position
    config.YMC1 = 0;                % MC1 initial Y position
    config.ZMC1 = 0;                % MC1 initial Z position
    config.XMC2 = 0;                % MC2 initial X position
    config.YMC2 = 0;                % MC2 initial Y position
    config.ZMC2 = 0;                % MC2 initial Z position
    
    % Control Parameters
    config.controllerGain = 1.0;            % Controller gain
    config.maxControlEffort = 100;          % Maximum control effort
    config.controlFrequency = 100;          % Control loop frequency [Hz]
    
    % Resolution Parameters [μm/pulse]
    config.resolutionX = 0.1;               % X-axis resolution
    config.resolutionY = 0.1;               % Y-axis resolution
    config.resolutionZ = 0.1;               % Z-axis resolution
    
    % PID Parameters
    config.Kp = [1.0, 1.0, 1.0];          % Proportional gains [X, Y, Z]
    config.Ki = [0.1, 0.1, 0.1];          % Integral gains [X, Y, Z]
    config.Kd = [0.01, 0.01, 0.01];       % Derivative gains [X, Y, Z]
    
    % Filter Parameters
    config.enableLowPassFilter = true;     % Enable low-pass filtering
    config.filterCutoffFreq = 50;          % Filter cutoff frequency [Hz]
    config.filterOrder = 2;                % Filter order
    
    % Homing Parameters
    config.homingSpeed = 10;               % Homing speed [mm/s]
    config.homingAcceleration = 50;        % Homing acceleration [mm/s²]
    config.homePositionTolerance = 0.01;   % Home position tolerance [mm]
    
    fprintf('Controller configuration loaded with %d parameters\n', length(fieldnames(config)));
end