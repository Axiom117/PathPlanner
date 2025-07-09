# PathPlanner Configuration Management System

This system provides a unified configuration management solution that automatically reads and loads all system parameters, keeping the workspace clean and organized.

## 📁 Directory Structure

```
src/
├── servers/
│   ├── PathPlannerClient.m      # Enhanced client (with integrated configuration management)
│   ├── PathPlannerEventData.m   # Event data class
│   └── ConfigManager.m          # Core configuration manager class
└── ...

configs/                         # Configuration files directory
├── robot_params.m              # Robot parameters
├── sim_params.m                # Simulation parameters  
├── controller_params.m         # Controller parameters
└── comm_params.m               # Communication parameters
```

## 🚀 Quick Start

### Basic Usage

```matlab
% 1. Create client (loads all configurations automatically)
client = PathPlannerClient();

% 2. Configurations are loaded into object properties
fprintf('Host: %s, Port: %d\n', client.controllerHost, client.controllerPort);
fprintf('Manipulator IDs: %s, %s\n', client.manipulatorID1, client.manipulatorID2);

% 3. Connect to server
client.connect();
```

### Custom Configuration

```matlab
% Use a custom configuration path
client = PathPlannerClient('configPath', 'my_configs');

% Override specific parameters
client = PathPlannerClient('host', '192.168.1.100', 'port', 6000);

% Disable automatic configuration loading
client = PathPlannerClient('loadConfig', false);
```

## 📋 Configuration Files Description

### 1. robot\_params.m - Robot Parameters

```matlab
% Geometric Parameters
CubeLength = 10;        % Cube size [mm]
TipDistance = 20;       % Tip separation [mm]
TiltAngle = -pi/4;      % Tilt angle [rad]

% Joint Limits
X_min = -10; X_max = 10;   % X-axis limits [mm]
Y_min = -10; Y_max = 10;   % Y-axis limits [mm]
Z_min = -15; Z_max = 15;   % Z-axis limits [mm]
Phi_min = -pi/4; Phi_max = pi/4;  % Phi angle limits [rad]
```

### 2. sim\_params.m - Simulation Parameters

```matlab
% Time Settings
SimTime = 0.4;          % Total simulation time [s]
TimeStep = 0.01;        % Time step [s]

% Model Names
modelFK = 'model_2R_RCM_FK';  % Forward kinematics model
modelIK = 'model_2R_RCM_IK';  % Inverse kinematics model

% Initial Conditions
X0 = 0; Y0 = 0; Z0 = 0; Phi0 = 0;  % Initial pose
```

### 3. controller\_params.m - Controller Parameters

```matlab
% Manipulator IDs
manipulatorID1 = 'MC1';
manipulatorID2 = 'MC2';

% Initial Positions [μm]
XMC1 = 0; YMC1 = 0; ZMC1 = 0;
XMC2 = 0; YMC2 = 0; ZMC2 = 0;

% Resolution Settings [μm/pulse]
ResolutionX = 0.1;
ResolutionY = 0.1;
ResolutionZ = 0.1;
```

### 4. comm\_params.m - Communication Parameters

```matlab
% TCP/IP Settings
ControllerHost = '127.0.0.1';
ControllerPort = 5000;
ConnectionTimeout = 10;     % Connection timeout [s]
ResponseTimeout = 5;        % Response timeout [s]

% Error Handling
MaxRetryAttempts = 3;
EnableAutoReconnect = true;
```

## 🔧 Using ConfigManager Independently

```matlab
% Create a configuration manager
configMgr = ConfigManager();

% Load specific configurations
robotConfig = configMgr.loadConfig('robot');
commConfig = configMgr.loadConfig('communication');

% Retrieve a specific parameter
cubeLength = configMgr.getParameter('robot', 'CubeLength', 10);

% Set a parameter value
configMgr.setParameter('simulation', 'SimTime', 0.8);

% Export configurations to workspace
configMgr.exportToWorkspace('robot');

% Save a configuration file
configMgr.saveConfig('simulation');
```

## 🎯 Advanced Features

### Dynamic Configuration Updates

```matlab
% Update configuration parameters at runtime
client.updateConfiguration('communication', 'ControllerPort', 7000);
client.updateConfiguration('simulation', 'SimTime', 0.8);

% Reload all configurations
client.reloadConfiguration();
```

### Event Listeners

```matlab
% Add event listeners
addlistener(client, 'StatusUpdate', @(src, evt) ...
    fprintf('[Status] %s\n', evt.Message));

addlistener(client, 'ConfigurationLoaded', @(src, evt) ...
    fprintf('[Configuration] %s\n', evt.Message));

addlistener(client, 'ConnectionStateChanged', @(src, evt) ...
    fprintf('[Connection] State changed\n'));
```

### Static Methods

```matlab
% Retrieve a global parameter (singleton pattern)
cubeLength = ConfigManager.getGlobalParameter('robot', 'CubeLength', 10);

% Load all global configurations
ConfigManager.loadGlobalConfigs();
```

### Workspace Management

```matlab
% Clear the workspace (keeping specified variables)
configMgr.clearWorkspace({'client', 'important_var'});

% Export specific configuration to workspace
configMgr.exportToWorkspace('robot');  % Export only robot parameters
configMgr.exportToWorkspace();         % Export all configurations
```

## 🔄 Migration Guide

### From the Old System

**Old Approach:**

```matlab
% Manually run parameter scripts
run('params.m');

% Manually create client instance
client = PathPlannerClient('127.0.0.1', 5000);
```

**New Approach:**

```matlab
% Automatically load all configurations
client = PathPlannerClient();
```

### Integrating Existing Parameter Files

1. Split existing `params.m` contents into the respective configuration files
2. Place parameters into corresponding files based on their type:

   * Geometric parameters → `robot_params.m`
   * Simulation settings → `sim_params.m`
   * Communication settings → `comm_params.m`
   * Control settings → `controller_params.m`

## 📝 Best Practices

### 1. Organizing Configuration Files

* Separate configurations by functional modules
* Use clear and descriptive parameter names
* Include appropriate comments and documentation
* Annotate units for each parameter

### 2. Parameter Validation

```matlab
% Add validation checks in configuration files
if CubeLength <= 0
    error('CubeLength must be positive');
end

if ~ismember(RotationSequence, {'ZYX', 'XYZ', 'ZXZ'})
    error('Invalid rotation sequence');
end
```

### 3. Version Control

* Include configuration files in version control
* Create backups for critical configurations
* Use template files for default configurations

### 4. Environment-Specific Configurations

```matlab
% Create environment-specific configuration files
% configs/dev_comm_params.m     - Development environment
% configs/prod_comm_params.m    - Production environment
% configs/test_comm_params.m    - Testing environment

% Load configurations based on environment
env = getenv('MATLAB_ENV');
if strcmp(env, 'production')
    client = PathPlannerClient('configPath', 'configs/production');
else
    client = PathPlannerClient(); % Default development configuration
end
```

## 🐛 Troubleshooting

### Common Issues

**1. Missing Configuration File**

```
Warning: Configuration file not found: configs/robot_params.m
```

* **Solution:** Creating a `ConfigManager()` instance will auto-generate default configuration files.

**2. Parameter Not Found**

```
Warning: Parameter robot.InvalidParam not found, using default value
```

* **Solution:** Verify the parameter name spelling or add the missing parameter to the appropriate file.

**3. Workspace Variable Conflict**

```
Error: Variable 'SimTime' already exists in workspace
```

* **Solution:** Use `clearWorkspace` option or manually clear conflicting variables.

### Debugging Tips

```matlab
% List all loaded configurations
configMgr.listConfigs();

% Inspect a specific configuration
robotConfig = configMgr.getConfig('robot');
disp(robotConfig);

% Validate configuration integrity
try
    client = PathPlannerClient();
    fprintf('Configuration validated successfully\n');
catch ME
    fprintf('Configuration error: %s\n', ME.message);
end
```

## 🔮 Extensibility

### Adding New Configuration Categories

1. Create a new configuration file:

```matlab
% configs/sensor_params.m
SensorType = 'IMU';
SampleRate = 100;  % Hz
FilterCutoff = 10; % Hz
```

2. Register in `ConfigManager`:

```matlab
% Update DEFAULT_CONFIG_FILES
DEFAULT_CONFIG_FILES = struct(...
    'robot', 'robot_params.m', ...
    'simulation', 'sim_params.m', ...
    'controller', 'controller_params.m', ...
    'communication', 'comm_params.m', ...
    'sensor', 'sensor_params.m');  % Added
```

3. Apply in `PathPlannerClient`:

```matlab
function applySensorConfigParameters(obj)
    % Apply sensor configuration parameters
    sensorConfig = obj.configManager.getConfig('sensor');
    if ~isempty(sensorConfig)
        obj.sensorType = obj.getConfigValue(sensorConfig, 'SensorType', 'None');
        obj.sampleRate = obj.getConfigValue(sensorConfig, 'SampleRate', 100);
    end
end
```

### Custom Configuration Loader

```matlab
% Implement a custom configuration loader
classdef CustomConfigLoader < ConfigManager
    methods
        function config = loadSpecialConfig(obj, fileName)
            % Load configurations in a special format, e.g. JSON, XML, INI
        end
    end
end
```

## 📊 Performance Optimization

* **Configuration Caching:** Loaded configurations are cached to avoid redundant file reads
* **Lazy Loading:** Load specific configurations only when needed
* **Singleton Pattern:** Use a single global configuration manager to save memory

## 🛡️ Security Considerations

* Store sensitive parameters (e.g., passwords) in a separate secure configuration file
* Apply file permission controls to restrict access
* Perform parameter value validation and range checking

---

## 📞 Support

If you have questions or suggestions, please contact the development team or refer to the project documentation.
