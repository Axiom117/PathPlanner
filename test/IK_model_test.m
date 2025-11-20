clear;
clc;

% Create workspaceManager instance and obtain the configManager
% instance
workspaceManager = WorkspaceManager();
configMgr = ConfigManager.getInstance();

configMgr.setParam('simulation', 'simTime', 0.001);

% Export required configurations for Simulink FK model
workspaceManager.smartExportConfig(configMgr, 'robot');
workspaceManager.smartExportConfig(configMgr, 'simulation');

% Get simulation parameters from config
simConfig = configMgr.getConfig('simulation');
        T = simConfig.simTime;
        dt = simConfig.timeStep;

currentPose = [0, 0, 0, 10, -45, 0];
targetPose = [0, 0, 0, 10, -45, 0];

trajectoryPose = [0, 0, 0, 0, -45, 0];

% Create trajectory structure for solverIK
trajectoryData = struct();
trajectoryData.poses = trajectoryPose;  % [m, rad]
trajectoryData.time = T;
trajectoryData.dt = dt;

model = 'model_3T2R_Pitch_Roll_IK';

[qTime, qData, ikElapsedTime] = solverIK(trajectoryData, model);