clear;
clc;

% Create workspaceManager instance and obtain the configManager
% instance
workspaceManager = WorkspaceManager();
configMgr = ConfigManager.getInstance();

% Export required configurations for Simulink FK model
workspaceManager.smartExportConfig(configMgr, 'robot');
workspaceManager.smartExportConfig(configMgr, 'simulation');

qDisplacement = [0, 0, 0, 0, 0, 0];

model = 'model_3T1R_Roll_FK';

[pose_m, elapsedTime] = solverFK(qDisplacement, model);