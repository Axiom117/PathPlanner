clear;
clc;

% Create workspaceManager instance and obtain the configManager
% instance
workspaceManager = WorkspaceManager();
configMgr = ConfigManager.getInstance();

% Export required configurations for Simulink FK model
workspaceManager.smartExportConfig(configMgr, 'robot');
workspaceManager.smartExportConfig(configMgr, 'sim');

% Get simulation parameters from config
simConfig = configMgr.getConfig('sim');
        T = simConfig.simTime;
        dt = simConfig.timeStep;

currentPose = [0, 0, 0, 0, 0, 0];
targetPose = [0, 0, 0, 0, 10, 0];

timeVector = (0:dt:T+0.01)';
numPoints = length(timeVector);

trajectoryPoses = zeros(numPoints, 6);

keyTimes = [0, T, T+0.01];

for i = 1:6
    keyValues = [currentPose(i), targetPose(i), targetPose(i)];
    trajectoryPoses(:, i) = interp1(keyTimes, keyValues, ...
        timeVector, 'linear');
end

% Create trajectory structure for solverIK
trajectoryData = struct();
trajectoryData.poses = trajectoryPoses;  % [mm, deg]
trajectoryData.time = timeVector;
trajectoryData.dt = dt;

model = 'model_3T2R_sasa_IK';

[qTime, qData, ikElapsedTime] = solverIK(trajectoryData, model);