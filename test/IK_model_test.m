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

currentPose = [0, 0, 0, 0, -45, 0];
targetPose = [0, 0, 0, 0, -45, 0];

% Create time vector
timeVector = (0:dt:T)';
numPoints = length(timeVector);

% Construct the time series of the pose
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

model = 'model_3T2R_Pitch_Roll_IK';

[qTime, qData, ikElapsedTime] = solverIK(trajectoryData, model);