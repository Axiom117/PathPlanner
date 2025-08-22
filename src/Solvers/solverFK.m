function [pose, elapsedTime] = solverFK(q_input, model)
% solverFK  Forward‐kinematics via Simulink/Simscape Multibody
%
%   [pose, elapsedTime] = solverFK(q_input, model)
%
% Inputs:
%   q_input  - 1×6 vector [x1 y1 z1 x2 y2 z2] (in meters)
%   model    - string, name of Simulink model (must have Inport 'q_input' 
%              and To Workspace block 'pose' logging the 1×6 output)
%
% Outputs:
%   pose         - 1×6 vector [x y z φ θ ψ]
%   elapsedTime  - wall‐clock simulation time (seconds)

    %–– Load model if needed
    if ~bdIsLoaded(model)
        load_system(model);
    end

    %–– Prepare single‐frame timeseries at t = 0
    ts = timeseries(q_input, 0);
    ts.Name = 'q_input';           
    assignin('base', 'q_input', ts);

    %–– Configure simulation for a single instant
    set_param(model, 'StopTime', '0.01', 'FastRestart', 'on');

    %–– Run simulation
    tic;
    simOut = sim(model);
    elapsedTime = toc;

    %–– Retrieve output
    %    Assumes you have a To Workspace block named 'pose' set to "Structure with time"
    pose_ts = simOut.get('pose');  
    % Data is [time × dims] so extract the only row
    pose = squeeze(pose_ts.Data(end, :));

end
