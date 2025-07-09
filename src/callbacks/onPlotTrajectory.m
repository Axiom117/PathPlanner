function onPlotTrajectory(obj, trajectoryData, varargin)
% ONPLOTTRAJECTORY Plot dual manipulator trajectory data
%
% SYNTAX:
%   onPlotTrajectory(obj, trajectoryData)
%   onPlotTrajectory(obj, trajectoryData, Name, Value)
%
% INPUTS:
%   obj            - App object containing UIAxes1 and UIAxes2 properties
%   trajectoryData - Structure containing trajectory data with fields:
%                    .qTime - Time vector (Nx1)
%                    .qData - Trajectory data matrix (Nx6)
%                             Columns 1-3: Manipulator 1 (X1, Y1, Z1)
%                             Columns 4-6: Manipulator 2 (X2, Y2, Z2)
%
% OPTIONAL NAME-VALUE PARAMETERS:
%   'PlotColors'     - Cell array of line colors (default: {'r','g','b'})
%   'LineWidth'      - Line width for plots (default: 1.5)
%   'MarkerSize'     - Size of circular markers at data points (default: 4)
%   'MarkerInterval' - Interval between markers (default: 1, every point)
%                      Set to 10 to show marker every 10th point
%
% EXAMPLES:
%   onPlotTrajectory(app, trajData);
%   onPlotTrajectory(app, trajData, 'LineWidth', 2.0);
%   onPlotTrajectory(app, trajData, 'MarkerInterval', 10);
%   onPlotTrajectory(app, trajData, 'MarkerSize', 6, 'MarkerInterval', 5);

    % Parse optional input arguments
    p = inputParser;
    addParameter(p, 'PlotColors', {'r','g','b'}, @iscell);
    addParameter(p, 'LineWidth', 1.5, @isnumeric);
    addParameter(p, 'MarkerSize', 5, @isnumeric);
    addParameter(p, 'MarkerInterval', 10, @(x) isnumeric(x) && x >= 1);
    parse(p, varargin{:});
    
    plotColors = p.Results.PlotColors;
    lineWidth = p.Results.LineWidth;
    markerSize = p.Results.MarkerSize;
    markerInterval = round(p.Results.MarkerInterval);
    
    % Extract trajectory data
    steps = 0:size(trajectoryData.qTime, 1)-1;
    qData = trajectoryData.qData * 1e3;
    
    % Get axes from app object
    axes1 = obj.UIAxes1;
    axes2 = obj.UIAxes2;
    
    % Clear previous plots
    cla(axes1, 'reset');
    cla(axes2, 'reset');
    
    % Calculate marker indices for sparse plotting
    numPoints = length(steps);
    markerIndices = 1:markerInterval:numPoints;
    
    % Calculate Y-axis limits with 5% margin
    ymin = min(qData(:));
    ymax = max(qData(:));
    margin = 0.05 * (ymax - ymin);
    if margin == 0
        margin = 1;
    end
    ylims = [ymin - margin, ymax + margin];

    % Plot Manipulator 1 trajectories (columns 1-3)
    plot(axes1, steps, qData(:,1), plotColors{1}, 'LineWidth', lineWidth, 'DisplayName', 'X_1');
    hold(axes1, 'on');
    plot(axes1, steps, qData(:,2), plotColors{2}, 'LineWidth', lineWidth, 'DisplayName', 'Y_1');
    plot(axes1, steps, qData(:,3), plotColors{3}, 'LineWidth', lineWidth, 'DisplayName', 'Z_1');
    plot(axes1, steps(markerIndices), qData(markerIndices,1), 'o', 'Color', plotColors{1}, 'MarkerSize', markerSize, 'HandleVisibility', 'off');
    plot(axes1, steps(markerIndices), qData(markerIndices,2), 'o', 'Color', plotColors{2}, 'MarkerSize', markerSize, 'HandleVisibility', 'off');
    plot(axes1, steps(markerIndices), qData(markerIndices,3), 'o', 'Color', plotColors{3}, 'MarkerSize', markerSize, 'HandleVisibility', 'off');
    hold(axes1, 'off');
    
    % Configure axes 1 properties
    title(axes1, 'Manipulator 1 Trajectory');
    xlabel(axes1, 'Point Number');
    ylabel(axes1, 'Displacement [mm]');
    legend(axes1, 'Location', 'best');
    grid(axes1, 'on');
    ylim(axes1, ylims);

    % Plot Manipulator 2 trajectories (columns 4-6)
    plot(axes2, steps, qData(:,4), plotColors{1}, 'LineWidth', lineWidth, 'DisplayName', 'X_2');
    hold(axes2, 'on');
    plot(axes2, steps, qData(:,5), plotColors{2}, 'LineWidth', lineWidth, 'DisplayName', 'Y_2');
    plot(axes2, steps, qData(:,6), plotColors{3}, 'LineWidth', lineWidth, 'DisplayName', 'Z_2');
    plot(axes2, steps(markerIndices), qData(markerIndices,4), 'o', 'Color', plotColors{1}, 'MarkerSize', markerSize, 'HandleVisibility', 'off');
    plot(axes2, steps(markerIndices), qData(markerIndices,5), 'o', 'Color', plotColors{2}, 'MarkerSize', markerSize, 'HandleVisibility', 'off');
    plot(axes2, steps(markerIndices), qData(markerIndices,6), 'o', 'Color', plotColors{3}, 'MarkerSize', markerSize, 'HandleVisibility', 'off');
    hold(axes2, 'off');
    
    % Configure axes 2 properties
    title(axes2, 'Manipulator 2 Trajectory');
    xlabel(axes2, 'Point Number');
    ylabel(axes2, 'Displacement [mm]');
    legend(axes2, 'Location', 'best');
    grid(axes2, 'on');
    ylim(axes2, ylims);
end