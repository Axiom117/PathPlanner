function onVisualizePose(obj, param, varargin)
% ONVISUALIZEPOSE Visualize World, Tool (Current), and Target Coordinate Frames
% ... (existing comments) ...

    % Parse optional inputs
    p = inputParser;
    addParameter(p, 'ShowTarget', false, @islogical);
    parse(p, varargin{:});
    showTarget = p.Results.ShowTarget;

    % Access app GUI object
    ax = obj.VisualizePanel;
    
    % Store current view to prevent reset on update
    [az, el] = view(ax);
    
    % Clear axes but maintain settings
    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    
    % Set axis limits
    viewLimit = 20; 
    xlim(ax, [-viewLimit viewLimit]);
    ylim(ax, [-viewLimit viewLimit]);
    zlim(ax, [-viewLimit viewLimit]);
    
    % Set labels and view
    xlabel(ax, 'X [mm]');
    ylabel(ax, 'Y [mm]');
    zlabel(ax, 'Z [mm]');
    
    % If view hasn't been set (default 2D), set to 3D. Otherwise keep user rotation
    if az == 0 && el == 90
        view(ax, 135, 30);
    else
        view(ax, az, el);
    end

    %% 1. Define Transformation Generation Logic
    function T = getTransform(x_m, y_m, z_m, phi_deg, theta_deg, psi_deg)
        x = x_m * 1000; % Convert meters to mm
        y = y_m * 1000;
        z = z_m * 1000;
        
        ph = deg2rad(phi_deg);
        th = deg2rad(theta_deg);
        ps = deg2rad(psi_deg);
        
        Rx = [1 0 0; 0 cos(ph) -sin(ph); 0 sin(ph) cos(ph)];
        Ry = [cos(th) 0 sin(th); 0 1 0; -sin(th) 0 cos(th)];
        Rz = [cos(ps) -sin(ps) 0; sin(ps) cos(ps) 0; 0 0 1];
        
        R = Rz * Ry * Rx;
        T = eye(4);
        T(1:3, 1:3) = R;
        T(1:3, 4) = [x; y; z];
    end

    %% 2. Define Plot Logic
    function h = plotFrame(T, label, scale, style, alphaVal, isWorld)
        if nargin < 6; isWorld = false; end
        origin = T(1:3, 4);
        R = T(1:3, 1:3);
        
        % Positive Axis vectors
        u = R(:,1) * scale;
        v = R(:,2) * scale;
        w = R(:,3) * scale;
        
        % Define Colors
        if isWorld
            % Orange for World Frame
            cx = [1 0.5 0]; cy = [1 0.5 0]; cz = [1 0.5 0];
        else
            % RGB for Tool/Target Frames
            cx = [1 0 0]; cy = [0 1 0]; cz = [0 0 1];
        end

        % Plot Positive Axes (Arrows)
        h = quiver3(ax, origin(1), origin(2), origin(3), u(1), u(2), u(3), ...
            'Color', [cx alphaVal], 'LineWidth', 2, 'LineStyle', style, 'MaxHeadSize', 0.5, 'AutoScale', 'off', 'DisplayName', label);
        quiver3(ax, origin(1), origin(2), origin(3), v(1), v(2), v(3), ...
            'Color', [cy alphaVal], 'LineWidth', 2, 'LineStyle', style, 'MaxHeadSize', 0.5, 'AutoScale', 'off', 'HandleVisibility', 'off');
        quiver3(ax, origin(1), origin(2), origin(3), w(1), w(2), w(3), ...
            'Color', [cz alphaVal], 'LineWidth', 2, 'LineStyle', style, 'MaxHeadSize', 0.5, 'AutoScale', 'off', 'HandleVisibility', 'off');
        
        % Add Axis Labels at tips
        text(ax, origin(1)+u(1), origin(2)+u(2), origin(3)+u(3), ' X', 'Color', 'w', 'FontSize', 7);
        text(ax, origin(1)+v(1), origin(2)+v(2), origin(3)+v(3), ' Y', 'Color', 'w', 'FontSize', 7);
        text(ax, origin(1)+w(1), origin(2)+w(2), origin(3)+w(3), ' Z', 'Color', 'w', 'FontSize', 7);

        % Plot Origin Point
        plot3(ax, origin(1), origin(2), origin(3), 'k.', 'MarkerSize', 10);
    end

    %% 3. Plot Frames
    % World Frame (Reference)
    T_World = eye(4);
    worldFrameScale = 15; % Scale for frame axes
    hWorld = plotFrame(T_World, 'World Frame', worldFrameScale, '-', 1.0, true);

    % Current Pose (Tool Frame)
    toolFrameScale = 10; % Scale for frame axes
    T_Current = getTransform(param.X0, param.Y0, param.Z0, ...
                             param.Phi0, param.Theta0, param.Psi0);
    hCurrent = plotFrame(T_Current, 'Tool Frame', toolFrameScale, '-', 0.5);
    
    handles = [hWorld, hCurrent];

    % Target Pose (Ghost Frame)
    if showTarget
        T_Target = getTransform(param.XTarget, param.YTarget, param.ZTarget, ...
                                param.PhiTarget, param.ThetaTarget, param.PsiTarget);
        hTarget = plotFrame(T_Target, 'Target Frame', toolFrameScale, ':', 0.5);
        
        handles = [handles, hTarget];

        % Draw connection line
        currPos = T_Current(1:3, 4);
        targPos = T_Target(1:3, 4);
        plot3(ax, [currPos(1), targPos(1)], [currPos(2), targPos(2)], [currPos(3), targPos(3)], ...
            'k:', 'LineWidth', 1);
    end
    
    legend(ax, handles, 'TextColor', 'w', 'Location', 'northeast', 'Color', 'none');
    hold(ax, 'off');
end