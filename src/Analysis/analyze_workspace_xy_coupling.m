%% analyze_workspace_flexible.m
% 通用 4自由度机构 (X, Y, Z, Any_Angle) 3D工作空间分析
% 支持指定分析 Phi, Theta 或 Psi
function analyze_workspace_xy_coupling(target_axis)
    % 输入参数 target_axis: 'phi', 'theta', 或 'psi'
    if nargin < 1
        target_axis = 'phi'; % 默认为 phi
    end
    
    clc; close all;
    fprintf('=== 启动柔性工作空间分析 ===\n');
    fprintf('当前分析耦合模式: X - Y - %s\n', upper(target_axis));

    % --- 1. 模型与参数设置 ---
    modelName = 'model_3T2R_Pitch_Roll_IK'; 
    
    % 驱动器物理极限 [米]
    actuatorLimits.x = [-10, 10] * 1e-3; 
    actuatorLimits.y = [-10, 10] * 1e-3;
    actuatorLimits.z = [-15, 15] * 1e-3;
    
    % --- 2. 定义扫描范围 ---
    range_X_mm = -40:1:40;  
    range_Y_mm = -40:1:40;
    
    % 定义动态角度的扫描范围 (通用变量)
    range_Angle_deg = -80:1:0; 
    
    % --- 3. 定义其他维度的固定值 ---
    % 这里定义所有自由度的默认固定值
    default_Z_mm = 0;
    default_Phi_deg   = 0; 
    default_Theta_deg = -45; % 比如原本代码里的固定值
    default_Psi_deg   = 0;
    
    % 根据选择的轴，更新固定值显示的逻辑（仅用于提示）
    switch lower(target_axis)
        case 'phi'
            target_col = 4; % Pose矩阵的第4列
            axis_label = 'Phi (Rx) [deg]';
            fixed_info = sprintf('Fixed: Theta=%.1f, Psi=%.1f', default_Theta_deg, default_Psi_deg);
        case 'theta'
            target_col = 5; % Pose矩阵的第5列
            axis_label = 'Theta (Ry) [deg]';
            fixed_info = sprintf('Fixed: Phi=%.1f, Psi=%.1f', default_Phi_deg, default_Psi_deg);
        case 'psi'
            target_col = 6; % Pose矩阵的第6列
            axis_label = 'Psi (Rz) [deg]';
            fixed_info = sprintf('Fixed: Phi=%.1f, Theta=%.1f', default_Phi_deg, default_Theta_deg);
        otherwise
            error('未知的轴名称，请使用 phi, theta, 或 psi');
    end
    fprintf('%s\n', fixed_info);

    % --- 4. 预计算点数 ---
    M = length(range_X_mm);    
    N = length(range_Y_mm);    
    P = length(range_Angle_deg); % 动态角度的步数
    numPoints = M * N * P;     
    
    % 构造极限向量
    L_vec = [max(abs(actuatorLimits.x)), max(abs(actuatorLimits.y)), max(abs(actuatorLimits.z))];
    L_max = [L_vec, L_vec]; 
    L_max_matrix = repmat(L_max, numPoints, 1); 
    
    % --- 5. 构造蛇形轨迹 (Snaking Generation) ---
    % 注意：这里的 Angle_query 代替了原来的 Phi_query
    X_query = zeros(numPoints, 1);
    Y_query = zeros(numPoints, 1);
    Angle_query = zeros(numPoints, 1); 
    current_idx = 1;
    
    for i = 1:P 
        ang_val = range_Angle_deg(i); 
        if mod(i, 2) == 1, range_Y_scan = range_Y_mm; else, range_Y_scan = fliplr(range_Y_mm); end
        
        for j = 1:N 
            y_val = range_Y_scan(j) * 1e-3; 
            if mod(j, 2) == 1, x_scan = range_X_mm * 1e-3; else, x_scan = fliplr(range_X_mm) * 1e-3; end
            
            end_idx = current_idx + M - 1;
            X_query(current_idx:end_idx) = x_scan;
            Y_query(current_idx:end_idx) = y_val;
            Angle_query(current_idx:end_idx) = ang_val; 
            current_idx = end_idx + 1;
        end
    end
    
    % --- 6. 组装 Pose 矩阵 (核心修改点) ---
    dt_per_point = 0.001; 
    T = numPoints * dt_per_point;
    timeVector = (0:dt_per_point:T-dt_per_point)';
    
    % 初始化全为固定值的 Pose 矩阵
    % Columns: [x, y, z, phi, theta, psi]
    Poses = zeros(numPoints, 6);
    Poses(:, 1) = X_query;
    Poses(:, 2) = Y_query;
    Poses(:, 3) = default_Z_mm * 1e-3; % 注意单位转换如果需要
    Poses(:, 4) = default_Phi_deg;
    Poses(:, 5) = default_Theta_deg;
    Poses(:, 6) = default_Psi_deg;
    
    % **核心操作：覆盖动态列**
    Poses(:, target_col) = Angle_query; 
    
    trajectoryData.poses = Poses;
    trajectoryData.time = timeVector;
    trajectoryData.dt = dt_per_point;
    
    try
        [~, qData, elapsedTime] = solverIK(trajectoryData, modelName);
        fprintf('计算完成，耗时: %.2f 秒\n', elapsedTime);
        
        % --- 7. 极限验证 ---
        normalized_q = abs(qData) ./ L_max_matrix; 
        Max_Utilization = max(normalized_q, [], 2); 
        inRange = Max_Utilization <= 1.0; 
        utilization_threshold = 0.9; 
        
        % --- 8. 数据重构与同步 (Un-snaking) ---
        % 注意：这里 Angle 代表Z轴维度，无论是 Phi, Theta 还是 Psi
        Grid_X = reshape(X_query, M, N, P);
        Grid_Y = reshape(Y_query, M, N, P);
        Grid_Angle = reshape(Angle_query, M, N, P); % 动态角度
        Grid_Feasible = reshape(inRange, M, N, P);
        Grid_Util = reshape(Max_Utilization, M, N, P);
        
        % Step A: 修复 X (偶数列反向)
        for j = 1:N
            if mod(j, 2) == 0
                Grid_X(:, j, :) = flipud(Grid_X(:, j, :));
                Grid_Y(:, j, :) = flipud(Grid_Y(:, j, :)); 
                Grid_Angle(:, j, :) = flipud(Grid_Angle(:, j, :));
                Grid_Feasible(:, j, :) = flipud(Grid_Feasible(:, j, :));
                Grid_Util(:, j, :) = flipud(Grid_Util(:, j, :));
            end
        end
        
        % Step B: 修复 Y (偶数层反向)
        for i = 1:P
            if mod(i, 2) == 0
                Grid_X(:, :, i) = fliplr(Grid_X(:, :, i));
                Grid_Y(:, :, i) = fliplr(Grid_Y(:, :, i));
                Grid_Angle(:, :, i) = fliplr(Grid_Angle(:, :, i));
                Grid_Feasible(:, :, i) = fliplr(Grid_Feasible(:, :, i));
                Grid_Util(:, :, i) = fliplr(Grid_Util(:, :, i));
            end
        end
        
        % 维度置换 -> [Y, X, Angle]
        p_order = [2 1 3];
        Final_X = permute(Grid_X, p_order) * 1000; 
        Final_Y = permute(Grid_Y, p_order) * 1000; 
        Final_Angle = permute(Grid_Angle, p_order); % 这里就是选定的角度
        Final_Feasible = permute(Grid_Feasible, p_order);
        Final_Util = permute(Grid_Util, p_order);
        
        % --- 9. 绘图 ---
        % 提取边界 Mask
        if exist('bwperim', 'file')
            Boundary_Mask = bwperim(Final_Feasible, 26);
        else
            Boundary_Mask = Final_Feasible & ~imerode(Final_Feasible, ones(3,3,3));
        end
        
        Scatter_X = Final_X(Boundary_Mask);
        Scatter_Y = Final_Y(Boundary_Mask);
        Scatter_Angle = Final_Angle(Boundary_Mask);
        Scatter_Util = Final_Util(Boundary_Mask);
        
        is_crit = Scatter_Util >= utilization_threshold;
        is_safe = Scatter_Util < utilization_threshold;
        
        figure('Name', ['Workspace Analysis: ' upper(target_axis)], 'Color', 'w'); hold on;
        
        % 绘制透明包络
        Smooth_Data = smooth3(double(Final_Feasible), 'box', 5);
        [F, V] = isosurface(Final_X, Final_Y, Final_Angle, Smooth_Data, 0.5);
        if ~isempty(F)
            patch('Faces', F, 'Vertices', V, 'FaceColor', [0.95 0.95 0.95], ...
                  'EdgeColor', 'none', 'FaceAlpha', 0.1);
        end
        
        if any(is_safe)
            scatter3(Scatter_X(is_safe), Scatter_Y(is_safe), Scatter_Angle(is_safe), ...
                15, [0 0.7 0], 'filled', 'MarkerFaceAlpha', 0);
        end
        if any(is_crit)
            scatter3(Scatter_X(is_crit), Scatter_Y(is_crit), Scatter_Angle(is_crit), ...
                5, [1 0 0], 'filled', 'MarkerFaceAlpha', 0);
        end
        
        grid on; box on; axis equal;
        view(3); camlight; lighting gouraud;
        
        title(sprintf('Workspace: X-Y vs %s', upper(target_axis)));
        xlabel('X Position [mm]');
        ylabel('Y Position [mm]');
        zlabel(axis_label); % 动态标签
        
        xlim([min(range_X_mm)-2, max(range_X_mm)+2]);
        ylim([min(range_Y_mm)-2, max(range_Y_mm)+2]);
        zlim([min(range_Angle_deg), max(range_Angle_deg)]);
        
    catch ME
        fprintf('错误: %s\n', ME.message);
        fprintf('Line: %d\n', ME.stack(1).line);
    end
end