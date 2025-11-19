%% analyze_workspace_4dof.m - 4自由度机构 (X,Y,Z,Phi) 3D工作空间分析 (体积+散点叠加)
function analyze_workspace_4dof()
    clc; close all;
    
    % --- 1. 模型与参数设置 ---
    modelName = 'model_2R_RCM_IK'; 
    
    % 驱动器物理极限 [米] (±10mm, ±10mm, ±15mm)
    actuatorLimits.x = [-10, 10] * 1e-3; 
    actuatorLimits.y = [-10, 10] * 1e-3;
    actuatorLimits.z = [-15, 15] * 1e-3;
    
    % --- 2. 定义 3D 扫描网格：X, Y (位移) vs Phi (转动) ---
    range_X_mm = -10:1:10;  % X轴位移范围：-10mm 到 10mm，步长 2mm 
    range_Y_mm = -10:1:10;  % Y轴位移范围：-10mm 到 10mm，步长 2mm 
    range_Phi_deg = -15:1:15; % Phi角度范围：-10度 到 10度，步长 1度 
    
    % 固定其他自由度的值
    fixed_Z = 0;     
    fixed_Theta = 0;      
    fixed_Psi = 0;        
    
    % --- 3. 预计算点数和归一化矩阵 ---
    M = length(range_X_mm);    % X 步数 (M)
    N = length(range_Y_mm);    % Y 步数 (N)
    P = length(range_Phi_deg); % Phi 步数 (P)
    numPoints = M * N * P;     % 总点数 
    
    L_max = [max(abs(actuatorLimits.x)), max(abs(actuatorLimits.y)), max(abs(actuatorLimits.z)), ...
             max(abs(actuatorLimits.x)), max(abs(actuatorLimits.y)), max(abs(actuatorLimits.z))];
    L_max_matrix = repmat(L_max, numPoints, 1); 
    
    % --- 4. 构造平滑的 1D 蛇形轨迹 (三层嵌套循环) ---
    
    X_query = zeros(numPoints, 1);
    Y_query = zeros(numPoints, 1);
    Phi_query = zeros(numPoints, 1); % 存储度数
    current_idx = 1;
    
    for i = 1:P 
        phi_val = range_Phi_deg(i); % 存储度数
        if mod(i, 2) == 1 
            range_Y_scan = range_Y_mm; 
        else 
            range_Y_scan = fliplr(range_Y_mm); 
        end
        for j = 1:N 
            y_val = range_Y_scan(j) * 1e-3; 
            if mod(j, 2) == 1 
                x_scan = range_X_mm * 1e-3; 
            else 
                x_scan = fliplr(range_X_mm) * 1e-3; 
            end
            end_idx = current_idx + M - 1;
            X_query(current_idx:end_idx) = x_scan;
            Y_query(current_idx:end_idx) = y_val;
            Phi_query(current_idx:end_idx) = phi_val; 
            current_idx = end_idx + 1;
        end
    end
    
    % --- 5. 构建 trajectoryData 结构体 ---
    dt_per_point = 0.001; 
    T = numPoints * dt_per_point;
    timeVector = (0:dt_per_point:T-dt_per_point)';
    
    poses = zeros(numPoints, 6);
    poses(:, 1) = X_query;    
    poses(:, 2) = Y_query;    
    poses(:, 3) = fixed_Z;    
    poses(:, 4) = Phi_query;  
    poses(:, 5) = fixed_Theta;
    poses(:, 6) = fixed_Psi;

    trajectoryData.poses = poses;  
    trajectoryData.time = timeVector;
    trajectoryData.dt = dt_per_point;
    
    fprintf('开始分析 4-DOF (X, Y, Phi) 3D 工作空间...\n');
    fprintf('总测试点数: %d\n', numPoints);
    
    try
        % 6. 批量运行 IK 解算
        [~, qData, elapsedTime] = solverIK(trajectoryData, modelName);
        fprintf('计算完成，耗时: %.2f 秒\n', elapsedTime);
        
        % --- 7. 极限检查和数据准备 ---
        normalized_q = abs(qData) ./ L_max_matrix; 
        Max_Utilization = max(normalized_q, [], 2); 
        inRange = Max_Utilization <= 1.0; 
        
        % 提取散点图所需的坐标和利用率
        Feasible_X = X_query(inRange) * 1000; 
        Feasible_Y = Y_query(inRange) * 1000;
        Feasible_Phi = Phi_query(inRange); 
        Feasible_Utilization = Max_Utilization(inRange);
        
        % 定义颜色阈值: 80% (0.8)
        utilization_threshold = 0.9; 
        is_critical = Feasible_Utilization >= utilization_threshold;
        is_safe = Feasible_Utilization < utilization_threshold;
        
        %% 8. 优化可视化：仅绘制边界散点 (Boundary Shell Only)
        
        % 1. 准备网格坐标 (与原代码一致)
        X_axis_mm = range_X_mm;
        Y_axis_mm = range_Y_mm;
        Phi_axis_deg = range_Phi_deg; 
        [Xg, Yg, Zg] = meshgrid(X_axis_mm, Y_axis_mm, Phi_axis_deg); 
        
        % 2. 数据重构与反向蛇形处理 (Un-snaking)
        % 注意：我们需要同时处理 "可行性逻辑矩阵" 和 "利用率矩阵"
        % 这样才能在提取边界后，依然能画出该边界点对应的颜色（安全/危险）
        
        Feasibility_Grid = reshape(inRange, M, N, P); 
        Utilization_Grid = reshape(Max_Utilization, M, N, P); % 新增：利用率也转为网格
        
        % --- 反向蛇形处理 (同步应用到两个矩阵) ---
        for i = 2:2:P 
             Feasibility_Grid(:, :, i) = fliplr(Feasibility_Grid(:, :, i)); 
             Utilization_Grid(:, :, i) = fliplr(Utilization_Grid(:, :, i)); % 同步翻转
        end
        for j = 2:2:N
            for i = 1:P
                Feasibility_Grid(:, j, i) = flipud(Feasibility_Grid(:, j, i)); 
                Utilization_Grid(:, j, i) = flipud(Utilization_Grid(:, j, i)); % 同步翻转
            end
        end
        
        % 3. 维度置换 (Permute) 以匹配 meshgrid
        Feasibility_Grid_Permuted = permute(Feasibility_Grid, [2 1 3]); 
        Utilization_Grid_Permuted = permute(Utilization_Grid, [2 1 3]);
        
        % 4. 核心步骤：提取 3D 边界 (Boundary Extraction)
        % 使用 bwperim 计算 3D 对象的周长（外壳）
        % 26 代表 26-邻域联通性 (包括斜角)，能得到更连续的边缘
        if exist('bwperim', 'file')
            Boundary_Mask = bwperim(Feasibility_Grid_Permuted, 26);
        else
            % 如果没有 Image Processing Toolbox，使用手动算法：
            % 边界 = 原始掩模 AND NOT (原始掩模的腐蚀版本)
            kernel = ones(3,3,3);
            Eroded_Mask = imerode(Feasibility_Grid_Permuted, kernel); % 需要基本图像函数
            Boundary_Mask = Feasibility_Grid_Permuted & ~Eroded_Mask;
        end
        
        % 5. 使用边界掩模提取坐标和数据
        % 仅提取 Mask 为 1 (即位于边界) 的点
        Boundary_X = Xg(Boundary_Mask);
        Boundary_Y = Yg(Boundary_Mask);
        Boundary_Phi = Zg(Boundary_Mask);
        Boundary_Util = Utilization_Grid_Permuted(Boundary_Mask);
        
        % 重新分类边界点的安全性
        is_boundary_critical = Boundary_Util >= utilization_threshold;
        is_boundary_safe = Boundary_Util < utilization_threshold;
        
        % 平滑用于 Isosurface 的数据
        smoothing_kernel_size = 9; 
        Feasibility_Grid_Smoothed = smooth3(double(Feasibility_Grid_Permuted), 'box', smoothing_kernel_size);

        % --- 绘图开始 ---
        figure('Name', '4-DOF Workspace (Boundary Shell Only)', 'Color', 'w');
        hold on; 
        
        % 定义颜色
        edge_color = [0.1 0.1 0.1];     
        
        % 6. 绘制体积 (半透明外壳，作为背景参考)
        % 这里的 FaceAlpha 调低一点，让散点更突出
        [F, V] = isosurface(Xg, Yg, Zg, Feasibility_Grid_Smoothed, 0.5); 
        if ~isempty(F)
            h_volume = patch('Faces', F, 'Vertices', V, ...
                  'FaceColor', [0.9 0.9 0.9], ... % 使用浅灰色作为体积基底
                  'EdgeColor', 'none', ...
                  'FaceAlpha', 0.1); % 非常透明，只提供轮廓感
        else
            h_volume = [];
        end
        
        % 7. 绘制散点图 (只绘制边界点！)
        
        % 边界上的安全点 (绿色)
        h_safe = scatter3(Boundary_X(is_boundary_safe), Boundary_Y(is_boundary_safe), Boundary_Phi(is_boundary_safe), ...
            20, [0 0.8 0], 'filled', 'MarkerFaceAlpha', 0.6); 
        
        % 边界上的临界点 (红色) - 稍微放大一点以示警告
        h_critical = scatter3(Boundary_X(is_boundary_critical), Boundary_Y(is_boundary_critical), Boundary_Phi(is_boundary_critical), ...
            25, [1 0 0], 'filled', 'MarkerFaceAlpha', 0.8); 
        
        % --- 图像修饰 ---
        camlight; 
        lighting gouraud;
        grid on;
        box on;
        
        daspect([1 1 1]); 
        view(3); 
        
        title(sprintf('4-DOF Workspace Boundary Shell (Z = %.1f mm)', fixed_Z*1000));
        xlabel('X Position [mm]');
        ylabel('Y Position [mm]');
        zlabel('Phi Angle [deg]');
        
        xlim([min(X_axis_mm), max(X_axis_mm)]);
        ylim([min(Y_axis_mm), max(Y_axis_mm)]);
        zlim([min(Phi_axis_deg), max(Phi_axis_deg)]);
        
        legend([h_safe, h_critical], ...
            'Boundary Safe (< 90%)', ...
            'Boundary Critical (>= 90%)', ...
            'Location', 'best');
        
    catch ME
        fprintf('错误: %s\n', ME.message);
        if contains(ME.identifier, 'SimulationFailed')
             fprintf('提示: 仿真崩溃。请尝试将 dt_per_point 进一步降低（例如 0.0001s）。\n');
        end
    end
end