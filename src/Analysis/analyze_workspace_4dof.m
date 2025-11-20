%% analyze_workspace_4dof.m - 4自由度机构 (X,Y,Z,Phi) 3D工作空间分析
% V4最终版：包含坐标同步修复 + 驱动器数值诊断功能
function analyze_workspace_4dof()
    clc; close all;
    
    % --- 1. 模型与参数设置 ---
    modelName = 'model_2R_RCM_IK'; 
    
    % 驱动器物理极限 [米] (请确认这里是相对位移还是绝对坐标)
    % 假设：Simulink 输出的是相对于零点的位移 (Displacement)
    actuatorLimits.x = [-10, 10] * 1e-3; 
    actuatorLimits.y = [-10, 10] * 1e-3;
    actuatorLimits.z = [-15, 15] * 1e-3;
    
    % --- 2. 定义 3D 扫描网格 ---
    % 提示：如果不希望看到 -20mm 到 20mm 的坐标轴，可以缩小这里的范围
    range_X_mm = -30:1:30;  
    range_Y_mm = -30:1:30;  
    range_Phi_deg = -45:1:45; 
    
    fixed_Z = 0;     
    fixed_Theta = 0;      
    fixed_Psi = 0;        
    
    % --- 3. 预计算点数 ---
    M = length(range_X_mm);    % X 步数
    N = length(range_Y_mm);    % Y 步数
    P = length(range_Phi_deg); % Phi 步数
    numPoints = M * N * P;     
    
    % 构造极限向量 (假设 output = [x1, y1, z1, x2, y2, z2])
    L_vec = [max(abs(actuatorLimits.x)), max(abs(actuatorLimits.y)), max(abs(actuatorLimits.z))];
    L_max = [L_vec, L_vec]; % 重复一次对应两台驱动器
    L_max_matrix = repmat(L_max, numPoints, 1); 
    
    % --- 4. 构造蛇形轨迹 (Snaking Generation) ---
    X_query = zeros(numPoints, 1);
    Y_query = zeros(numPoints, 1);
    Phi_query = zeros(numPoints, 1);
    current_idx = 1;
    
    for i = 1:P 
        phi_val = range_Phi_deg(i); 
        if mod(i, 2) == 1, range_Y_scan = range_Y_mm; else, range_Y_scan = fliplr(range_Y_mm); end
        
        for j = 1:N 
            y_val = range_Y_scan(j) * 1e-3; 
            if mod(j, 2) == 1, x_scan = range_X_mm * 1e-3; else, x_scan = fliplr(range_X_mm) * 1e-3; end
            
            end_idx = current_idx + M - 1;
            X_query(current_idx:end_idx) = x_scan;
            Y_query(current_idx:end_idx) = y_val;
            Phi_query(current_idx:end_idx) = phi_val; 
            current_idx = end_idx + 1;
        end
    end
    
    % --- 5. 运行 IK 解算 ---
    dt_per_point = 0.001; 
    T = numPoints * dt_per_point;
    timeVector = (0:dt_per_point:T-dt_per_point)';
    
    % 组装 Pose 矩阵
    trajectoryData.poses = [X_query, Y_query, ones(numPoints,1)*fixed_Z, Phi_query, ones(numPoints,1)*fixed_Theta, ones(numPoints,1)*fixed_Psi];
    trajectoryData.time = timeVector;
    trajectoryData.dt = dt_per_point;
    
    fprintf('开始分析 4-DOF (X, Y, Phi) 3D 工作空间...\n');
    
    try
        [~, qData, elapsedTime] = solverIK(trajectoryData, modelName);
        fprintf('计算完成，耗时: %.2f 秒\n', elapsedTime);
        
        % --- 6. 极限验证 (Limit Check) ---
        % 注意：这里假设 qData 是位移。如果 qData 是绝对坐标(如 100mm)，需要先减去零位。
        normalized_q = abs(qData) ./ L_max_matrix; 
        
        % 计算每个点最大的利用率 (Max over the 6 actuators)
        Max_Utilization = max(normalized_q, [], 2); 
        
        % 核心判定：利用率 <= 1.0 即为可行
        inRange = Max_Utilization <= 1.0; 
        utilization_threshold = 0.9; % 90% 以上显示红色
        
        %% 7. 数据重构与同步 (V3 逻辑：先内后外，严格对齐)
        
        % Reshape 原始向量
        Grid_X = reshape(X_query, M, N, P);
        Grid_Y = reshape(Y_query, M, N, P);
        Grid_Phi = reshape(Phi_query, M, N, P);
        Grid_Feasible = reshape(inRange, M, N, P);
        Grid_Util = reshape(Max_Utilization, M, N, P);
        
        % --- 反向蛇形处理 (Un-snaking) ---
        
        % Step A: 修复 X 方向 (偶数列反向)
        for j = 1:N
            if mod(j, 2) == 0
                Grid_X(:, j, :) = flipud(Grid_X(:, j, :));
                Grid_Y(:, j, :) = flipud(Grid_Y(:, j, :)); % 虽为常数，为了逻辑统一也翻转
                Grid_Phi(:, j, :) = flipud(Grid_Phi(:, j, :));
                Grid_Feasible(:, j, :) = flipud(Grid_Feasible(:, j, :));
                Grid_Util(:, j, :) = flipud(Grid_Util(:, j, :));
            end
        end
        
        % Step B: 修复 Y 方向 (偶数层反向)
        for i = 1:P
            if mod(i, 2) == 0
                Grid_X(:, :, i) = fliplr(Grid_X(:, :, i));
                Grid_Y(:, :, i) = fliplr(Grid_Y(:, :, i));
                Grid_Phi(:, :, i) = fliplr(Grid_Phi(:, :, i));
                Grid_Feasible(:, :, i) = fliplr(Grid_Feasible(:, :, i));
                Grid_Util(:, :, i) = fliplr(Grid_Util(:, :, i));
            end
        end
        
        % 维度置换 -> [Y, X, Phi] 以匹配 plot 坐标系
        p_order = [2 1 3];
        Final_X = permute(Grid_X, p_order) * 1000; % mm
        Final_Y = permute(Grid_Y, p_order) * 1000; % mm
        Final_Phi = permute(Grid_Phi, p_order);
        Final_Feasible = permute(Grid_Feasible, p_order);
        Final_Util = permute(Grid_Util, p_order);

        %% === 诊断模块 (Diagnostic Block) ===
        % 目的：查明为什么 Y=20mm 的点被认为是可行的
        
        fprintf('\n--- 诊断报告: Y 方向边界检查 ---\n');
        % 查找一个位于 Y > 15mm 且被标记为 Feasible 的点
        suspicious_mask = (Final_Y > 15) & Final_Feasible;
        
        if any(suspicious_mask(:))
            % 获取第一个异常点的线性索引
            idx_linear = find(suspicious_mask, 1);
            
            % 找到它在原始 qData 中的位置 (需要反推还是直接用坐标匹配？)
            % 为了简单，我们用坐标反查最近的 qData 索引
            target_x = Final_X(idx_linear);
            target_y = Final_Y(idx_linear);
            target_phi = Final_Phi(idx_linear);
            
            % 在原始输入中找到这个点 (避免 reshape 混乱)
            orig_idx = find(abs(X_query*1000 - target_x) < 0.01 & ...
                            abs(Y_query*1000 - target_y) < 0.01 & ...
                            abs(Phi_query - target_phi) < 0.01, 1);
            
            if ~isempty(orig_idx)
                q_vals = qData(orig_idx, :);
                fprintf('[发现] 在 Pose Y = %.1f mm 处有一个可行点。\n', target_y);
                fprintf('该点的驱动器输入值 (mm):\n');
                fprintf('  Driver 1: X=%.2f, Y=%.2f, Z=%.2f\n', q_vals(1)*1000, q_vals(2)*1000, q_vals(3)*1000);
                fprintf('  Driver 2: X=%.2f, Y=%.2f, Z=%.2f\n', q_vals(4)*1000, q_vals(5)*1000, q_vals(6)*1000);
                fprintf('判断逻辑:\n');
                fprintf('  Limit Y = +/- %.2f mm\n', actuatorLimits.y(2)*1000);
                if max(abs([q_vals(2), q_vals(5)])) <= actuatorLimits.y(2)
                    fprintf('  [结论] 驱动器数值 %.2f < Limit，因此判定为【可行】。\n', max(abs([q_vals(2), q_vals(5)]))*1000);
                    fprintf('  (这是物理现象：末端动20mm，驱动器只动了不到10mm)\n');
                else
                    fprintf('  [结论] 驱动器数值超标，但未被剔除？请检查 inRange 逻辑。\n');
                end
            end
        else
            fprintf('[正常] 没有发现 Y > 15mm 的可行点。之前的散点可能是坐标映射 Bug 导致的。\n');
        end
        fprintf('------------------------------------\n\n');

        %% 8. 边界提取与绘图
        
        % 提取边界 Mask
        if exist('bwperim', 'file')
            Boundary_Mask = bwperim(Final_Feasible, 26);
        else
            Boundary_Mask = Final_Feasible & ~imerode(Final_Feasible, ones(3,3,3));
        end
        
        % 仅提取边界数据
        Scatter_X = Final_X(Boundary_Mask);
        Scatter_Y = Final_Y(Boundary_Mask);
        Scatter_Phi = Final_Phi(Boundary_Mask);
        Scatter_Util = Final_Util(Boundary_Mask);
        
        is_crit = Scatter_Util >= utilization_threshold;
        is_safe = Scatter_Util < utilization_threshold;
        
        figure('Name', '4-DOF Workspace Analysis', 'Color', 'w'); hold on;
        
        % A. 绘制体积轮廓 (极淡背景)
        Smooth_Data = smooth3(double(Final_Feasible), 'box', 5);
        [F, V] = isosurface(Final_X, Final_Y, Final_Phi, Smooth_Data, 0.5);
        if ~isempty(F)
            patch('Faces', F, 'Vertices', V, 'FaceColor', [0.95 0.95 0.95], ...
                  'EdgeColor', 'none', 'FaceAlpha', 0.1);
        end
        
        % B. 绘制边界散点
        h_safe = []; h_crit = [];
        
        if any(is_safe)
            h_safe = scatter3(Scatter_X(is_safe), Scatter_Y(is_safe), Scatter_Phi(is_safe), ...
                15, [0 0.7 0], 'filled', 'MarkerFaceAlpha', 0);
        end
        
        if any(is_crit)
            h_crit = scatter3(Scatter_X(is_crit), Scatter_Y(is_crit), Scatter_Phi(is_crit), ...
                5, [1 0 0], 'filled', 'MarkerFaceAlpha', 0);
        end
        
        % --- 图像设置 ---
        grid on; box on; axis equal;
        view(3); camlight; lighting gouraud;
        
        title(sprintf('Workspace Boundary (Phi: %d to %d deg)', min(range_Phi_deg), max(range_Phi_deg)));
        xlabel('X Position [mm]');
        ylabel('Y Position [mm]');
        zlabel('Phi Angle [deg]');
        
        xlim([min(range_X_mm)-2, max(range_X_mm)+2]);
        ylim([min(range_Y_mm)-2, max(range_Y_mm)+2]);
        
        % 动态生成图例
        legend_handles = []; legend_names = {};
        if ~isempty(h_safe), legend_handles(end+1)=h_safe; legend_names{end+1}='Safe (<90%)'; end
        if ~isempty(h_crit), legend_handles(end+1)=h_crit; legend_names{end+1}='Critical (>90%)'; end
        if ~isempty(legend_handles), legend(legend_handles, legend_names, 'Location', 'best'); end
        
    catch ME
        fprintf('错误: %s\n', ME.message);
        fprintf('Line: %d\n', ME.stack(1).line);
    end
end