%% analyze_workspace_2d_projections.m - 4自由度机构 2D 投影工作空间分析
% 说明：分别计算并绘制 X-Phi, Y-Phi, Z-Phi 的可达空间平面图
% 颜色规定：X(红), Y(绿), Z(蓝)
% 修改记录：横轴改为位移，纵轴改为Phi；强制开启图表工具栏

function analyze_workspace_2d_projections()
    clc; close all;
    
    % --- 1. 全局配置与参数 ---
    modelName = 'model_2R_RCM_IK'; 
    
    % 驱动器物理极限 [米] (Input: 20x20x30 mm box -> +/-10, +/-10, +/-15)
    limits.x = 10e-3; 
    limits.y = 10e-3;
    limits.z = 15e-3;
    
    % 扫描范围设置 (为了覆盖整个空间，范围设得稍微大一点，让程序自己算出边界)
    % 空间位移范围 (mm)
    scan_range_mm = -35:1:35; 
    % 角度偏转范围 (deg)
    scan_phi_deg = -50:1:50;   
    
    fprintf('=== 开始 4-DOF 机构 2D 可达空间分析 ===\n');

    % 创建图形窗口
    % 修正：添加 'ToolBar' 和 'MenuBar' 属性以启用编辑工具
    fig = figure('Name', '4-DOF Workspace 2D Projections', ...
                 'Color', 'w', ...
                 'Position', [100, 100, 1200, 400], ...
                 'ToolBar', 'figure', ...
                 'MenuBar', 'figure');
                 
    t = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    
    %% --- 2. 分析 X 方向 (X vs Phi) ---
    fprintf('正在分析 X 方向可达性...\n');
    % 构造输入: 变化 X 和 Phi, 固定 Y=0, Z=0
    % 这里的 1 代表正在扫描的是 pose 的第 1 个分量 (x)
    [valid_Pos_X, valid_Phi_X] = run_sweep_scan(1, scan_range_mm, scan_phi_deg, modelName, limits);
    
    % 绘图 (红色) - 修正：Swap X/Y axes -> scatter(Pos, Phi)
    nexttile;
    if ~isempty(valid_Pos_X)
        scatter(valid_Pos_X, valid_Phi_X, 15, 'MarkerEdgeColor', 'none', ...
                'MarkerFaceColor', 'r', 'MarkerFaceAlpha', 0.5);
    end
    setup_plot('X-Axis Reachability', 'X Position [mm]', 'Phi Angle [deg]');

    %% --- 3. 分析 Y 方向 (Y vs Phi) ---
    fprintf('正在分析 Y 方向可达性...\n');
    % 构造输入: 变化 Y 和 Phi, 固定 X=0, Z=0
    % 这里的 2 代表正在扫描的是 pose 的第 2 个分量 (y)
    [valid_Pos_Y, valid_Phi_Y] = run_sweep_scan(2, scan_range_mm, scan_phi_deg, modelName, limits);
    
    % 绘图 (绿色) - 修正：Swap X/Y axes -> scatter(Pos, Phi)
    nexttile;
    if ~isempty(valid_Pos_Y)
        scatter(valid_Pos_Y, valid_Phi_Y, 15, 'MarkerEdgeColor', 'none', ...
                'MarkerFaceColor', 'g', 'MarkerFaceAlpha', 0.5);
    end
    setup_plot('Y-Axis Reachability', 'Y Position [mm]', 'Phi Angle [deg]');

    %% --- 4. 分析 Z 方向 (Z vs Phi) ---
    fprintf('正在分析 Z 方向可达性...\n');
    % 构造输入: 变化 Z 和 Phi, 固定 X=0, Y=0
    % 这里的 3 代表正在扫描的是 pose 的第 3 个分量 (z)
    [valid_Pos_Z, valid_Phi_Z] = run_sweep_scan(3, scan_range_mm, scan_phi_deg, modelName, limits);
    
    % 绘图 (蓝色) - 修正：Swap X/Y axes -> scatter(Pos, Phi)
    nexttile;
    if ~isempty(valid_Pos_Z)
        scatter(valid_Pos_Z, valid_Phi_Z, 15, 'MarkerEdgeColor', 'none', ...
                'MarkerFaceColor', 'b', 'MarkerFaceAlpha', 0.5);
    end
    setup_plot('Z-Axis Reachability', 'Z Position [mm]', 'Phi Angle [deg]');

    fprintf('=== 分析完成 ===\n');
end

%% --- 辅助函数 1: 扫描与解算核心逻辑 ---
function [valid_Pos, valid_Phi] = run_sweep_scan(axis_idx, range_pos_mm, range_phi_deg, modelName, limits)
    % axis_idx: 1=X, 2=Y, 3=Z
    
    % 1. 生成网格
    [Grid_Pos, Grid_Phi] = meshgrid(range_pos_mm, range_phi_deg);
    numPoints = numel(Grid_Pos);
    
    % 2. 初始化 Pose 矩阵 (N x 6) -> [x, y, z, Rx, Ry, Rz]
    % 默认为 0
    poses = zeros(numPoints, 6);
    
    % 填充 Phi (第4列，假设 Phi 对应 Rx, Ry 或 Rz，根据你的描述应该是绕Z或特定的 Phi)
    % 注意：根据你的描述 Phi 是第4个分量 (Rx)，如果你的定义不同请在此修改
    % 通常 robot plant 输入: [x, y, z, Rx, Ry, Rz]
    poses(:, 4) = Grid_Phi(:); 
    
    % 填充主扫描轴 (X, Y 或 Z)
    % 将 mm 转换为 m
    poses(:, axis_idx) = Grid_Pos(:) * 1e-3; 
    
    % 3. 构造 trajectoryData
    dt = 0.001;
    trajectoryData.poses = poses;
    trajectoryData.time = (0:dt:(numPoints-1)*dt)';
    trajectoryData.dt = dt;
    
    % 4. 调用 IK 解算器
    % 使用 try-catch 避免个别奇异点导致整个脚本崩溃
    try
        [~, qData, ~] = solverIK(trajectoryData, modelName);
    catch
        warning('Solver IK failed for axis %d. Returning empty.', axis_idx);
        valid_Pos = []; valid_Phi = [];
        return;
    end
    
    % 5. 极限检查 (Limit Check)
    % qData 结构假设: [x1, y1, z1, x2, y2, z2]
    % 限制向量: [Lx, Ly, Lz, Lx, Ly, Lz]
    L_vec = [limits.x, limits.y, limits.z];
    L_full = [L_vec, L_vec]; 
    
    % 扩展限制矩阵以匹配数据大小
    L_matrix = repmat(L_full, numPoints, 1);
    
    % 计算利用率 (Abs(Value) / Limit)
    utilization = abs(qData) ./ L_matrix;
    
    % 找出所有驱动器都在限制范围内的点 (max utilization <= 1.0)
    is_feasible = max(utilization, [], 2) <= 1.0;
    
    % 6. 提取有效数据用于绘图
    valid_Pos = Grid_Pos(is_feasible); % 保持 mm 单位用于绘图
    valid_Phi = Grid_Phi(is_feasible); % deg
end

%% --- 辅助函数 2: 绘图设置 ---
function setup_plot(title_str, x_label_str, y_label_str)
    grid on; box on;
    title(title_str, 'FontSize', 12, 'FontWeight', 'bold');
    xlabel(x_label_str);
    ylabel(y_label_str);
    axis tight;
    % 添加十字中心线辅助观察
    xline(0, '--k', 'Alpha', 0.3);
    yline(0, '--k', 'Alpha', 0.3);
    set(gca, 'FontSize', 10);
end