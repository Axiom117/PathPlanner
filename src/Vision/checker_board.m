% ==============================
% 1. 参数设置
innerCorners = [9, 6];        % 内角点数 [列, 行]
squareSizeMM   = 5;          % 棋盘格方格边长，单位：毫米（打印时可按此比例换算）
dpi            = 300;         % 打印分辨率，单位：dots per inch
pxPerMM        = dpi / 25.4;  % 每毫米对应的像素数

% 2. 计算生成图像尺寸
numSquares = innerCorners + [1, 1];            % 棋盘格方格总数 = 内角点数 + 1
squareSizePx = round(squareSizeMM * pxPerMM);  % 每个方格在图像中的像素尺寸
imgSize       = numSquares .* squareSizePx;    % 图像整体尺寸 [高, 宽]

% 3. 生成棋盘格矩阵
%    MATLAB 自带的 checkerboard 函数生成的图案是灰度渐变，这里手动构造二值矩阵
board = false(imgSize);  % 先全部置黑
for row = 1:numSquares(2)
    for col = 1:numSquares(1)
        if mod(row+col, 2)==0
            r1 = (row-1)*squareSizePx + (1:squareSizePx);
            c1 = (col-1)*squareSizePx + (1:squareSizePx);
            board(r1, c1) = true;
        end
    end
end

% 4. 转换为图像并保存
img = uint8(board) * 255;               % 转为 0-255 范围的灰度图
imwrite(img, 'checkerboard_9x6.png');   % 保存为 PNG

% 5. 显示结果（可选）
figure; imshow(img);
axis on; axis image;
title('9×6 内角点棋盘格标定板');
xlabel(sprintf('每格 %d px ≈ %.1f mm at %d DPI', squareSizePx, squareSizePx/pxPerMM, dpi));
% ==============================
