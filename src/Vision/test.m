% 检查可用的适配器
imaqhwinfo

% 连接GenICam兼容的相机
vid = videoinput('winvideo', 1);

% 配置相机参数
src = getselectedsource(vid);
src.Gain = 1.0;

% 预览图像
preview(vid);

% 获取单帧图像
img = getsnapshot(vid);
imshow(img);

% 清理
delete(vid);