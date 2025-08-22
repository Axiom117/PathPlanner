function pose_estimation()
% Real-time ArUco pose estimation and visualization (manual trigger + getdata)
% Prerequisite: cameraParams.mat containing cameraParams.Intrinsics

%% ---- Camera Initialization ----
imaqreset
vid = videoinput('gentl', 1);           % Basler GenTL adapter
src = getselectedsource(vid);           % GenTL video source (Basler feature map)

% Use grayscale output to accelerate image processing
if isprop(vid,'ReturnedColorSpace'), vid.ReturnedColorSpace = 'grayscale'; end

% Enable auto exposure/gain for initial brightness stabilization
if isprop(src,'ExposureAuto'), src.ExposureAuto = 'Continuous'; end
if isprop(src,'GainAuto'),     src.GainAuto     = 'Continuous'; end

% Configure manual trigger to minimize latency
vid.LoggingMode       = 'memory';   % low-latency logging
vid.FramesPerTrigger  = 1;          % exactly one frame per trigger
vid.TriggerRepeat     = Inf;        % keep triggering until stopped
triggerconfig(vid, 'manual');
vid.Timeout = 1.0;                  % seconds, for getdata timeout

% Start the acquisition engine
start(vid);

% Warm up a few triggers so exposure/gain can settle
for i = 1:5
    trigger(vid);
    try
        Iraw = getdata(vid, 1, 'uint8'); %#ok<NASGU>
    catch
        % If timeout occurs during warm-up, just continue
    end
end

% Lock exposure/gain to avoid brightness drift during tracking (optional but recommended)
if isprop(src,'ExposureAuto'), src.ExposureAuto = 'Off'; end
if isprop(src,'GainAuto'),     src.GainAuto     = 'Off'; end

%% ---- Calibration Load ----
S = load('cameraParams.mat');              % Must contain variable: cameraParams
if isfield(S,'cameraParams')
    intrinsics = S.cameraParams.Intrinsics;
else
    error('cameraParams.mat does not contain variable "cameraParams".');
end

%% ---- ArUco Settings ----
markerFamily = "DICT_6X6_250";
markerSizeMM = 8.0;              % Marker edge length (mm)

% Axis overlay length (mm) in the marker coordinate frame
axisLen = 15;
worldPts = [0 0 0; axisLen 0 0; 0 axisLen 0; 0 0 axisLen];

% Grab one fresh frame for initializing the UI
trigger(vid);
I = getdata(vid, 1, 'uint8');

% Create figure and image handle once (avoid re-imshow to prevent flicker)
hFig = figure('Name','ArUco Live','NumberTitle','off');
hIm = imshow(I,'Border','tight'); hold on
htitle = title('Detecting...');

% Preallocate overlay handles to reuse line/text objects across frames
maxMarkers = 16;        % Increase/decrease as needed
hX = gobjects(maxMarkers,1);
hY = gobjects(maxMarkers,1);
hZ = gobjects(maxMarkers,1);
hTxt = gobjects(maxMarkers,1);

frameCount = 0; t0 = tic;

%% ---- Main Loop (runs until the figure is closed) ----
while ishghandle(hFig)
    % Software trigger one frame and fetch it immediately
    trigger(vid);
    try
        Iraw = getdata(vid, 1, 'uint8');   % returns the just-triggered frame
    catch
        % In case of timeout or transient error, skip this iteration
        drawnow limitrate
        continue
    end

    % Optional undistortion:
    % readArucoMarker already accounts for intrinsics/distortion internally.
    % If lens distortion is strong and you need extra precision, uncomment:
    % Iu = undistortImage(Iraw, intrinsics, 'OutputView','same');
    Iu = Iraw;

    % Detection + pose estimation
    [ids, locs, poses, detectedFamily, rej] = readArucoMarker( ...
        Iu, markerFamily, intrinsics, markerSizeMM, ...
        RefineCorners=true, NumBorderBits=1, WindowSizeRange=[3 51], WindowSizeStep=6); %#ok<ASGLU>

    % Update the background image pixels without recreating the image object
    set(hIm, 'CData', Iu);

    % Hide previous overlays for this frame; will re-show active ones
    for k = 1:maxMarkers
        if isgraphics(hX(k)),  set(hX(k), 'Visible','off');  end
        if isgraphics(hY(k)),  set(hY(k), 'Visible','off');  end
        if isgraphics(hZ(k)),  set(hZ(k), 'Visible','off');  end
        if isgraphics(hTxt(k)), set(hTxt(k),'Visible','off'); end
    end

    % Draw axes for each detected marker
    if ~isempty(ids)
        n = min(numel(ids), maxMarkers);
        for k = 1:n
            % Project 3D axis endpoints (marker frame, mm) into image pixels
            imgPts = worldToImage(intrinsics, poses(k).Rotation, poses(k).Translation, worldPts);

            % Three axis lines: O->X, O->Y, O->Z
            [hX(k), hY(k), hZ(k)] = ensureAxisLines(hX(k), hY(k), hZ(k));
            set(hX(k),'XData',[imgPts(1,1) imgPts(2,1)],'YData',[imgPts(1,2) imgPts(2,2)],'LineWidth',4,'Color',[1 0 0],'Visible','on');
            set(hY(k),'XData',[imgPts(1,1) imgPts(3,1)],'YData',[imgPts(1,2) imgPts(3,2)],'LineWidth',4,'Color',[0 1 0],'Visible','on');
            set(hZ(k),'XData',[imgPts(1,1) imgPts(4,1)],'YData',[imgPts(1,2) imgPts(4,2)],'LineWidth',4,'Color',[0 0 1],'Visible','on');

            % Text label (marker ID + translation)
            cxy  = mean(locs(:,:,k),1);          % Marker centroid in pixels
            t_mm = poses(k).Translation;         % Translation in mm (units tied to markerSizeMM)
            hTxt(k) = ensureText(hTxt(k), cxy(1), cxy(2), ...
                sprintf('ID=%d  t=[%.1f %.1f %.1f] mm', ids(k), t_mm(1), t_mm(2), t_mm(3)));
        end
        set(htitle,'String',sprintf('Detected %d marker(s)', numel(ids)));
    else
        set(htitle,'String','No ArUco detected');
    end

    % FPS display (update every 10 frames)
    frameCount = frameCount + 1;
    if mod(frameCount, 10) == 0
        fps = frameCount / toc(t0);
        set(htitle,'String',sprintf('%s | FPS: %.1f', get(htitle,'String'), fps));
    end

    drawnow limitrate   % Smooth UI refresh without overloading the event loop
end

% Cleanup
try stop(vid); catch, end
try delete(vid); catch, end
imaqreset

end

% ---------- Helpers: ensure line/text handles exist ----------
function [hx,hy,hz] = ensureAxisLines(hx,hy,hz)
    if ~isgraphics(hx), hx = line(nan,nan); end
    if ~isgraphics(hy), hy = line(nan,nan); end
    if ~isgraphics(hz), hz = line(nan,nan); end
end

function ht = ensureText(ht, x, y, str)
    if ~isgraphics(ht)
        ht = text(x, y, str, 'Color','y','FontSize',12,'FontWeight','bold','HorizontalAlignment','center');
    else
        set(ht,'Position',[x y 0],'String',str,'Visible','on');
    end
end
