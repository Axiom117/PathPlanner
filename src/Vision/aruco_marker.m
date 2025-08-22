% Params
fam   = "DICT_6X6_250";   % 与你后续检测用的字典保持一致
ids   = 1:20;             % 需要的 ID
px    = 600;              % 导出像素边长（建议 >=400）
bBits = 1;                % 边框比特数（默认 1）

outdir = "markers_6x6_250";
if ~exist(outdir,'dir'), mkdir(outdir); end

for id = ids
    I = generateArucoMarker(fam, id, px, NumBorderBits=bBits);
    fn = fullfile(outdir, sprintf('aruco_%s_id%03d_%dpx_b%d.png', fam, id, px, bBits));
    imwrite(I, fn, 'png');
end

disp("Done: PNGs saved to folder " + outdir);
