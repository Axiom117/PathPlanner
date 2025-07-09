function printFolderTree(folderPath, indent)
    % 如果没有指定缩进，则初始化为空
    if nargin < 2
        indent = '';
    end

    % 获取文件夹内容
    files = dir(folderPath);
    % 排除 . 和 ..
    files = files(~ismember({files.name}, {'.', '..'}));

    for i = 1:length(files)
        name = files(i).name;
        fullPath = fullfile(folderPath, name);

        % 打印当前文件或文件夹
        if files(i).isdir
            fprintf('%s📁 %s\n', indent, name); % 文件夹
            % 递归打印子文件夹
            printFolderTree(fullPath, [indent '    ']);
        else
            fprintf('%s📄 %s\n', indent, name); % 文件
        end
    end
end
