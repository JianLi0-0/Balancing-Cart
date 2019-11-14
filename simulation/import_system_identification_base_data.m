%% 导入电子表格中的数据
% 用于从以下电子表格导入数据的脚本:
%
%    工作簿: G:\程序\Car-double-pendulum system\simulation\base_data.xlsx
%    工作表: Sheet1
%
% 要扩展代码以供其他选定数据或其他电子表格使用，请生成函数来代替脚本。

% 由 MATLAB 自动生成于 2019/02/22 16:34:15

%% 导入数据
[~, ~, raw] = xlsread('G:\程序\Car-double-pendulum system\simulation\base_data.xlsx','Sheet1');
raw = raw(:,11:end);

%% 创建输出变量
data = reshape([raw{:}],size(raw));

%% 将导入的数组分配给列变量名称
pitch_vel = data(:,1);
pitch = data(:,2);

%% 清除临时变量
clearvars data raw;