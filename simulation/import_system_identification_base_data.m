%% ������ӱ���е�����
% ���ڴ����µ��ӱ�������ݵĽű�:
%
%    ������: G:\����\Car-double-pendulum system\simulation\base_data.xlsx
%    ������: Sheet1
%
% Ҫ��չ�����Թ�����ѡ�����ݻ��������ӱ��ʹ�ã������ɺ���������ű���

% �� MATLAB �Զ������� 2019/02/22 16:34:15

%% ��������
[~, ~, raw] = xlsread('G:\����\Car-double-pendulum system\simulation\base_data.xlsx','Sheet1');
raw = raw(:,11:end);

%% �����������
data = reshape([raw{:}],size(raw));

%% ����������������б�������
pitch_vel = data(:,1);
pitch = data(:,2);

%% �����ʱ����
clearvars data raw;