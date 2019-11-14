clear
%导入数据
[~, ~, raw] = xlsread('G:\程序\Car-double-pendulum system\simulation\rod1.xlsx','Sheet1');
% raw = raw(1:1000,11:12);
raw = raw(1:1000,4:5);
% raw = raw(1:1000,36:37);
data = reshape([raw{:}],size(raw));
pitch_vel = data(:,1);
pitch = data(:,2);
clearvars data raw;


%[0.994309512530195;-0.148050494265480]
g = 9.8; %m/s^2
Cfw =  0.000098;
T = 0.005; %数据采样周期
Mb = 0.821; %body质量 kg
syms Ib L;



i = 0;
for i = 1:1:1000
    if pitch(i)<0
        pitch(i) = pitch(i)+2*180.0;
    end
end
pitch = pitch-180.0;
pitch = pitch/180.0*pi;
pitch = -pitch;
pitch_vel = pitch_vel/180.0*pi;

%最小二乘法拟合
y = pitch_vel(2:end);
x=[pitch_vel(1:end-1) sin(pitch(1:end-1))];
theta = inv(x'*x)*x'*y;
fprintf('θ1 =  ');
fprintf('%f \n', theta(1));
fprintf('θ2 =  ');
fprintf('%f \n',theta(2));

%绘图比较
pitch_vel_predict = x*theta;
plot(pitch_vel , sin(pitch) , 'o' , 'MarkerSize' ,5);
hold on
xlabel('angular speed  rad/s');
ylabel('pendulum angle  rad/s');
plot(pitch_vel_predict,sin(pitch(2:end)));
legend('Training data of pitch_vel', 'Prediction of pitch_vel')
hold off

figure %plot with time
hold on

ylabel('speed of body angle  rad/s');
xlabel('time  s');
plot(0.005:0.005:0.005*999,pitch_vel(2:end),'o' , 'MarkerSize' ,3);
plot(0.005:0.005:0.005*999,pitch_vel_predict);
legend('Training data of pitch_vel', 'Prediction of pitch_vel')
hold off

equation1 = (Ib-2*T*Cfw)/Ib == theta(1);
equation2 = Mb*L*g*T/Ib == theta(2);
[Ib,L] = solve(equation1,equation2,Ib,L);
Ib = vpa(Ib);
L = vpa(L);
fprintf('Ib =  ');
fprintf('%f \n', Ib);
fprintf('L =  ');
fprintf('%f \n',L);



