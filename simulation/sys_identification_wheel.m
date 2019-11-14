clear
%导入数据
[~, ~, raw] = xlsread('G:\程序\Car-double-pendulum system\simulation\base_data.xlsx','Sheet1');
raw = raw(:,20:21);
data = reshape([raw{:}],size(raw));
delta_angle = data(:,1);
current = data(:,2);
clearvars data raw;

syms Iw Cfw 
T = 0.002;
Km = 0.3*187/3591; %转矩常数 N*m/A

current = current/16384.0*20.0;
delta_angle = delta_angle/8191.0*360.0;
wheel_angular_speed = delta_angle/180.0*pi/T;

i = 0;
for i = 2:1:2500
    if abs(wheel_angular_speed(i)-wheel_angular_speed(i-1)) > 15
        wheel_angular_speed(i) = wheel_angular_speed(i-1);
    end
end

y = wheel_angular_speed(2:end);
x=[wheel_angular_speed(1:end-1) current(1:end-1)];
theta = inv(x'*x)*x'*y;

fprintf('θ1 =  ');
fprintf('%f \n', theta(1));
fprintf('θ2 =  ');
fprintf('%f \n',theta(2));

%绘图比较
ww_predict = x*theta;

ylabel('angular speed of wheel  rad/s');
xlabel('time  s');
plot(0.002:0.002:0.002*2499,wheel_angular_speed(2:end),'o' , 'MarkerSize' ,1);
hold on
plot(0.002:0.002:0.002*2499,ww_predict);
legend('Training data of wheel_angular_speed', 'Prediction of wheel_angular_speed')

hold off

equation1 = (Iw - Cfw*T)/Iw == theta(1);
equation2 = Km*T/Iw == theta(2);
[Iw,Cfw] = solve(equation1,equation2,Iw,Cfw);
fprintf('Iw =  ');
fprintf('%f \n', Iw);
fprintf('Cfw =  ');
fprintf('%f \n',Cfw);

