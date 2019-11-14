clc
clear

% x =[x theta x_dot theta_dot]

g = 9.8;
Km = 0.3*187/3591; %转矩常数 N*m/A
R = 0.095/2;%轮子半径 m
bw = 0;
Mw = 0.134;
Mb = 0.790;
Iw = 0.000020;
Ib = 0.000311;
L = 0.005548;
T = 0.005;%控制周期

Cwb = 0.000098;%电机阻尼系数
Cf = 00.00;%轮子地面摩擦阻尼系数

% syms g Km R bw
% syms Mw Mb
% syms Iw Ib L
% syms T
syms i X_ddot Theta_ddot Xdot Theta

% eqn1 = (2*Mw + 2*Iw/(R^2) +Mb)*X_ddot + Mb*L*Theta_ddot + 2*bw/(R^2)*Xdot == 2*T/R;
% eqn2 =  Mb*L*X_ddot + (Mb*L^2 + Ib)*Theta_ddot - Mb*g*L*Theta == -2*T;
% 
% [X_ddot,Theta_ddot]=solve(eqn1,eqn2,X_ddot,Theta_ddot);
% 
% display([X_ddot;Theta_ddot]);

E = [2*Mw+2*Iw/(R^2)+Mb Mb*L;
     Mb*L               Mb*L^2+Ib];
 
F = [2*Cwb/(R^2)         -2*Cwb/R;
     -2*Cwb/R            2*Cwb];
 
G = [0                     0;
     0                     -Mb*g*L];

H = [ 2/R*Km;
      -2*Km ];
% X_ddot
% (
% -(g*L^2*Mb^2*R^2)*Theta 
% -(2*bw*L^2*Mb + 2*Ib*bw)*Xdot
% +(2*L^2*Mb*R^2 + 2*L*Mb*R^2 + 2*Ib*R^2)*Km*i 
% )
% /(R^2*(2*Ib*Iw + Ib*Mb + 2*Ib*Mw + 2*Iw*L^2*Mb + 2*L^2*Mb*Mw))

% Theta_ddot
% (
% + (L*Mb^2*R^2*g + 2*Iw*L*Mb*R^2*g + 2*L*Mb*Mw*R^2*g)*Theta
% + 2*L*Mb*bw*Xdot
% - (4*Iw*R^2 + 2*Mb*R^2 + 4*Mw*R^2 + 2*L*Mb*R^2)*Km*i 
% )
% /(R^2*(2*Ib*Iw + Ib*Mb + 2*Ib*Mw + 2*Iw*L^2*Mb + 2*L^2*Mb*Mw))



% A = [0 0 1 0;
%      0 0 0 1;
%      0 -(g*L^2*Mb^2*R^2)/(R^2*(2*Ib*Iw + Ib*Mb + 2*Ib*Mw + 2*Iw*L^2*Mb + 2*L^2*Mb*Mw))                                          -(2*bw*L^2*Mb + 2*Ib*bw)/(R^2*(2*Ib*Iw + Ib*Mb + 2*Ib*Mw + 2*Iw*L^2*Mb + 2*L^2*Mb*Mw))     0;
%      0 (L*Mb^2*R^2*g + 2*Iw*L*Mb*R^2*g + 2*L*Mb*Mw*R^2*g)/(R^2*(2*Ib*Iw + Ib*Mb + 2*Ib*Mw + 2*Iw*L^2*Mb + 2*L^2*Mb*Mw))          (2*L*Mb*bw)/(R^2*(2*Ib*Iw + Ib*Mb + 2*Ib*Mw + 2*Iw*L^2*Mb + 2*L^2*Mb*Mw))                 0
%     ];

A = [0 0       1 0;
     0 0       0 1;
     -inv(E)*G -inv(E)*F];

% B = [0 0 (2*L^2*Mb*R^2 + 2*L*Mb*R^2 + 2*Ib*R^2)*Km/(R^2*(2*Ib*Iw + Ib*Mb + 2*Ib*Mw + 2*Iw*L^2*Mb + 2*L^2*Mb*Mw)) (2*L^2*Mb*R^2 + 2*L*Mb*R^2 + 2*Ib*R^2)*Km/(R^2*(2*Ib*Iw + Ib*Mb + 2*Ib*Mw + 2*Iw*L^2*Mb + 2*L^2*Mb*Mw))]';
B = [0;0;inv(E)*H];
C = eye(4);
D = zeros(4,1);

Co=ctrb(A,B);
[~,ran]=size(A);
if rank(Co)==ran
    'controlable'
else
    'uncontrolable'
end

%pole placement ,state feedback
% p = [-0.1 -2 -1+1i -1-1i];
% k = place(A,B,p);

%discrete linear-quadratic regulator

% x =[x theta x_dot theta_dot]
R = 3;%控制量加权，积分后代表能量功率
Q = [20  0  0  0;
     0  100  0  0;
     0  0  1  0;
     0  0  0  4]; %误差加权，越重要的状态变量，权值越大，penalty

%good parameter for angle
% % % R = 3;%控制量加权，积分后代表能量功率
% % % Q = [20  0  0  0;
% % %      0  100  0  0;
% % %      0  0  2  0;
% % %      0  0  0  4]; %误差加权，越重要的状态变量，权值越大，penalty


k = lqrd(A,B,Q,R,T);
display(k);
k0 = k(1);
k1 = k(2);
k2 = k(3);
k3 = k(4);
fprintf('%f , %f , %f , %f \n',k0,k1,k2,k3);
period = 0.005;

Ac=A - B*k;  

x0 = [0;10/180*pi;0;0]; %initial state
t = 0:0.005:10;
u = zeros(size(t));
[y,x]=lsim(Ac,B,C,D,u,t,x0);
plot(t,y(:,2));
ylabel('rod angle rad');
xlabel('T s');

% C=[0 0 1 0];
% Ob = obsv(A,C)
% rank(Ob)
% det(obsv(A,C))


