%ȫ�ֱ���
clear all;clc;close all;
global mb mh mp m Sb Sh Sp rb_h rh_p Ib Ih Ip K epsilon delta lambda Cr f11 f12 f21 f22
global Re R omega_c phi_g lambda_g omega_0 omega_g omega_t Rt flag i OMEGA_T omega_T
flag=2;%flag=1ʱ���ٵ���Ŀ�꣬flag=2ʱ�����˶�Ŀ��
mb=1200;
mh=20;
mp=60;
m=mb+mh+mp;
Sb=[0;0;0];
Sh=[0;0;5];
Sp=[0;5;0];
rb_h=[0;0;1.0];
rh_p=[0;0;0.5];
Ib=[1200 -20 -35;
    -20 1500 48;
    -35 48 1350];
Ih=[80 0 0;
    0 80 0;
    0 0 20];
Ip=[110 0 0;
    0 110 0;
    0 0 40];
%Simulink���渳��ֵ
omega_0=0.00075563;%���ǹ�����ٶ�
% omega_0=0.005;
u_0=[0;0;0;0;0;0;0;0];
% alpha_0 = 34*pi/180;
if(flag==1)
    alpha_0 = -60*pi/180;
    beta_0 = -50*pi/180;%֧��ת�Ǻ���Ч�غ�ת�ǳ�ֵ�ֱ�-10�ȡ�10��
    attitude_angle_0=[2*pi/180;3*pi/180;-2*pi/180];%��ʼ��̬��
elseif(flag==2)
    alpha_0 = 10*pi/180;
    beta_0 = -10*pi/180;%֧��ת�Ǻ���Ч�غ�ת�ǳ�ֵ�ֱ�-10�ȡ�10��
    attitude_angle_0=[5*pi/180;3*pi/180;-5*pi/180];%��ʼ��̬��
end
% beta_0 = 65*pi/180;%֧��ת�Ǻ���Ч�غ�ת�ǳ�ֵ�ֱ�-10�ȡ�10��
%����������
K=0.2*eye(5);
% lambda=diag([0.2;0.2;0.2;0.4;0.4]);
% lambda=diag([0.2;0.2;0.2;2.5;2.5]);
if(flag==1)
    epsilon=diag([1;1.5;1.5;1;1]);
%     epsilon=diag([0.9;0.9;1;5;0.7]);
%     epsilon=diag([0.9;0.9;1;2;2]);
%     lambda=diag([0.025;0.015;0.02;2.5;0.8]);
    lambda=diag([0.03;0.03;0.03;0.3;0.4]);
%     lambda=diag([0.025;0.04;0.05;0.4;0.2]);
%     epsilon=diag([1;1;1;1;1]);
%     lambda=diag([0.025;0.04;0.02;0.1;0.1]);
elseif(flag==2)
    epsilon=diag([1;1;1;1;1]);
%     lambda=diag([0.03;0.08;0.015;0.1;0.1]);
    lambda=diag([0.03;0.03;0.015;0.1;0.1]);
end

Cr=[0       0  -omega_0  0  0;
    0       0  0         0  0;
    omega_0 0  0         0  0;
    0       0  0         0  0;
    0       0  0         0  0];
delta=1;
%Ħ�����ز���
f11=0.1;
f12=30;
f21=0.1; 
f22=30;
%�������ٹ��ɲ���
% % %�������ٽǶȳ�ֵ
% % alpha_d0=15*pi/180;%(15��)
% % beta_d0=-30*pi/180;%(-30��)
% %�������ٽǶ�ԲƵ
% alpha_d_omega=5*pi/180;
% beta_d_omega=10*pi/180;
% %�������������˶���ֵ
% alpha_d_am=30*pi/180;
% beta_d_am=45*pi/180;
% alpha_d_am*sin(alpha_d_omega*u(1))
% beta_d_am*sin(beta_d_omega*u(1))
% alpha_d_am*alpha_d_omega*cos(alpha_d_omega*u(1))
% beta_d_am*beta_d_omega*cos(beta_d_omega*u(1))
% alpha_d_am*alpha_d_omega^2*sin(alpha_d_omega*u(1))
% beta_d_am*beta_d_omega^2*sin(beta_d_omega*u(1))
% �������
h=2500*1000;%�������ǹ���߶�
Re=6371*1000;%����뾶
R=h+Re;%���ǵĹ�����ٶ�
%�������
% lambda_g=115*pi/180;%���ľ���
lambda_g=3*pi/180;%���ľ���
% lambda_g=-10*pi/180;%���ľ���
phi_g=10*pi/180;%����γ��
% phi_g=20*pi/180;%����γ��
omega_g=0.00007292;%������ת���ٶ�
omega_c=omega_g-omega_0;
%Ŀ��������
omega_t=0.0008243;%Ŀ�����ǹ�����ٶ�
ht=2000*1000;%Ŀ�����ǹ���߶�
Rt=Re+ht;
i=10*pi/180;%������
OMEGA_T=10*pi/180;%������ྶ
omega_T=0;%���ص����
