% clear all;clc;close all;
% % �������
% omega_0=0.00075563;%���ǹ���뾶
% h=2500;%����߶�
% Re=6371;%����뾶
% R=h+Re;%���ǵĹ�����ٶ�
% %�������
% lambda_g=115*pi/180;%���ľ���
% phi_g=40*pi/180;%����γ��
% omega_g=0.00007292;%������ת���ٶ�
% omega_c=omega_g-omega_0;
% t=[0:0.01:200];
% for i=1:20001
%     y(i)=asin((-Re*cos(phi_g)*cos(omega_c*t(i)+lambda_g)+R)/sqrt(Re*Re-2*Re*R*cos(phi_g)*cos(omega_c*t(i)+lambda_g)+R*R))*180/pi;
%     z(i)=atan(-tan(phi_g)/sin(omega_c*t(i)+lambda_g))*180/pi;
% end
y=qd(:,4);
z=qd(:,5);
figure
plot(t,y,'b-')
grid on
figure
plot(t,z,'r-')
grid on

% x=[-100:0.01:100];
% for i=1:20001
% %     y(i)=-tanh(x(i));
%     y(i)=tanh(x(i));
%     if(x(i)>=0)
%         z(i)=1;
%     else
%         z(i)=-1;
%     end
% end
% figure
% plot(x,y,'r--')
% hold on
% plot(x,z,'b-')
% legend('˫�����к���','���ź���');
% grid on
% axis([-10 10 -1 1]);