% clear all;clc;close all;
% % 轨道参数
% omega_0=0.00075563;%卫星轨道半径
% h=2500;%轨道高度
% Re=6371;%地球半径
% R=h+Re;%卫星的轨道角速度
% %地理参数
% lambda_g=115*pi/180;%地心经度
% phi_g=40*pi/180;%地心纬度
% omega_g=0.00007292;%地球自转角速度
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
% legend('双曲正切函数','符号函数');
% grid on
% axis([-10 10 -1 1]);