close all
global flag
%����ͼ��
%�������ٵ���̬�Ǽ�ʵ�ʸ��ٵ���̬��
figure(1);
subplot(1,2,1); 
plot(t,qd(:,1)*180/pi,'b-','linewidth',2);
hold on;
plot(t,qd(:,2)*180/pi,'r--','linewidth',2);
plot(t,qd(:,3)*180/pi,'k:','linewidth',2);
hold off;
legend('��','��','��');
xlabel('ʱ��( sec )');
ylabel('������̬��( �� )');
title('������̬��');
if(flag==1)
    axis([0 300 -4 4]);
elseif(flag==2)
    axis([0 300 -6 6]);
end
grid on
subplot(1,2,2);
plot(t,attitude_angle(:,1)*180/pi,'b-','linewidth',2);
hold on;
plot(t,attitude_angle(:,2)*180/pi,'r--','linewidth',2);
plot(t,attitude_angle(:,3)*180/pi,'k:','linewidth',2);
hold off;
legend('��','��','��');
xlabel('ʱ��( sec )');
ylabel('ʵ����̬��( �� )');
title('ʵ����̬��');
if(flag==1)
    axis([0 300 -4 4]);
elseif(flag==2)
    axis([0 300 -6 6]);
end
grid on
%ϵͳ���ٵ���̬�����
% figure(2);
% plot(t,(attitude_angle(:,1)-qd(:,1))*180/pi,'b-','linewidth',2);
% hold on;
% plot(t,(attitude_angle(:,2)-qd(:,2))*180/pi,'r--','linewidth',2);
% plot(t,(attitude_angle(:,3)-qd(:,3))*180/pi,'k:','linewidth',2);
% hold off;
% legend('����','����','����');
% xlabel('ʱ��( sec )');
% ylabel('��̬�����( �� )');
% title('��̬�����');
% axis([0 300 -6 12]);
% grid on
%�������ٵ���̬���ٶ���ʵ�ʸ��ٵ���̬���ٶ�
figure(3);
subplot(1,2,1);
plot(t,qd_dao(:,1)*180/pi,'b-','linewidth',2);
hold on;
plot(t,qd_dao(:,2)*180/pi,'r--','linewidth',2);
plot(t,qd_dao(:,3)*180/pi,'k:','linewidth',2);
hold off;
legend('d��/dt','d��/dt','d��/dt');
xlabel('ʱ��( sec )');
ylabel('������̬���ٶ�( ��/s )');
title('������̬���ٶ�');
if(flag==1)
    axis([0 300 -0.1 0.1]);
elseif(flag==2)
    axis([0 300 -0.2 0.2]);
end
grid on
subplot(1,2,2);
plot(t,attitude_angle_dao(:,1)*180/pi,'b-','linewidth',2);
hold on;
plot(t,attitude_angle_dao(:,2)*180/pi,'r--','linewidth',2);
plot(t,attitude_angle_dao(:,3)*180/pi,'k:','linewidth',2);
hold off;
legend('d��/dt','d��/dt','d��/dt');
xlabel('ʱ��( sec )');
ylabel('ʵ����̬���ٶ�( ��/s )');
title('ʵ����̬���ٶ�');
if(flag==1)
    axis([0 300 -0.1 0.1]);
elseif(flag==2)
    axis([0 300 -0.2 0.2]);
end
grid on
%ϵͳ���ٵ���̬���ٶ����
% figure(4);
% plot(t,(attitude_angle_dao(:,1)-qd_dao(:,1))*180/pi,'b-','linewidth',2);
% hold on;
% plot(t,(attitude_angle_dao(:,2)-qd_dao(:,2))*180/pi,'r--','linewidth',2);
% plot(t,(attitude_angle_dao(:,3)-qd_dao(:,3))*180/pi,'k:','linewidth',2);
% hold off;
% legend('��d��/dt','��d��/dt','��d��/dt');
% xlabel('ʱ��( sec )');
% ylabel('��̬���ٶ����( ��/s )');
% title('��̬���ٶ����');
% axis([0 300 -0.5 0.5]);
% % axis([0 300 -0.4 0.1]);
% grid on
%�������ٵ�֧��ת����ʵ�ʵ�֧��ת��
figure(5);
subplot(1,2,1);
plot(t,qd(:,4)*180/pi,'b-','linewidth',2);
%legend('��');
xlabel('ʱ��( sec )');
ylabel('����֧��ת��( �� )');
title('����֧��ת��');
% axis([0 200 -40 40]);
grid on
subplot(1,2,2);
plot(t,(alpha-qd(:,4))*180/pi,'b-','linewidth',2);
xlabel('ʱ��( sec )');
ylabel('֧��ת�����( �� )');
title('֧��ת�����');
if(flag==1)
    axis([0 150 -1 15]);
elseif(flag==2)
    axis([0 150 -1 15]);
end
% plot(t,alpha*180/pi,'b-','linewidth',2);
% %legend('��');
% xlabel('ʱ��( sec )');
% ylabel('ʵ��֧��ת��( �� )');
% title('ʵ��֧��ת��');
% axis([0 200 -40 40]);
grid on
%�������ٵ���Ч�غ�ת����ʵ�ʵ���Ч�غ�ת��
figure(6);
subplot(1,2,1);
plot(t,qd(:,5)*180/pi,'r-','linewidth',2);
%legend('��');
xlabel('ʱ��( sec )');
ylabel('������Ч�غ�ת��( �� )');
title('������Ч�غ�ת��');
grid on
subplot(1,2,2);
plot(t,(beta-qd(:,5))*180/pi,'r-','linewidth',2);
xlabel('ʱ��( sec )');
ylabel('��Ч�غ�ת�����( �� )');
title('��Ч�غ�ת�����');
if(flag==1)
    axis([0 150 -1 20]);
elseif(flag==2)
    axis([0 150 -1 15]);
end
% plot(t,beta*180/pi,'r-','linewidth',2);
% %legend('��');
% xlabel('ʱ��( sec )');
% ylabel('ʵ����Ч�غ�ת��( �� )');
% title('ʵ����Ч�غ�ת��');
grid on
%֧����Ч�غ�����ת�ǵ����
% figure(7);
% plot(t,(alpha-qd(:,4))*180/pi,'b-','linewidth',2);
% hold on;
% plot(t,(beta-qd(:,5))*180/pi,'r--','linewidth',2);
% hold off;
% legend('����','����');
% xlabel('ʱ��( sec )');
% ylabel('֧��ת��������Ч�غ�ת�����( �� )');
% title('֧��ת��������Ч�غ�ת�����');
% axis([0 50 -20 5]);
% axis([0 100 -8 6]);
% grid on
%�������ٵ�֧�ܽ��ٶ���ʵ�ʵ�֧�ܽ��ٶ�
figure(8);
subplot(1,2,1);
plot(t,qd_dao(:,4)*180/pi,'b-','linewidth',2);
xlabel('ʱ��( sec )');
ylabel('����֧�ܽ��ٶ�( ��/s )');
title('����֧�ܽ��ٶ�');
% axis([0 200 -8 10]);
grid on
subplot(1,2,2);
plot(t,(alpha_dao-qd_dao(:,4))*180/pi,'b-','linewidth',2);
xlabel('ʱ��( sec )');
ylabel('֧�ܽ��ٶ����( ��/s )');
title('֧�ܽ��ٶ����');
if(flag==1)
    axis([0 150 -3 0.3]);
elseif(flag==2)
    axis([0 150 -1 0.5]);
end
% plot(t,alpha_dao*180/pi,'b-','linewidth',2);
% xlabel('ʱ��( sec )');
% ylabel('ʵ��֧�ܽ��ٶ�( ��/s )');
% title('ʵ��֧�ܽ��ٶ�');
% axis([0 200 -8 10]);
grid on
figure(9);
subplot(1,2,1);
plot(t,qd_dao(:,5)*180/pi,'r-','linewidth',2);
%legend('d��/dt');
xlabel('ʱ��( sec )');
ylabel('������Ч�غɽ��ٶ�( ��/s )');
title('������Ч�غɽ��ٶ�');
% axis([0 200 -8 10]);
grid on
subplot(1,2,2);
plot(t,(beta_dao-qd_dao(:,5))*180/pi,'r-','linewidth',2);
xlabel('ʱ��( sec )');
ylabel('��Ч�غɽ��ٶ����( ��/s )');
title('��Ч�غɽ��ٶ����');
if(flag==1)
    axis([0 150 -4 0.5]);
elseif(flag==2)
    axis([0 150 -1.5 0.5]);
end
% plot(t,beta_dao*180/pi,'r-','linewidth',2);
% %legend('d��/dt');
% xlabel('ʱ��( sec )');
% ylabel('ʵ����Ч�غɽ��ٶ�( ��/s )');
% title('ʵ����Ч�غɽ��ٶ�');
%%%
grid on
%֧����Ч�غ�������ٶ����
% figure(10);
% plot(t,(alpha_dao-qd_dao(:,4))*180/pi,'b-','linewidth',2);
% hold on;
% plot(t,(beta_dao-qd_dao(:,5))*180/pi,'r--','linewidth',2);
% hold off;
% legend('��d��/dt','��d��/dt');
% xlabel('ʱ��( sec )');
% ylabel('֧�ܽ��ٶ�������Ч�غɽ��ٶ����( ��/s )');
% title('֧�ܽ��ٶ�������Ч�غɽ��ٶ����');
% axis([0 50 -1 5]);
% grid on
%���Ǳ�����Խ��ٶ�
figure(11);
plot(t,omega(:,1)*180/pi,'b-','linewidth',2);
hold on;
plot(t,omega(:,2)*180/pi,'r--','linewidth',2);
plot(t,omega(:,3)*180/pi,'k:','linewidth',2);
hold off;
legend('��_x','��_y','��_z');
xlabel('ʱ��( sec )');
ylabel('���Ǳ�����Խ��ٶ�( ��/s )');
title('���Ǳ�����Խ��ٶ�');
grid on
%���������������Ч�غ������������
figure(12);
subplot(1,2,1);
plot(t,F(:,1),'b-','linewidth',2);
hold on;
plot(t,F(:,2),'r--','linewidth',2);
plot(t,F(:,3),'k:','linewidth',2);
hold off;
legend('T_��','T_��','T_��');
xlabel('ʱ��( sec )');
ylabel('���Ǳ����������( Nm )');
title('���Ǳ��������������');
if(flag==1)
    axis([0 150 -20 5]);
elseif(flag==2)
    axis([0 150 -10 5]);
end
grid on
subplot(1,2,2);
plot(t,F(:,4),'b-','linewidth',2);
hold on;
plot(t,F(:,5),'r--','linewidth',2);
hold off;
legend('T_��','T_��');
xlabel('ʱ��( sec )');
ylabel('֧�ܺ���Ч�غɿ�������( Nm )');
title('֧�ܺ���Ч�غ������������');
if(flag==1)
    axis([0 150 -20 5]);
elseif(flag==2)
    axis([0 150 -4 1]);
end
grid on
%��֤�������ȵĴ�С�Ի������Ӱ��
figure(13);
subplot(1,2,1);
plot(t,s(:,1),'b-','linewidth',2);
hold on;
plot(t,s(:,2),'r--','linewidth',2);
plot(t,s(:,3),'k:','linewidth',2);
hold off;
legend('������','������','ƫ����');
xlabel('ʱ��( sec )');
ylabel('s');
title('���Ǳ��廬����');
if(flag==1)
    axis([0 30 -0.002 0.003]);
elseif(flag==2)
    axis([0 30 -0.002 0.004]);
end
grid on
subplot(1,2,2);
plot(t,s(:,4),'b-','linewidth',2);
hold on;
plot(t,s(:,5),'r--','linewidth',2);
hold off;
legend('֧����ת��','��Ч�غ���ת��');
xlabel('ʱ��( sec )');
ylabel('s');
title('֧�ܻ��������Ч�غɻ�����');
if(flag==1)
    axis([0 30 -0.05 0.15]);
elseif(flag==2)
    axis([0 30 -0.01 0.03]);
end
% axis([0 30 -0.06 0.06]);
grid on
if(flag==1)
    figure(15)
    plot(t,lo,'b-','linewidth',2);
    hold on;
    plot(t,la,'r--','linewidth',2);
    hold off;
    legend('�������','γ�����');
    xlabel('ʱ��( sec )');
    ylabel('��');
    title('��γ�����');
    axis([0 150 -0.1 20]);
%     axis([100 300 -10 10]);
    grid on
elseif(flag==2)
    figure(15)
    plot(t,la*180/pi,'b-','linewidth',2);
    xlabel('ʱ��( sec )');
    ylabel('��');
    title('ʵ�ʸ���������ָ�����߼н�');
    %axis([0 30 -0.16 0.06]);
    grid on
end