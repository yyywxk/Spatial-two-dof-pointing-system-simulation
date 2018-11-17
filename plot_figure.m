close all
global flag
%绘制图像
%期望跟踪的姿态角及实际跟踪的姿态角
figure(1);
subplot(1,2,1); 
plot(t,qd(:,1)*180/pi,'b-','linewidth',2);
hold on;
plot(t,qd(:,2)*180/pi,'r--','linewidth',2);
plot(t,qd(:,3)*180/pi,'k:','linewidth',2);
hold off;
legend('φ','θ','ψ');
xlabel('时间( sec )');
ylabel('期望姿态角( ° )');
title('期望姿态角');
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
legend('φ','θ','ψ');
xlabel('时间( sec )');
ylabel('实际姿态角( ° )');
title('实际姿态角');
if(flag==1)
    axis([0 300 -4 4]);
elseif(flag==2)
    axis([0 300 -6 6]);
end
grid on
%系统跟踪的姿态角误差
% figure(2);
% plot(t,(attitude_angle(:,1)-qd(:,1))*180/pi,'b-','linewidth',2);
% hold on;
% plot(t,(attitude_angle(:,2)-qd(:,2))*180/pi,'r--','linewidth',2);
% plot(t,(attitude_angle(:,3)-qd(:,3))*180/pi,'k:','linewidth',2);
% hold off;
% legend('Δφ','Δθ','Δψ');
% xlabel('时间( sec )');
% ylabel('姿态角误差( ° )');
% title('姿态角误差');
% axis([0 300 -6 12]);
% grid on
%期望跟踪的姿态角速度与实际跟踪的姿态角速度
figure(3);
subplot(1,2,1);
plot(t,qd_dao(:,1)*180/pi,'b-','linewidth',2);
hold on;
plot(t,qd_dao(:,2)*180/pi,'r--','linewidth',2);
plot(t,qd_dao(:,3)*180/pi,'k:','linewidth',2);
hold off;
legend('dφ/dt','dθ/dt','dψ/dt');
xlabel('时间( sec )');
ylabel('期望姿态角速度( °/s )');
title('期望姿态角速度');
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
legend('dφ/dt','dθ/dt','dψ/dt');
xlabel('时间( sec )');
ylabel('实际姿态角速度( °/s )');
title('实际姿态角速度');
if(flag==1)
    axis([0 300 -0.1 0.1]);
elseif(flag==2)
    axis([0 300 -0.2 0.2]);
end
grid on
%系统跟踪的姿态角速度误差
% figure(4);
% plot(t,(attitude_angle_dao(:,1)-qd_dao(:,1))*180/pi,'b-','linewidth',2);
% hold on;
% plot(t,(attitude_angle_dao(:,2)-qd_dao(:,2))*180/pi,'r--','linewidth',2);
% plot(t,(attitude_angle_dao(:,3)-qd_dao(:,3))*180/pi,'k:','linewidth',2);
% hold off;
% legend('Δdφ/dt','Δdθ/dt','Δdψ/dt');
% xlabel('时间( sec )');
% ylabel('姿态角速度误差( °/s )');
% title('姿态角速度误差');
% axis([0 300 -0.5 0.5]);
% % axis([0 300 -0.4 0.1]);
% grid on
%期望跟踪的支架转角与实际的支架转角
figure(5);
subplot(1,2,1);
plot(t,qd(:,4)*180/pi,'b-','linewidth',2);
%legend('α');
xlabel('时间( sec )');
ylabel('期望支架转角( ° )');
title('期望支架转角');
% axis([0 200 -40 40]);
grid on
subplot(1,2,2);
plot(t,(alpha-qd(:,4))*180/pi,'b-','linewidth',2);
xlabel('时间( sec )');
ylabel('支架转角误差( ° )');
title('支架转角误差');
if(flag==1)
    axis([0 150 -1 15]);
elseif(flag==2)
    axis([0 150 -1 15]);
end
% plot(t,alpha*180/pi,'b-','linewidth',2);
% %legend('α');
% xlabel('时间( sec )');
% ylabel('实际支架转角( ° )');
% title('实际支架转角');
% axis([0 200 -40 40]);
grid on
%期望跟踪的有效载荷转角与实际的有效载荷转角
figure(6);
subplot(1,2,1);
plot(t,qd(:,5)*180/pi,'r-','linewidth',2);
%legend('β');
xlabel('时间( sec )');
ylabel('期望有效载荷转角( ° )');
title('期望有效载荷转角');
grid on
subplot(1,2,2);
plot(t,(beta-qd(:,5))*180/pi,'r-','linewidth',2);
xlabel('时间( sec )');
ylabel('有效载荷转角误差( ° )');
title('有效载荷转角误差');
if(flag==1)
    axis([0 150 -1 20]);
elseif(flag==2)
    axis([0 150 -1 15]);
end
% plot(t,beta*180/pi,'r-','linewidth',2);
% %legend('β');
% xlabel('时间( sec )');
% ylabel('实际有效载荷转角( ° )');
% title('实际有效载荷转角');
grid on
%支架有效载荷两轴转角的误差
% figure(7);
% plot(t,(alpha-qd(:,4))*180/pi,'b-','linewidth',2);
% hold on;
% plot(t,(beta-qd(:,5))*180/pi,'r--','linewidth',2);
% hold off;
% legend('Δα','Δβ');
% xlabel('时间( sec )');
% ylabel('支架转角误差和有效载荷转角误差( ° )');
% title('支架转角误差和有效载荷转角误差');
% axis([0 50 -20 5]);
% axis([0 100 -8 6]);
% grid on
%期望跟踪的支架角速度与实际的支架角速度
figure(8);
subplot(1,2,1);
plot(t,qd_dao(:,4)*180/pi,'b-','linewidth',2);
xlabel('时间( sec )');
ylabel('期望支架角速度( °/s )');
title('期望支架角速度');
% axis([0 200 -8 10]);
grid on
subplot(1,2,2);
plot(t,(alpha_dao-qd_dao(:,4))*180/pi,'b-','linewidth',2);
xlabel('时间( sec )');
ylabel('支架角速度误差( °/s )');
title('支架角速度误差');
if(flag==1)
    axis([0 150 -3 0.3]);
elseif(flag==2)
    axis([0 150 -1 0.5]);
end
% plot(t,alpha_dao*180/pi,'b-','linewidth',2);
% xlabel('时间( sec )');
% ylabel('实际支架角速度( °/s )');
% title('实际支架角速度');
% axis([0 200 -8 10]);
grid on
figure(9);
subplot(1,2,1);
plot(t,qd_dao(:,5)*180/pi,'r-','linewidth',2);
%legend('dβ/dt');
xlabel('时间( sec )');
ylabel('期望有效载荷角速度( °/s )');
title('期望有效载荷角速度');
% axis([0 200 -8 10]);
grid on
subplot(1,2,2);
plot(t,(beta_dao-qd_dao(:,5))*180/pi,'r-','linewidth',2);
xlabel('时间( sec )');
ylabel('有效载荷角速度误差( °/s )');
title('有效载荷角速度误差');
if(flag==1)
    axis([0 150 -4 0.5]);
elseif(flag==2)
    axis([0 150 -1.5 0.5]);
end
% plot(t,beta_dao*180/pi,'r-','linewidth',2);
% %legend('dβ/dt');
% xlabel('时间( sec )');
% ylabel('实际有效载荷角速度( °/s )');
% title('实际有效载荷角速度');
%%%
grid on
%支架有效载荷两轴角速度误差
% figure(10);
% plot(t,(alpha_dao-qd_dao(:,4))*180/pi,'b-','linewidth',2);
% hold on;
% plot(t,(beta_dao-qd_dao(:,5))*180/pi,'r--','linewidth',2);
% hold off;
% legend('Δdα/dt','Δdβ/dt');
% xlabel('时间( sec )');
% ylabel('支架角速度误差和有效载荷角速度误差( °/s )');
% title('支架角速度误差和有效载荷角速度误差');
% axis([0 50 -1 5]);
% grid on
%卫星本体绝对角速度
figure(11);
plot(t,omega(:,1)*180/pi,'b-','linewidth',2);
hold on;
plot(t,omega(:,2)*180/pi,'r--','linewidth',2);
plot(t,omega(:,3)*180/pi,'k:','linewidth',2);
hold off;
legend('ω_x','ω_y','ω_z');
xlabel('时间( sec )');
ylabel('卫星本体绝对角速度( °/s )');
title('卫星本体绝对角速度');
grid on
%三轴控制力矩与有效载荷两轴控制力矩
figure(12);
subplot(1,2,1);
plot(t,F(:,1),'b-','linewidth',2);
hold on;
plot(t,F(:,2),'r--','linewidth',2);
plot(t,F(:,3),'k:','linewidth',2);
hold off;
legend('T_φ','T_θ','T_ψ');
xlabel('时间( sec )');
ylabel('卫星本体控制力矩( Nm )');
title('卫星本体三轴控制力矩');
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
legend('T_α','T_β');
xlabel('时间( sec )');
ylabel('支架和有效载荷控制力矩( Nm )');
title('支架和有效载荷两轴控制力矩');
if(flag==1)
    axis([0 150 -20 5]);
elseif(flag==2)
    axis([0 150 -4 1]);
end
grid on
%验证滑动面厚度的大小对滑动面的影响
figure(13);
subplot(1,2,1);
plot(t,s(:,1),'b-','linewidth',2);
hold on;
plot(t,s(:,2),'r--','linewidth',2);
plot(t,s(:,3),'k:','linewidth',2);
hold off;
legend('滚动轴','俯仰轴','偏航轴');
xlabel('时间( sec )');
ylabel('s');
title('卫星本体滑动面');
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
legend('支架旋转轴','有效载荷旋转轴');
xlabel('时间( sec )');
ylabel('s');
title('支架滑动面和有效载荷滑动面');
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
    legend('经度误差','纬度误差');
    xlabel('时间( sec )');
    ylabel('°');
    title('经纬度误差');
    axis([0 150 -0.1 20]);
%     axis([100 300 -10 10]);
    grid on
elseif(flag==2)
    figure(15)
    plot(t,la*180/pi,'b-','linewidth',2);
    xlabel('时间( sec )');
    ylabel('°');
    title('实际跟踪视线与指令视线夹角');
    %axis([0 30 -0.16 0.06]);
    grid on
end