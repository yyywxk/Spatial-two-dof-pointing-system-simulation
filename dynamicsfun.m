function y=dynamicsfun(X)%定义函数，求解系统广义加速度
global mh mp m Sb Sh Sp rb_h rh_p Ib Ih Ip f11 f12 f21 f22
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%函数输入变量定义
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
v=X(1:3);%卫星本体质心速度
omega=X(4:6);%卫星本体角速度
alpha_dao=X(7);%支架绕转动角速度
beta_dao=X(8);%有效载荷转动角速度
F=X(9:16);%系统广义主动力
alpha=X(17);
beta=X(18);
%theta_p=X(17:18);%有效载荷两轴转角 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%变量细化定义
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% alpha_dao=theta_p_dao(1);%有效载荷旋转轴1的转动角速度
% beta_dao=theta_p_dao(2);%有效载荷旋转轴2的转动角速度
% alpha=theta_p(1);%有效载荷旋转轴1的转角
% beta=theta_p(2);%有效载荷旋转轴2的转角
Fb=F(1:3);%卫星本体所受的作用力主矢
Tb=F(4:6);%卫星本体所受的主矩
Th=F(7);%作用于旋转轴1的力矩
Tp=F(8);%作用于旋转轴2的力矩
%Tpg=F(7:8);%作用于两个旋转轴的力矩（Tpg=(zeta_beta)'*Tp)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%坐标转换矩阵及有效载荷运动学关系矩阵的计算
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ly_beta=[cos(beta) 0 -sin(beta);
         0         1 0;
         sin(beta) 0 cos(beta)];
Lz_alpha=[cos(alpha)  sin(alpha) 0;
          -sin(alpha) cos(alpha) 0;
          0           0          1];
Ahb=Lz_alpha;%卫星本体坐标系到支架坐标系的坐标转换矩阵
Abh=Ahb';%支架坐标系到卫星本体坐标系的坐标转换矩阵
Aph=Ly_beta;%支架坐标系到有效载荷坐标系的坐标转换矩阵
Ahp=Aph';%有效载荷坐标系到支架坐标系的坐标转换矩阵
Apb=Ly_beta*Lz_alpha;%卫星本体坐标系到有效载荷坐标系的坐标转换矩阵
Abp=Apb';%有效载荷坐标系到卫星本体坐标系的坐标转换矩阵
zetap_h=[0;0;1];%支架运动学关系矩阵
zetap_p=[0;1;0];%有效载荷运动学关系矩阵
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%计算系统质量阵M
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%卫星平动方程的系数
rb_p=rb_h+Abh*rh_p;
Sbh=mh*rb_h+Abh*Sh;%支架对卫星本体坐标系的静矩
Shp=mp*rh_p+Ahp*Sp;%有效载荷对支架体坐标系的静矩
Sbp=mp*rb_p+Abp*Sp;%有效载荷对卫星本体坐标系的静矩
Sbt=Sb+Sbh+Sbp;%系统对卫星本体坐标系的静矩
Sht=Sb+Shp;%支架与有效载荷构成的系统对支架体坐标系的静矩
Sbx=Sb+mh*rb_h+mp*rb_h;%星体增广体对其本体坐标系的静矩
Shx=Sh+mp*rh_p;%支架有效载荷增广体对支架体坐标系的静矩
Sh_cross=crossarrayfun(Sh);
Sp_cross=crossarrayfun(Sp);
Sbh_cross=crossarrayfun(Sbh);
Shp_cross=crossarrayfun(Shp);
Sbp_cross=crossarrayfun(Sbp);
Sbt_cross=crossarrayfun(Sbt);
Sht_cross=crossarrayfun(Sht);
Sbx_cross=crossarrayfun(Sbx);
Shx_cross=crossarrayfun(Shx);
%卫星转动方程的系数
omega_h=zetap_h*alpha_dao;%支架对卫星的相对角速度
omega_p=zetap_p*beta_dao;%有效载荷对支架的相对角速度
OMEGA_h=omega_h+Ahb*omega;%支架绝对角速度
OMEGA_p=omega_p+Aph*omega_h+Apb*omega;%有效载荷绝对角速度
omega_cross=crossarrayfun(omega);
OMEGA_h_cross=crossarrayfun(OMEGA_h);
OMEGA_p_cross=crossarrayfun(OMEGA_p);
rb_h_cross=crossarrayfun(rb_h);
rh_p_cross=crossarrayfun(rh_p);
rb_p_cross=crossarrayfun(rb_p);
Ibh=mh*(rb_h_cross)'*rb_h_cross+(rb_h_cross)'*Abh*Sh_cross*Ahb+Abh*(Sh_cross)'*Ahb*rb_h_cross+Abh*Ih*Ahb;%支架对星体坐标系的惯量矩阵
Ihp=mp*(rh_p_cross)'*rh_p_cross+(rh_p_cross)'*Ahp*Sp_cross*Aph+Ahp*(Sp_cross)'*Aph*rh_p_cross+Ahp*Ip*Aph;%有效载荷对支架坐标系的惯量矩阵
Ibp=mp*(rb_p_cross)'*rb_p_cross+(rb_p_cross)'*Abp*Sp_cross*Apb+Abp*(Sp_cross)'*Apb*rb_p_cross+Abp*Ip*Apb;%有效载荷对星体坐标系的惯量矩阵
Ibt=Ib+Ibh+Ibp;%系统对星体坐标系的惯量矩阵
Iht=Ih+Ihp;%支架与有效载荷构成的系统对支架坐标系的惯量矩阵
I_h_b=Ahb*(rb_h_cross)'*Abh*Sht_cross+Iht;%支架对星体坐标系的惯量耦合矩阵
I_p_b=Apb*(rb_p_cross)'*Abp*Sp_cross+Ip;%有效载荷对星体坐标系的惯量耦合矩阵
I_p_h=Aph*(rh_p_cross)'*Ahp*Sp_cross+Ip;%有效载荷对支架坐标系的惯量耦合矩阵
I_OMEGA_h_b=OMEGA_h_cross*Ih+Ahb*(rb_h_cross)'*Abh*OMEGA_h_cross*Sh_cross+Ahb*(Sbp_cross)'*Abh*OMEGA_h_cross*rh_p_cross;%有效载荷对星体坐标系的拟惯量矩阵
I_OMEGA_p_b=OMEGA_p_cross*Ip+Apb*(rb_p_cross)'*Abp*OMEGA_p_cross*Sp_cross;%有效载荷对星体坐标系的拟惯量矩阵
I_OMEGA_h_h=OMEGA_h_cross*Ih+(Shp_cross)'*OMEGA_h_cross*rh_p_cross;%修正后的支架对其体坐标系的拟惯量矩阵
I_OMEGA_p_h=OMEGA_p_cross*Ip+Aph*(rh_p_cross)'*Ahp*OMEGA_p_cross*Sp_cross;%修正后的有效载荷对支架坐标系的拟惯量矩阵
I_OMEGA_h_p=Ahp*(Sp_cross)'*Aph*OMEGA_h_cross*rh_p_cross;%修正后的支架对有效载荷体坐标系的拟惯量矩阵
I_OMEGA_p_p=OMEGA_p_cross*Ip;%修正后的有效载荷对其体坐标系的拟惯量矩阵
Iw=omega_cross*Ib+(Sbh_cross)'*omega_cross*rb_h_cross+(Sbp_cross)'*omega_cross*rb_h_cross;%系统对星体坐标系的拟惯量矩阵
%计算M
M11=m*eye(3);
M12=-Sbt_cross;
M13=-Abh*Sht_cross*zetap_h;
M14=-Abp*Sp_cross*zetap_p;
M21=M12';
M22=Ibt;
M23=Abh*I_h_b*zetap_h;
M24=Abp*I_p_b*zetap_p;
M31=M13';
M32=M23';
M33=(zetap_h)'*Iht*zetap_h;
M34=(zetap_h)'*Ahp*I_p_h*zetap_p;
M41=M14';
M42=M24';
M43=M34';
M44=(zetap_p)'*Ip*zetap_p;
M=[M11 M12 M13 M14;
   M21 M22 M23 M24;
   M31 M32 M33 M34;
   M41 M42 M43 M44];%系统质量阵
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%计算系统的非线性耦合力
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Qv1=m*omega_cross*v;
Qv2=-omega_cross*Sbx_cross*omega;
Qv3=-Abh*OMEGA_h_cross*Shx_cross*OMEGA_h;
Qv4=-Abp*OMEGA_p_cross*Sp_cross*OMEGA_p;
Qv5=-Abh*Sht_cross*Ahb*omega_cross*Abh*OMEGA_h;
Qv6=-Abp*Sp_cross*Aph*OMEGA_h_cross*Ahp*OMEGA_p;
Qw1=Sbt_cross*omega_cross*v;
Qw2=Iw*omega;
Qw3=Abh*I_OMEGA_h_b*OMEGA_h;
Qw4=Abp*I_OMEGA_p_b*OMEGA_p;
Qw5=Abh*I_h_b*Ahb*omega_cross*Abh*OMEGA_h;
Qw6=Abp*I_p_b*Aph*OMEGA_h_cross*Ahp*OMEGA_p;
Qwh1=Sht_cross*Ahb*omega_cross*v;
Qwh2=(Sht_cross)'*Ahb*omega_cross*rb_h_cross*omega;
Qwh3=I_OMEGA_h_h*OMEGA_h;
Qwh4=Ahp*I_OMEGA_p_h*OMEGA_p;
Qwh5=Iht*Ahb*omega_cross*Abh*OMEGA_h;
Qwh6=Ahp*I_p_h*Aph*OMEGA_h_cross*Ahp*OMEGA_p;
Qwp1=Sp_cross*Apb*omega_cross*v;
Qwp2=(Sp_cross)'*Apb*omega_cross*rb_h_cross*omega;
Qwp3=Aph*I_OMEGA_h_p*OMEGA_h;
Qwp4=I_OMEGA_p_p*OMEGA_p;
Qwp5=(I_p_h)'*Apb*omega_cross*Abh*OMEGA_h;
Qwp6=Ip*Aph*OMEGA_h_cross*Ahp*OMEGA_p;

Qv=Qv1+Qv2+Qv3+Qv4+Qv5+Qv6;
Qw=Qw1+Qw2+Qw3+Qw4+Qw5+Qw6;
Qwh=Qwh1+Qwh2+Qwh3+Qwh4+Qwh5+Qwh6;
Qwp=Qwp1+Qwp2+Qwp3+Qwp4+Qwp5+Qwp6;
Q=[Qv;Qw;(zetap_h)'*Qwh;(zetap_p)'*Qwp];%系统的非线性耦合力
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%计算系统的干扰力矩
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Fd=zeros(8,1);
Fd(7)=-f11*tanh(f12*alpha_dao);
% Fd(7)=0;
Fd(8)=-f21*tanh(f22*beta_dao);
% Fd(8)=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%求系统广义加速度
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dudt=M\(F-Q+Fd);
% dudt=M\(F-Q);
y=dudt;
end