function Attitude_dot=sys_kinematicfun(X)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%变量定义
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Attitude=X(1:3);
phi=Attitude(1);%滚动角
theta=Attitude(2);%俯仰角
psi=Attitude(3);%偏航角
omega0=X(4);
omega=X(5:7);%星体角速度
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%计算星体相对轨道坐标系角速度
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Lz=[cos(psi) sin(psi) 0;
    -sin(psi) cos(psi) 0;
    0 0 1];%绕z轴基元旋转矩阵
Ly=[cos(theta) 0 -sin(theta);
    0 1 0;
    sin(theta) 0 cos(theta)];%绕y轴基元旋转矩阵
Lx=[1 0 0;
    0 cos(phi) sin(phi);
    0 -sin(phi) cos(phi)];%绕x轴基元旋转矩阵
Abr=Ly*Lx*Lz;%轨道坐标系到卫星本体坐标系的坐标变换矩阵（z-x-y旋转顺序）
omega0_r=[0;-omega0;0];%轨道坐标系中描述的轨道角速度
omega0_b=Abr*omega0_r;%卫星本体坐标系中描述的轨道角速度
omegar_b=omega-omega0_b;%卫星本体坐标系中描述的相对角速度
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%计算姿态角时间导数
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
G=[cos(theta) 0 sin(theta);
    tan(phi)*sin(theta) 1 -tan(phi)*cos(theta);
    -sin(theta)/cos(phi) 0 cos(theta)/cos(phi)];%运动学关系矩阵
Attitude_dot=G*omegar_b;%姿态角时间导数
end

