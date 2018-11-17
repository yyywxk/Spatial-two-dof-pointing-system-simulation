function Attitude_dot=sys_kinematicfun(X)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%��������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Attitude=X(1:3);
phi=Attitude(1);%������
theta=Attitude(2);%������
psi=Attitude(3);%ƫ����
omega0=X(4);
omega=X(5:7);%������ٶ�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����������Թ������ϵ���ٶ�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Lz=[cos(psi) sin(psi) 0;
    -sin(psi) cos(psi) 0;
    0 0 1];%��z���Ԫ��ת����
Ly=[cos(theta) 0 -sin(theta);
    0 1 0;
    sin(theta) 0 cos(theta)];%��y���Ԫ��ת����
Lx=[1 0 0;
    0 cos(phi) sin(phi);
    0 -sin(phi) cos(phi)];%��x���Ԫ��ת����
Abr=Ly*Lx*Lz;%�������ϵ�����Ǳ�������ϵ������任����z-x-y��ת˳��
omega0_r=[0;-omega0;0];%�������ϵ�������Ĺ�����ٶ�
omega0_b=Abr*omega0_r;%���Ǳ�������ϵ�������Ĺ�����ٶ�
omegar_b=omega-omega0_b;%���Ǳ�������ϵ����������Խ��ٶ�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%������̬��ʱ�䵼��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
G=[cos(theta) 0 sin(theta);
    tan(phi)*sin(theta) 1 -tan(phi)*cos(theta);
    -sin(theta)/cos(phi) 0 cos(theta)/cos(phi)];%�˶�ѧ��ϵ����
Attitude_dot=G*omegar_b;%��̬��ʱ�䵼��
end

