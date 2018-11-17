function y=dynamicsfun(X)%���庯�������ϵͳ������ٶ�
global mh mp m Sb Sh Sp rb_h rh_p Ib Ih Ip f11 f12 f21 f22
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%���������������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
v=X(1:3);%���Ǳ��������ٶ�
omega=X(4:6);%���Ǳ�����ٶ�
alpha_dao=X(7);%֧����ת�����ٶ�
beta_dao=X(8);%��Ч�غ�ת�����ٶ�
F=X(9:16);%ϵͳ����������
alpha=X(17);
beta=X(18);
%theta_p=X(17:18);%��Ч�غ�����ת�� 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����ϸ������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% alpha_dao=theta_p_dao(1);%��Ч�غ���ת��1��ת�����ٶ�
% beta_dao=theta_p_dao(2);%��Ч�غ���ת��2��ת�����ٶ�
% alpha=theta_p(1);%��Ч�غ���ת��1��ת��
% beta=theta_p(2);%��Ч�غ���ת��2��ת��
Fb=F(1:3);%���Ǳ������ܵ���������ʸ
Tb=F(4:6);%���Ǳ������ܵ�����
Th=F(7);%��������ת��1������
Tp=F(8);%��������ת��2������
%Tpg=F(7:8);%������������ת������أ�Tpg=(zeta_beta)'*Tp)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����ת��������Ч�غ��˶�ѧ��ϵ����ļ���
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ly_beta=[cos(beta) 0 -sin(beta);
         0         1 0;
         sin(beta) 0 cos(beta)];
Lz_alpha=[cos(alpha)  sin(alpha) 0;
          -sin(alpha) cos(alpha) 0;
          0           0          1];
Ahb=Lz_alpha;%���Ǳ�������ϵ��֧������ϵ������ת������
Abh=Ahb';%֧������ϵ�����Ǳ�������ϵ������ת������
Aph=Ly_beta;%֧������ϵ����Ч�غ�����ϵ������ת������
Ahp=Aph';%��Ч�غ�����ϵ��֧������ϵ������ת������
Apb=Ly_beta*Lz_alpha;%���Ǳ�������ϵ����Ч�غ�����ϵ������ת������
Abp=Apb';%��Ч�غ�����ϵ�����Ǳ�������ϵ������ת������
zetap_h=[0;0;1];%֧���˶�ѧ��ϵ����
zetap_p=[0;1;0];%��Ч�غ��˶�ѧ��ϵ����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����ϵͳ������M
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����ƽ�����̵�ϵ��
rb_p=rb_h+Abh*rh_p;
Sbh=mh*rb_h+Abh*Sh;%֧�ܶ����Ǳ�������ϵ�ľ���
Shp=mp*rh_p+Ahp*Sp;%��Ч�غɶ�֧��������ϵ�ľ���
Sbp=mp*rb_p+Abp*Sp;%��Ч�غɶ����Ǳ�������ϵ�ľ���
Sbt=Sb+Sbh+Sbp;%ϵͳ�����Ǳ�������ϵ�ľ���
Sht=Sb+Shp;%֧������Ч�غɹ��ɵ�ϵͳ��֧��������ϵ�ľ���
Sbx=Sb+mh*rb_h+mp*rb_h;%������������䱾������ϵ�ľ���
Shx=Sh+mp*rh_p;%֧����Ч�غ��������֧��������ϵ�ľ���
Sh_cross=crossarrayfun(Sh);
Sp_cross=crossarrayfun(Sp);
Sbh_cross=crossarrayfun(Sbh);
Shp_cross=crossarrayfun(Shp);
Sbp_cross=crossarrayfun(Sbp);
Sbt_cross=crossarrayfun(Sbt);
Sht_cross=crossarrayfun(Sht);
Sbx_cross=crossarrayfun(Sbx);
Shx_cross=crossarrayfun(Shx);
%����ת�����̵�ϵ��
omega_h=zetap_h*alpha_dao;%֧�ܶ����ǵ���Խ��ٶ�
omega_p=zetap_p*beta_dao;%��Ч�غɶ�֧�ܵ���Խ��ٶ�
OMEGA_h=omega_h+Ahb*omega;%֧�ܾ��Խ��ٶ�
OMEGA_p=omega_p+Aph*omega_h+Apb*omega;%��Ч�غɾ��Խ��ٶ�
omega_cross=crossarrayfun(omega);
OMEGA_h_cross=crossarrayfun(OMEGA_h);
OMEGA_p_cross=crossarrayfun(OMEGA_p);
rb_h_cross=crossarrayfun(rb_h);
rh_p_cross=crossarrayfun(rh_p);
rb_p_cross=crossarrayfun(rb_p);
Ibh=mh*(rb_h_cross)'*rb_h_cross+(rb_h_cross)'*Abh*Sh_cross*Ahb+Abh*(Sh_cross)'*Ahb*rb_h_cross+Abh*Ih*Ahb;%֧�ܶ���������ϵ�Ĺ�������
Ihp=mp*(rh_p_cross)'*rh_p_cross+(rh_p_cross)'*Ahp*Sp_cross*Aph+Ahp*(Sp_cross)'*Aph*rh_p_cross+Ahp*Ip*Aph;%��Ч�غɶ�֧������ϵ�Ĺ�������
Ibp=mp*(rb_p_cross)'*rb_p_cross+(rb_p_cross)'*Abp*Sp_cross*Apb+Abp*(Sp_cross)'*Apb*rb_p_cross+Abp*Ip*Apb;%��Ч�غɶ���������ϵ�Ĺ�������
Ibt=Ib+Ibh+Ibp;%ϵͳ����������ϵ�Ĺ�������
Iht=Ih+Ihp;%֧������Ч�غɹ��ɵ�ϵͳ��֧������ϵ�Ĺ�������
I_h_b=Ahb*(rb_h_cross)'*Abh*Sht_cross+Iht;%֧�ܶ���������ϵ�Ĺ�����Ͼ���
I_p_b=Apb*(rb_p_cross)'*Abp*Sp_cross+Ip;%��Ч�غɶ���������ϵ�Ĺ�����Ͼ���
I_p_h=Aph*(rh_p_cross)'*Ahp*Sp_cross+Ip;%��Ч�غɶ�֧������ϵ�Ĺ�����Ͼ���
I_OMEGA_h_b=OMEGA_h_cross*Ih+Ahb*(rb_h_cross)'*Abh*OMEGA_h_cross*Sh_cross+Ahb*(Sbp_cross)'*Abh*OMEGA_h_cross*rh_p_cross;%��Ч�غɶ���������ϵ�����������
I_OMEGA_p_b=OMEGA_p_cross*Ip+Apb*(rb_p_cross)'*Abp*OMEGA_p_cross*Sp_cross;%��Ч�غɶ���������ϵ�����������
I_OMEGA_h_h=OMEGA_h_cross*Ih+(Shp_cross)'*OMEGA_h_cross*rh_p_cross;%�������֧�ܶ���������ϵ�����������
I_OMEGA_p_h=OMEGA_p_cross*Ip+Aph*(rh_p_cross)'*Ahp*OMEGA_p_cross*Sp_cross;%���������Ч�غɶ�֧������ϵ�����������
I_OMEGA_h_p=Ahp*(Sp_cross)'*Aph*OMEGA_h_cross*rh_p_cross;%�������֧�ܶ���Ч�غ�������ϵ�����������
I_OMEGA_p_p=OMEGA_p_cross*Ip;%���������Ч�غɶ���������ϵ�����������
Iw=omega_cross*Ib+(Sbh_cross)'*omega_cross*rb_h_cross+(Sbp_cross)'*omega_cross*rb_h_cross;%ϵͳ����������ϵ�����������
%����M
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
   M41 M42 M43 M44];%ϵͳ������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����ϵͳ�ķ����������
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
Q=[Qv;Qw;(zetap_h)'*Qwh;(zetap_p)'*Qwp];%ϵͳ�ķ����������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����ϵͳ�ĸ�������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Fd=zeros(8,1);
Fd(7)=-f11*tanh(f12*alpha_dao);
% Fd(7)=0;
Fd(8)=-f21*tanh(f22*beta_dao);
% Fd(8)=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%��ϵͳ������ٶ�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dudt=M\(F-Q+Fd);
% dudt=M\(F-Q);
y=dudt;
end