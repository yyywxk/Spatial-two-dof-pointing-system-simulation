function y = error1(u)
    global Re R omega_0 omega_g phi_g lambda_g flag Rt omega_t i OMEGA_T omega_T
    t=u(1);
    alpha=u(2);
    beta=u(3);
    phi=u(4);%������
    theta=u(5);%������
    psi=u(6);%ƫ����
    OMEGA_s=omega_0 * t;
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
    Arb=Abr';
    A1=cos(alpha)*cos(beta);
    A2=sin(alpha)*cos(beta);
    A3=-sin(beta);
    A=[A1 A2 A3]';
    if(flag==1)
        l=Arb*A;
%         l=A;
%         l=Abr\A;
        root=Re*Re*(l(1)*l(1)+l(2)*l(2)+l(3)*l(3))-R*R*(l(1)*l(1)+l(2)*l(2));
        if(root>=0)
            k=(R*l(3)-sqrt(root))/(l(1)*l(1)+l(2)*l(2)+l(3)*l(3));
%             k=R*sin(beta)-sqrt(Re*Re-R*R*cos(beta)*cos(beta));
            y11=asin(-l(2)*k/Re);
            y1=y11-phi_g;
            y2=asin((R*sin(OMEGA_s)+(l(1)*cos(OMEGA_s)-l(3)*sin(OMEGA_s))*k)/Re/cos(y11))-omega_g*t-lambda_g;
            y=[y1/pi*180,y2/pi*180];
        elseif(root<0)
            y=[0,0];
        end
    elseif(flag==2)
        uu=omega_T+omega_t*t;
        dOMEGA=-omega_0*t+OMEGA_T;
        y1=-cos(dOMEGA)*cos(uu)+sin(dOMEGA)*cos(i)*sin(uu);
        y2=sin(dOMEGA)*cos(uu)+cos(dOMEGA)*cos(i)*sin(uu);
        dx=Rt*y2;
        dz=Rt*y1+R;
        dy=-Rt*sin(i)*sin(uu);
        B1=[dx dy dz]';
        B=Abr*B1;
        y1=acos(dot(A,B)/(norm(A)*norm(B)));%�����ƣ�ת�Ƕ��Ƴ�180/pi
        y=[y1,0];
    end
end