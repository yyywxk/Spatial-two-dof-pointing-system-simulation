function y = beta_d_dao(u)
    global flag Re R omega_c phi_g lambda_g Rt omega_t omega_0 i OMEGA_T omega_T
    if(flag==1)
        %%地面目标
        dx=Re*cos(phi_g)*sin(omega_c*u+lambda_g);
        dy=-Re*sin(phi_g);
        dz=-Re*cos(phi_g)*cos(omega_c*u+lambda_g)+R;
        xbd=2*Re*cos(phi_g)*sin(omega_c*u+lambda_g)*omega_c;
        dzd=Re*cos(phi_g)*sin(omega_c*u+lambda_g)*omega_c;
        xb=dx*dx+dy*dy+dz*dz;
        y1=asin(-dz/sqrt(xb));
    elseif(flag==2)
        %%运动目标
        uu=omega_T+omega_t*u;
        dOMEGA=-omega_0*u+OMEGA_T;
        dx=Rt*(sin(dOMEGA)*cos(uu)+cos(dOMEGA)*cos(i)*sin(uu));
        dy=-Rt*sin(i)*sin(uu);
        dz=Rt*(-cos(dOMEGA)*cos(uu)+sin(dOMEGA)*cos(i)*sin(uu))+R;
        xb=dx*dx+dy*dy+dz*dz;
        y1=asin(-dz/sqrt(xb));
        y11=cos(dOMEGA)*sin(uu)*omega_t+sin(dOMEGA)*cos(i)*cos(uu)*omega_t...
            -sin(dOMEGA)*cos(uu)*omega_0-cos(dOMEGA)*cos(i)*sin(uu)*omega_0;
        xbd=2*Rt*R*y11;
        dzd=Rt*y11;
    end 
    y=1/sin(pi*2-y1)*(-2*xb*dzd+xbd*dz)/(2*xb*sqrt(xb));
end