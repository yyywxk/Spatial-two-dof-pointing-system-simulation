function y = alpha_d_dao(u)
    global flag Re R omega_c phi_g lambda_g Rt omega_t omega_0 i OMEGA_T omega_T
    if(flag==1)
        %%地面目标
        dx=Re*cos(phi_g)*sin(omega_c*u+lambda_g);
        dy=-Re*sin(phi_g);
        dxd=Re*cos(phi_g)*cos(omega_c*u+lambda_g)*omega_c;
        dyd=0;
    elseif(flag==2)
        %%运动目标
        uu=omega_T+omega_t*u;
        dOMEGA=-omega_0*u+OMEGA_T;
        dx=Rt*(sin(dOMEGA)*cos(uu)+cos(dOMEGA)*cos(i)*sin(uu));
        dy=-Rt*sin(i)*sin(uu);
        y1=sin(dOMEGA)*cos(i)*sin(uu)*omega_0+cos(dOMEGA)*cos(i)*cos(uu)*omega_t...
            -cos(dOMEGA)*cos(uu)*omega_0-sin(dOMEGA)*sin(uu)*omega_t;
        dxd=Rt*y1;
        dyd=-Rt*sin(i)*cos(uu)*omega_t;       
    end
    y=(dyd*dx-dxd*dy)/(dx*dx+dy*dy);
end