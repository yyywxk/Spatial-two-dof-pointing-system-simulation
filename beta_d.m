function y=beta_d(u)
    global flag Re R omega_c phi_g lambda_g Rt omega_t omega_0 i OMEGA_T omega_T
    if(flag==1)
        %%地面目标
        dx=Re*cos(phi_g)*sin(omega_c*u+lambda_g);
        dy=-Re*sin(phi_g);
        dz=-Re*cos(phi_g)*cos(omega_c*u+lambda_g)+R;
    elseif(flag==2)
        %%运动目标
        uu=omega_T+omega_t*u;
        dOMEGA=-omega_0*u+OMEGA_T;
        dx=Rt*(sin(dOMEGA)*cos(uu)+cos(dOMEGA)*cos(i)*sin(uu));
        dy=-Rt*sin(i)*sin(uu);
        dz=Rt*(-cos(dOMEGA)*cos(uu)+sin(dOMEGA)*cos(i)*sin(uu))+R;   
    end
    d=sqrt(dx*dx+dy*dy+dz*dz);
    y=asin(-dz/d);
end