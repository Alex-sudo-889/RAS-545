function sol_ddq2 = compute_ddq2(q1,q2,q3,dq1,dq2,dq3,L1,L2,L3,m1,m2,m3,g)
%COMPUTE_DDQ2
%    SOL_DDQ2 = COMPUTE_DDQ2(Q1,Q2,Q3,DQ1,DQ2,DQ3,L1,L2,L3,M1,M2,M3,G)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    16-Nov-2024 02:55:41

t2 = sin(q1);
t3 = sin(q2);
t4 = sin(q3);
t5 = q1+q2;
t6 = L1.^2;
t7 = L2.^2;
t8 = dq1.^2;
t9 = dq2.^2;
t10 = dq3.^2;
t11 = m2.^2;
t12 = q2.*2.0;
t13 = q3.*2.0;
t19 = -q1;
t20 = -q2;
t21 = -q3;
t14 = cos(t12);
t15 = cos(t13);
t16 = sin(t12);
t17 = sin(t13);
t18 = sin(t5);
t22 = -t13;
t23 = q2+t5;
t24 = q1+t13;
t25 = q2+t13;
t26 = q3+t12;
t31 = q1+t20;
t33 = q2+t21;
t34 = t12+t13;
t27 = sin(t23);
t28 = sin(t24);
t29 = sin(t25);
t30 = sin(t26);
t32 = q1+t22;
t35 = sin(t31);
t37 = sin(t33);
t38 = t19+t25;
t36 = sin(t32);
t39 = sin(t38);
et1 = t3.*t6.*t8.*t11.*4.0+t3.*t7.*t8.*t11+t3.*t7.*t9.*t11-L2.*g.*t2.*t11+L1.*g.*t11.*t18.*2.0+L2.*g.*t11.*t27-L1.*g.*t11.*t35.*2.0+L1.*L2.*t8.*t11.*t16.*2.0+L1.*L2.*t9.*t11.*t16-L2.*g.*m1.*m2.*t2-L2.*g.*m1.*m3.*t2.*2.0-L2.*g.*m2.*m3.*t2.*3.0+L1.*g.*m2.*m3.*t18.*3.0+L2.*g.*m1.*m3.*t28+L2.*g.*m2.*m3.*t27.*2.0+L2.*g.*m2.*m3.*t28-L1.*g.*m1.*m2.*t35-L1.*g.*m1.*m3.*t35-L1.*g.*m2.*m3.*t35.*3.0+L2.*g.*m1.*m3.*t36+L2.*g.*m2.*m3.*t36-L1.*g.*m1.*m3.*t39-L1.*g.*m2.*m3.*t39+dq1.*dq2.*t3.*t7.*t11.*2.0+m1.*m2.*t3.*t6.*t8+m1.*m3.*t3.*t6.*t8+m2.*m3.*t3.*t6.*t8.*6.0+m2.*m3.*t3.*t7.*t8.*3.0+m2.*m3.*t3.*t7.*t9.*3.0;
et2 = -m1.*m3.*t6.*t8.*t29-m2.*m3.*t6.*t8.*t29.*2.0-m2.*m3.*t7.*t8.*t29-m2.*m3.*t7.*t9.*t29-L1.*g.*m2.*m3.*sin(t5+t13)-L2.*g.*m2.*m3.*sin(t13+t23)-L1.*L2.*m2.*m3.*t8.*sin(t34)+L1.*L2.*dq1.*dq2.*t11.*t16.*2.0-L1.*L3.*m1.*m3.*t4.*t8-L1.*L3.*m1.*m3.*t4.*t9-L1.*L3.*m2.*m3.*t4.*t8.*3.0-L1.*L3.*m1.*m3.*t4.*t10-L1.*L3.*m2.*m3.*t4.*t9.*3.0-L1.*L3.*m2.*m3.*t4.*t10.*3.0-L1.*L2.*m1.*m3.*t8.*t17+L1.*L2.*m2.*m3.*t8.*t16.*4.0-L1.*L2.*m1.*m3.*t9.*t17-L1.*L2.*m2.*m3.*t8.*t17.*2.0+L1.*L2.*m2.*m3.*t9.*t16.*2.0-L1.*L2.*m2.*m3.*t9.*t17.*2.0+L1.*L3.*m2.*m3.*t8.*t30+L1.*L3.*m2.*m3.*t9.*t30+L1.*L3.*m2.*m3.*t10.*t30;
et3 = L2.*L3.*m2.*m3.*t8.*t37+L2.*L3.*m2.*m3.*t9.*t37+L2.*L3.*m2.*m3.*t10.*t37+dq1.*dq2.*m2.*m3.*t3.*t7.*6.0-dq1.*dq2.*m2.*m3.*t7.*t29.*2.0-L1.*L3.*dq1.*dq2.*m1.*m3.*t4.*2.0-L1.*L3.*dq1.*dq2.*m2.*m3.*t4.*6.0-L1.*L3.*dq1.*dq3.*m1.*m3.*t4.*2.0-L1.*L3.*dq1.*dq3.*m2.*m3.*t4.*6.0-L1.*L3.*dq2.*dq3.*m1.*m3.*t4.*2.0-L1.*L3.*dq2.*dq3.*m2.*m3.*t4.*6.0-L1.*L2.*dq1.*dq2.*m1.*m3.*t17.*2.0+L1.*L2.*dq1.*dq2.*m2.*m3.*t16.*4.0-L1.*L2.*dq1.*dq2.*m2.*m3.*t17.*4.0+L1.*L3.*dq1.*dq2.*m2.*m3.*t30.*2.0+L1.*L3.*dq1.*dq3.*m2.*m3.*t30.*2.0+L1.*L3.*dq2.*dq3.*m2.*m3.*t30.*2.0+L2.*L3.*dq1.*dq2.*m2.*m3.*t37.*2.0+L2.*L3.*dq1.*dq3.*m2.*m3.*t37.*2.0+L2.*L3.*dq2.*dq3.*m2.*m3.*t37.*2.0;
sol_ddq2 = ((et1+et2+et3).*-2.0)./(L1.*L2.*(t11.*2.0+m1.*m2+m1.*m3.*2.0+m2.*m3.*6.0-t11.*t14.*2.0+m2.*m3.*cos(t34).*2.0-m1.*m3.*t15.*2.0-m2.*m3.*t14.*4.0-m2.*m3.*t15.*4.0));
end
