function [dfx_ds,dfy_ds,dfx_dalpha,dfy_dalpha] = pacejkaDerivatives(Ba,Bs,Ca,Cs,Da,Ds,Ea,Es,Ka,Ks,alpha,fz,mu,s,sgn)
%PACEJKA_DERIVATIVES
%    [DFX_DS,DFY_DS,DFX_DALPHA,DFY_DALPHA] = PACEJKA_DERIVATIVES(BA,BS,CA,CS,DA,DS,EA,ES,KA,KS,ALPHA,FZ,MU,S,SGN)

%    This function was generated by the Symbolic Math Toolbox version 5.10.
%    05-Aug-2014 15:29:21

t2 = 1.0./pi;
t3 = Ba.*Ka.*alpha.*t2.*2.0;
t4 = atan(t3);
t5 = Ea.*t4;
t6 = Ea-1.0;
t21 = Ba.*Ka.*alpha.*t2.*t6.*2.0;
t7 = t5-t21;
t8 = atan(t7);
t9 = Ca.*t8;
t10 = sin(t9);
t11 = Bs.*Ks.*s;
t12 = atan(t11);
t13 = Es.*t12;
t14 = Es-1.0;
t20 = Bs.*Ks.*s.*t14;
t15 = t13-t20;
t16 = atan(t15);
t17 = Cs.*t16;
t18 = sin(t17);
t19 = tan(alpha);
t22 = Da.*Ds.*t10.*t18;
t23 = abs(t22);
t24 = Da.^2;
t25 = s.^2;
t26 = t10.^2;
t27 = t24.*t25.*t26;
t28 = Ds.^2;
t29 = t18.^2;
t30 = t19.^2;
t31 = t28.*t29.*t30;
t32 = t27+t31;
t33 = cos(t17);
t34 = Bs.*Ks.*t14;
t35 = Bs.^2;
t36 = Ks.^2;
t37 = t25.*t35.*t36;
t38 = t37+1.0;
t39 = 1.0./t38;
t47 = Bs.*Es.*Ks.*t39;
t40 = t34-t47;
t41 = t15.^2;
t42 = t41+1.0;
t43 = 1.0./t42;
t44 = 1.0./sqrt(t32);
t45 = 1.0./t32.^(3.0./2.0);
t46 = s.*t24.*t26.*2.0;
t48 = t46-Cs.*t18.*t28.*t30.*t33.*t40.*t43.*2.0;
t49 = sign(t22);
dfx_ds = fz.*mu.*sgn.*t23.*t44-fz.*mu.*s.*sgn.*t23.*t45.*t48.*(1.0./2.0)-Cs.*Da.*Ds.*fz.*mu.*s.*sgn.*t10.*t33.*t40.*t43.*t44.*t49;
if nargout > 1
    dfy_ds = fz.*mu.*sgn.*t19.*t23.*t45.*t48.*(1.0./2.0)+Cs.*Da.*Ds.*fz.*mu.*sgn.*t10.*t19.*t33.*t40.*t43.*t44.*t49;
end
if nargout > 2
    t50 = cos(t9);
    t51 = Ba.*Ka.*t2.*t6.*2.0;
    t52 = Ba.^2;
    t53 = Ka.^2;
    t54 = 1.0./pi.^2;
    t55 = alpha.^2;
    t56 = t52.*t53.*t54.*t55.*4.0;
    t57 = t56+1.0;
    t58 = 1.0./t57;
    t65 = Ba.*Ea.*Ka.*t2.*t58.*2.0;
    t59 = t51-t65;
    t60 = t7.^2;
    t61 = t60+1.0;
    t62 = 1.0./t61;
    t63 = t30+1.0;
    t64 = t19.*t28.*t29.*t63.*2.0;
    t66 = t64-Ca.*t10.*t24.*t25.*t50.*t59.*t62.*2.0;
    dfx_dalpha = fz.*mu.*s.*sgn.*t23.*t45.*t66.*(-1.0./2.0)-Ca.*Da.*Ds.*fz.*mu.*s.*sgn.*t18.*t44.*t49.*t50.*t59.*t62;
end
if nargout > 3
    dfy_dalpha = -fz.*mu.*sgn.*t23.*t44.*t63+fz.*mu.*sgn.*t19.*t23.*t45.*t66.*(1.0./2.0)+Ca.*Da.*Ds.*fz.*mu.*sgn.*t18.*t19.*t44.*t49.*t50.*t59.*t62;
end
