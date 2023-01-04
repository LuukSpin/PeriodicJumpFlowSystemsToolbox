function jfsys = sd2jf(sdsys, ny, nu)

arguments
    sdsys
    ny      (1,1) double = 0
    nu      (1,1) double = 0
end

if nu ~= 0 || ny ~= 0

    nx = sdsys.nx;
    nwc = sdsys.nwc;
    jfsys = OpenLoopJumpFlowSystem(sdsys.Ac, sdsys.Bwc, sdsys.Buc, sdsys.Ad, sdsys.Bwd, sdsys.Bud, sdsys.Czc, sdsys.Dzc_wc, sdsys.Dzc_uc, ...
                                   sdsys.Czd, sdsys.Dzd_wd, sdsys.Dzd_ud, zeros(0, nx), zeros(0, nwc), zeros(0, nu), sdsys.Cy, sdsys.Dy_wd, sdsys.Dy_ud);

else
    Ac = sdsys.Ac;
    Bc = [sdsys.Bwc, sdsys.Buc];
    Ad = sdsys.Ad;
    Bd = [sdsys.Bwd, sdsys.Bud];
    Cc = sdsys.Czc;
    Dc = [sdsys.Dzc_wc, sdsys.Dzc_uc];
    Cd = [sdsys.Czd; sdsys.Cy];
    Dd = [sdsys.Dzd_wd, sdsys.Dzd_ud; sdsys.Dy_wd, sdsys.Dy_ud];

    jfsys = JumpFlowSystem(Ac, Bc, Ad, Bd, Cc, Dc, Cd, Dd);
    
end
end