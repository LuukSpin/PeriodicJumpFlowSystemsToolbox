function jfsys_out = jf2jf(jfsys_in, ny, nu)

arguments
    jfsys_in OpenLoopJumpFlowSystem
    ny      (1,1) double = 0
    nu      (1,1) double = 0
end

if nu ~= 0 || ny ~= 0

    jfsys_out = jfsys_in;

else
    Ac = jfsys_in.Ac;
    Bc = [jfsys_in.Bwc, jfsys_in.Buf];
    Ad = jfsys_in.Ad;
    Bd = [jfsys_in.Bwd, jfsys_in.Buj];
    Cc = [jfsys_in.Czc, jfsys_in.Cyf];
    Dc = [jfsys_in.Dzc_wc, jfsys_in.Dzc_uf; jfsys_in.Dyf_wc, jfsys_in.Dyf_uf];
    Cd = [jfsys_in.Czd, jfsys_in.Cyj];
    Dd = [jfsys_in.Dzd_wd, jfsys_in.Dzd_uj; jfsys_in.Dyj_wd, jfsys_in.Dyj_uj];

    jfsys_out = JumpFlowSystem(Ac, Bc, Ad, Bd, Cc, Dc, Cd, Dd);
    
end
end