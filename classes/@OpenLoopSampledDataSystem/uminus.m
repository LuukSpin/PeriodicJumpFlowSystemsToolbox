function SDSystem = uminus(objSD)
arguments
    objSD (1,1) OpenLoopSampledDataSystem
end

Ac = objSD.Ac;
Bwc = objSD.Bwc;
Buc = objSD.Buc;
Ad = objSD.Ad;
Bwd = objSD.Bwd;
Bud = objSD.Bud;
Czc = -objSD.Czc;
Dzc_wc = -objSD.Dzc_wc;
Dzc_uc = -objSD.Dzc_uc;
Czd = -objSD.Czd;
Dzd_wd = -objSD.Dzd_wd;
Dzd_u = -objSD.Dzd_uc;
Cy = -objSD.Cy;
Dy_wd = -objSD.Dy_wd;
Dy_u = -objSD.Dy_u;

SDSystem = OpenLoopSampledDataSystem(Ac, Bwc, Buc, Ad, Bwd, Bud, Czc, Dzc_wc, Dzc_uc, Czd, Dzd_wd, Dzd_u, Cy, Dy_wd, Dy_u);
end