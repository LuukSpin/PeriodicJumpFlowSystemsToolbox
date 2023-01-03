function objSD = addCompensator(OLSDSystem, compensator, opts)
arguments
    OLSDSystem      (1,1) OpenLoopSampledDataSystem
    compensator
    opts            SDopts
end

compensator = ss(compensator);
A_l = compensator.A;
B_l = compensator.B;
C_l = compensator.C;
D_l = compensator.D;

nx = OLSDSystem.nx;
nx_l = size(A_l, 1);
nwc = OLSDSystem.nwc;
nwd = OLSDSystem.nwd;
nzc = OLSDSystem.nzc;
nzd = OLSDSystem.nzd;
nu = OLSDSystem.nu;

Ac = blkdiag(OLSDSystem.Ac, zeros(nx_l));
Bwc = [OLSDSystem.Bwc; zeros(nx_l, nwc)];
Buc = [OLSDSystem.Buc; zeros(nx_l, nu)];

Ad = [OLSDSystem.Ad, OLSDSystem.Bud*C_l; zeros(nx_l, nx), A_l];
Bwd = [OLSDSystem.Bwd; zeros(nx_l, nwd)];
Bud = [OLSDSystem.Bud*D_l; B_l];

Czc = [OLSDSystem.Czc, zeros(nzc, nx_l)];
Dzc_wc = OLSDSystem.Dzc_wc;
Dzc_uc = OLSDSystem.Dzc_uc;

Czd = [OLSDSystem.Czd, OLSDSystem.Dzd_ud*C_l];
Dzd_wd = OLSDSystem.Dzd_wd;
Dzd_ud = OLSDSystem.Dzd_ud*D_l;

Cy = [OLSDSystem.Cy, OLSDSystem.Dy_ud*C_l];
Dy_wd = OLSDSystem.Dy_wd;
Dy_ud = OLSDSystem.Dy_ud*D_l;

objSD = OpenLoopSampledDataSystem(Ac, Bwc, Buc, Ad, Bwd, Bud, Czc, Dzc_wc, Dzc_uc, Czd, Dzd_wd, Dzd_ud, Cy, Dy_wd, Dy_ud);

Ac = blkdiag(OLSDSystem.Ac, zeros(nx_l));
Bwc = [OLSDSystem.Bwc; zeros(nx_l, nwc)];
Buc = [OLSDSystem.Buc; zeros(nx_l, nu)];

Ad = [OLSDSystem.Ad, zeros(nx, nx_l); B_l*OLSDSystem.Cy, A_l];
Bwd = [OLSDSystem.Bwd; B_l*OLSDSystem.Dy_wd];
Bud = [OLSDSystem.Bud; B_l*OLSDSystem.Dy_ud];

Czc = [OLSDSystem.Czc, zeros(nzc, nx_l)];
Dzc_wc = OLSDSystem.Dzc_wc;
Dzc_uc = OLSDSystem.Dzc_uc;

Czd = [OLSDSystem.Czd, zeros(nzd, nx_l)];
Dzd_wd = OLSDSystem.Dzd_wd;
Dzd_ud = OLSDSystem.Dzd_ud;

Cy = [D_l*OLSDSystem.Cy, C_l];
Dy_wd = D_l*OLSDSystem.Dy_wd;
Dy_ud = D_l*OLSDSystem.Dy_ud;

objSD = OpenLoopSampledDataSystem(Ac, Bwc, Buc, Ad, Bwd, Bud, Czc, Dzc_wc, Dzc_uc, Czd, Dzd_wd, Dzd_ud, Cy, Dy_wd, Dy_ud);
end