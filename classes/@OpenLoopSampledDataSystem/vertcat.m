function SDSystem = vertcat(obj1, obj2)

Ac = blkdiag(obj1.Ac, obj2.Ac);
Bwc = [obj1.Bwc; obj2.Bwc];
Ad = blkdiag(obj1.Ad, obj2.Ad);
Bwd = [obj1.Bwd; obj2.Bwd];
Bud = [obj1.Bud; obj2.Bud];

Czc = blkdiag(obj1.Czc, obj2.Czc);
Dzc_wc = [obj1.Dzc_wc; obj2.Dzc_wc];
Czd = blkdiag(obj1.Czd, obj2.Czd);
Dzd_wd = [obj1.Dzd_wd; obj2.Dzd_wd];
Dzd_u = [obj1.Dzd_uc; obj2.Dzd_u];
Cy = blkdiag(obj1.Cy, obj2.Cy);
Dy_wd = [obj1.Dy_wd; obj2.Dy_wd];
Dy_u = [obj1.Dy_u; obj2.Dy_u];

SDSystem = OpenLoopSampledDataSystem(Ac, Bwc, Ad, Bwd, Bud, Czc, Dzc_wc, Czd, Dzd_wd, Dzd_u, Cy, Dy_wd, Dy_u);
end