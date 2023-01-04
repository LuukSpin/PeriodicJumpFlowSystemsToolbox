function SDSystem = plus(objSD1, objSD2)

Ac = blkdiag(objSD1.Ac, objSD2.Ac);
Bwc = [objSD1.Bwc; objSD2.Bwc];
Buc = [objSD1.Buc; objSD2.Buc];
Ad = blkdiag(objSD1.Ad, objSD2.Ad);
Bwd = [objSD1.Bwd; objSD2.Bwd];
Bud = [objSD1.Bud; objSD2.Bud];
Czc = [objSD1.Czc, objSD2.Czc];
Dzc_wc = objSD1.Dzc_wc+objSD2.Dzc_wc;
Dzc_uc = objSD1.Dzc_uc+objSD2.Dzc_uc;
Czd = [objSD1.Czd, objSD2.Czd];
Dzd_wd = objSD1.Dzd_wd+objSD2.Dzd_wd;
Dzd_u = objSD1.Dzd_uc+objSD2.Dzd_u;
Cy = [objSD1.Cy, objSD2.Cy];
Dy_wd = objSD1.Dy_wd+objSD2.Dy_wd;
Dy_u = objSD1.Dy_u+objSD2.Dy_u;

SDSystem = OpenLoopSampledDataSystem(Ac, Bwc, Buc, Ad, Bwd, Bud, Czc, Dzc_wc, Dzc_uc, Czd, Dzd_wd, Dzd_u, Cy, Dy_wd, Dy_u);
end