function ctsys = sd2ct(sdsys)
%The continuous-time counter part of the unweighted sampled-data plant has
%the following state-space realization
% [\dot{x}] = [ A  Bc  Bd  Bu ] [ x ]
% |  z_c  | = | Cc Dcc Dcd Dcu| |w_c|
% |  z_d  | = | Cd Ddc Ddd Ddu| |w_d|
% [   y   ] = [ Cy Dyc Dyd Dyu] [ u ]
% Here z_d is interpreted as z_d(t) instead of z_d[k]. This also holds for
% w_d and y

%This can only be done when the plant has no jump dynamics (or when the SD
%system is unweighted)
arguments
    sdsys   OpenLoopSampledDataSystem
end

nx = sdsys.nx;
nwd = sdsys.nwd;
nu = sdsys.nu;

if ~strcmpi(simplestForm(sdsys), 'ct')
    A = sdsys.Ac;
    B = [sdsys.Bwc, sdsys.Buc];
    C = sdsys.Czc;
    D = [sdsys.Dzc_wc, sdsys.Dzc_uc];
elseif all(sdsys.Ad == eye(nx), 'all') && all(sdsys.Bwd == zeros(nx, nwd), 'all') && all(sdsys.bud == zeros(nx, nu), 'all')
    nwc = sdsys.nwc;
    nzc = sdsys.nzc;
    nzd = sdsys.nzd;
    ny = sdsys.ny;
    A = sdsys.Ac;
    B = [sdsys.Bwc, sdsys.Bwd, sdsys.Buc];
    C = [sdsys.Czc; sdsys.Czd; sdsys.Cy];
    D = [sdsys.Dzc_wc, zeros(nzc, nwd), sdsys.Dzc_uc; zeros(nzd, nwc), sdsys.Dzd_wd, sdsys.Dzd_ud; zeros(ny, nwc), sdsys.Dy_wd, sdsys.Dy_ud];
else
    error('Sampled-data system cannot be converted into a continuous-time system');
end

ctsys = minreal(ss(A, B, C, D));

end