function jfsys = dt2sd(dtsys, ny, nu)

arguments
    dtsys
    ny      (1,1) double = 0
    nu      (1,1) double = 0
end

dtsys = ss(dtsys);

A = dtsys.A;
B = dtsys.B;
C = dtsys.C;
D = dtsys.D;

[p, m] = size(dtsys);

nx = size(dtsys.A, 1);
nw = m-nu;
nz = p-ny;

if nu ~= 0 || ny ~= 0

    Ac = zeros(nx);
    Bwc = zeros(nx, 0);
    Buc = zeros(nx, nu);
    Ad = A;
    Bwd = B(:, 1:nw);
    Bud = B(:, nw+1:end);
    Czc = zeros(0, nx);
    Dzc_wc = zeros(0, 0);
    Dzc_uc = zeros(0, nu);
    Czd = C(1:nz, :);
    Dzd_wd = D(1:nz, 1:nw);
    Dzd_ud = D(1:nz, nw+1:end);
    Cy = C(nz+1:end, :);
    Dy_wd = D(nz+1:end, 1:nw);
    Dy_ud = D(nz+1:end, nw+1:end);

    jfsys = OpenLoopSampledDataSystem(Ac, Bwc, Buc, Ad, Bwd, Bud, Czc, Dzc_wc, Dzc_uc, Czd, Dzd_wd, Dzd_ud, Cy, Dy_wd, Dy_ud);

else
    Ac = zeros(nx);
    Bc = zeros(nx, 0);
    Ad = A;
    Bd = B;
    Cc = zeros(0, nx);
    Dc = zeros(0, 0);
    Cd = C;
    Dd = D;

    jfsys = JumpFlowSystem(Ac, Bc, Ad, Bd, Cc, Dc, Cd, Dd);
    
end
end