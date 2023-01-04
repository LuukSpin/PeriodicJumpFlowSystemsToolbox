function jfsys = dt2jf(dtsys, ny, nu)

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
    Buf = zeros(nx, 0);
    Ad = A;
    Bwd = B(:, 1:nw);
    Buj = B(:, nw+1:end);
    Czc = zeros(0, nx);
    Dzc_wc = zeros(0, 0);
    Dzc_uf = zeros(0, 0);
    Czd = C(1:nz, :);
    Dzd_wd = D(1:nz, 1:nw);
    Dzd_uj = D(1:nz, nw+1:end);
    Cyf = zeros(0, nx);
    Dyf_wc = zeros(0, 0);
    Dyf_uf = zeros(0, 0);
    Cyj = C(nz+1:end, :);
    Dyj_wd = D(nz+1:end, 1:nw);
    Dyj_uj = D(nz+1:enb, nw+1:end);

    jfsys = OpenLoopJumpFlowSystem(Ac, Bwc, Buf, Ad, Bwd, Buj, Czc, Dzc_wc, Dzc_uf, Czd, Dzd_wd, Dzd_uj, Cyf, Dyf_wc, Dyf_uf, Cyj, Dyj_wd, Dyj_uj);

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