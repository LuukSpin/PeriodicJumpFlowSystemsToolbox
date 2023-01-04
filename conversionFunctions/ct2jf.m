function jfsys = ct2jf(ctsys, ny, nu)

arguments
    ctsys
    ny      (1,1) double = 0
    nu      (1,1) double = 0
end

ctsys = ss(ctsys);

A = ctsys.A;
B = ctsys.B;
C = ctsys.C;
D = ctsys.D;

[p, m] = size(ctsys);

nx = size(ctsys.A, 1);
nw = m-nu;
nz = p-ny;

if nu ~= 0 || ny ~= 0

    Ac = A;
    Bwc = B(:, 1:nw);
    Buf = B(:, nw+1:end);
    Ad = eye(nx);
    Bwd = zeros(nx, 0);
    Buj = zeros(nx, 0);
    Czc = C(1:nz, :);
    Dzc_wc = D(1:nz, 1:nw);
    Dzc_uf = D(1:nz, nw+1:end);
    Czd = zeros(0, nx);
    Dzd_wd = zeros(0, 0);
    Dzd_uj = zeros(0, 0);
    Cyf = C(nz+1:end, :);
    Dyf_wc = D(nz+1:end, 1:nw);
    Dyf_uf = D(nz+1:end, nw+1:end);
    Cyj = zeros(0, nx);
    Dyj_wd = zeros(0, 0);
    Dyj_uj = zeros(0, 0);

    jfsys = OpenLoopJumpFlowSystem(Ac, Bwc, Buf, Ad, Bwd, Buj, Czc, Dzc_wc, Dzc_uf, Czd, Dzd_wd, Dzd_uj, Cyf, Dyf_wc, Dyf_uf, Cyj, Dyj_wd, Dyj_uj);

else
    Ac = A;
    Bc = B;
    Ad = eye(nx);
    Bd = zeros(nx, 0);
    Cc = C;
    Dc = D;
    Cd = zeros(0, nx);
    Dd = zeros(0, 0);

    jfsys = JumpFlowSystem(Ac, Bc, Ad, Bd, Cc, Dc, Cd, Dd);

end
end