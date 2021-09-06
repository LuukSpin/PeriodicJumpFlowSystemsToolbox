function OpenLoopCTPlant = makeCTPlantFromSD(OpenLoopSDSystem)
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

AdIdentityCheck = all(OpenLoopSDSystem.Ad == eye(size(OpenLoopSDSystem.Ad)), 'all');
BwdZeroCheck = all(OpenLoopSDSystem.Bwd == zeros(size(OpenLoopSDSystem.Bwd)), 'all');
BudZeroCheck =all(OpenLoopSDSystem.Bud == zeros(size(OpenLoopSDSystem.Bud)), 'all');
if ~(AdIdentityCheck || BwdZeroCheck || BudZeroCheck)
    error('The sampled-data system should be unweighted for this function, or Ad=I, Bwd=0 and Bud=0.');
end

%dimensions
nx = size(OpenLoopSDSystem.Ac, 1);
nwc = size(OpenLoopSDSystem.Bwc, 2);
nwd = size(OpenLoopSDSystem.Bwd, 2);
nzc = size(OpenLoopSDSystem.Czc, 1);
nzd = size(OpenLoopSDSystem.Czd, 1);
ny = size(OpenLoopSDSystem.Cy, 1);

A  = OpenLoopSDSystem.Ac;
Bc = OpenLoopSDSystem.Bwc;
Bd = zeros(nx, nwd); %This is zero because the flow dynamics are not influenced by the discrete-time disturbance channels
Bu = OpenLoopSDSystem.Buc;
B = [Bc, Bd, Bu];

Cc = OpenLoopSDSystem.Czc;
Cd = OpenLoopSDSystem.Czd;
Cy = OpenLoopSDSystem.Cy;
C = [Cc; Cd; Cy];

Dcc = OpenLoopSDSystem.Dzc_wc;
Dcd = zeros(nzc, nwd); %This is zero because the continuous-time performance channels are not influenced by the discrete-time disturbance channels
Dcu = OpenLoopSDSystem.Dzc_u;

Ddc = zeros(nzd, nwc); %This is zero because the discrete-time performance channels are not influenced by the continuous-time disturbance channels
Ddd = OpenLoopSDSystem.Dzd_wd;
Ddu = OpenLoopSDSystem.Dzd_u;

Dyc = zeros(ny, nwc); %This is zero because the controller input is not influenced by the continuous-time disturbance channels.
Dyd = OpenLoopSDSystem.Dy_wd;
Dyu = OpenLoopSDSystem.Dy_u;

D = [Dcc, Dcd, Dcu; Ddc, Ddd, Ddu; Dyc, Dyd, Dyu];

OpenLoopCTPlant = minreal(ss(A, B, C, D));

end

