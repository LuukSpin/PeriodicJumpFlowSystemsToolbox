function OpenLoopCTPlant = makeCTPlantFromJF(OpenLoopJFSystem)
%The continuous-time counter part of the unweighted jump-flow plant has
%the following state-space realization
% [\dot{x}] = [ A  Bc  Bd  Bu ] [ x ]
% |  z_c  | = | Cc Dcc Dcd Dcu| |w_c|
% |  z_d  | = | Cd Ddc Ddd Ddu| |w_d|
% [   y   ] = [ Cy Dyc Dyd Dyu] [ u ]
% Here z_d is interpreted as z_d(t) instead of z_d[k]. This also holds for
% w_d and y

%This can only be done when the plant has no jump dynamics (or when the SD
%system is unweighted)

dimCheck(OpenLoopJFSystem);

AdIdentityCheck = all(OpenLoopJFSystem.Ad == eye(size(OpenLoopJFSystem.Ad)), 'all');
BwdZeroCheck = all(OpenLoopJFSystem.Bwd == zeros(size(OpenLoopJFSystem.Bwd)), 'all');
BudZeroCheck =all(OpenLoopJFSystem.Bud == zeros(size(OpenLoopJFSystem.Bud)), 'all');
if ~(AdIdentityCheck || BwdZeroCheck || BudZeroCheck)
    error('The sampled-data system should be unweighted for this function, or Ad=I, Bwd=0 and Bud=0.');
end

%dimensions
nx = OpenLoopJFSystem.nx;
nwc = OpenLoopJFSystem.nwc;
nwd = OpenLoopJFSystem.nwd;
nzc = OpenLoopJFSystem.nzc;
nzd = OpenLoopJFSystem.nzd;
nyc = OpenLoopJFSystem.nyc;
nyd = OpenLoopJFSystem.nyd;
nuc = OpenLoopJFSystem.nuc;
nud = OpenLoopJFSystem.nud;

A  = OpenLoopJFSystem.Ac;
Bc = OpenLoopJFSystem.Bwc;
Bd = zeros(nx, nwd); %This is zero because the flow dynamics are not influenced by the discrete-time disturbance channels
Bu = [OpenLoopJFSystem.Buc, OpenLoopJFSystem.Bud];
B = [Bc, Bd, Bu];

Cc = OpenLoopJFSystem.Czc;
Cd = OpenLoopJFSystem.Czd;
Cy = [OpenLoopJFSystem.Cyc; OpenLoopJFSystem.Cyd];
C = [Cc; Cd; Cy];

Dcc = OpenLoopJFSystem.Dzc_wc;
Dcd = zeros(nzc, nwd); %This is zero because the continuous-time performance channels are not influenced by the discrete-time disturbance channels
Dcu = OpenLoopJFSystem.Dzc_u;

Ddc = zeros(nzd, nwc); %This is zero because the discrete-time performance channels are not influenced by the continuous-time disturbance channels
Ddd = OpenLoopJFSystem.Dzd_wd;
Ddu = OpenLoopJFSystem.Dzd_u;

Dyc = zeros(ny, nwc); %This is zero because the controller input is not influenced by the continuous-time disturbance channels.
Dyd = OpenLoopJFSystem.Dy_wd;
Dyu = [OpenLoopJFSystem.Dyc_uc, zeros(nyc, nud); zeros(nyd, nuc), OpenLoopJFSystem.Dyd_ud];

D = [Dcc, Dcd, Dcu; Ddc, Ddd, Ddu; Dyc, Dyd, Dyu];

OpenLoopCTPlant = minreal(ss(A, B, C, D));

end

