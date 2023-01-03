function [Ac, Bwc, Czc, Dzc_wc] = ClosedLoopFlowMatrices(OLSDSystem, opts, nc)
%CLOSEDLOOPFLOWMATRICES calculates closed-loop continuous-time flow matrices (Ac, Bwc,
%Czc, Dzc_wc) from an open-loop sampled-data system.
%
%   [Ac, Bc, Cc, Dc] = CLOSEDLOOPFLOWMATRICES(OLJFSystem, nc) returns the
%   closed-loop flow matrices based on an open-loop sampled-data system. Here
%   nc denotes the dimension of the controller.
%
%   The open-loop sampled-data system has the following state-space
%   realization.
%
%  \dot{x} = Ac*x  + Bwc*w_c
%   x^+    = Ad*x  + Bwd*w_d    + Bud*\hat{u}
%   z_c    = Czc*x + Dzc_wc*w_c
%   z_d    = Czd*x + Dzd_wd*w_d + Dzd_u*\hat{u}
%   y      = Cy*x  + Dy_wd*w_d + Dy_u*\hat{u}
%
%   The closed-loop jump-flow system has the following state-space
%   realization
%
%  \xi   = Ac*\xi  + Bc*w_c
%  \xi^+ = Ad*\xi  + Bd*w_d
%   z_c  = Cc*\xi + Dc*w_c
%   z_d  = Cd*\zi + Dd*w_d

arguments
    OLSDSystem      (1,1) OpenLoopSampledDataSystem
    opts            (1,1) SDopts
    nc              (1,1) double = OLSDSystem.nx
end

if strcmpi(OLSDSystem.reconstructor, 'unspecified')
    OLSDSystem = applyReconstructor(OLSDSystem, opts);
    nc = nc+OLSDSystem.nu;
end

% Dimensions
nwc = OLSDSystem.nwc;
nzc = OLSDSystem.nzc;

Ac = blkdiag(OLSDSystem.Ac, zeros(nc));
Bwc = [OLSDSystem.Bwc; zeros(nc, nwc)];
Czc = [OLSDSystem.Czc, zeros(nzc, nc)];
Dzc_wc = OLSDSystem.Dzc_wc;
end