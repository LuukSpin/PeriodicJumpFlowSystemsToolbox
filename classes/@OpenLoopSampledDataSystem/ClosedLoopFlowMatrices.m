function [Ac, Bwc, Czc, Dzc_wc] = ClosedLoopFlowMatrices(sys, opts, nc)
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
    sys             OpenLoopSampledDataSystem
    opts            jfopt
    nc              double = sys.nx
end

if strcmpi(sys.reconstructor, 'unspecified')
    sys = applyReconstructor(sys, opts);
    nc = nc+sys.nu;
end

% Dimensions
nwc = sys.nwc;
nzc = sys.nzc;

Ac = blkdiag(sys.Ac, zeros(nc));
Bwc = [sys.Bwc; zeros(nc, nwc)];
Czc = [sys.Czc, zeros(nzc, nc)];
Dzc_wc = sys.Dzc_wc;
end