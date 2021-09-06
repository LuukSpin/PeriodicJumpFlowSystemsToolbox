function [A, B, C, D] = closedLoopFlowMatrices(JFSystem, nc)
%CLOSEDLOOPFLOWMATRICES calculates continuous-time flow matrices (Ac, Bwc,
%Czc, Dzc_wd) fromt the open-loop jump-flow system.
%   
%   [Ac, Bc, Cc, Dc] = CLOSEDLOOPFLOWMATRICES(OLJFSystem, nc) returns the
%   closed-loop flow matrices based on an open-loop jump-flow system. Here
%   nc denotes the dimension of the controller.
%   
%   The open-loop jump-flow system has the following state-space
%   realization.
%
%  \dot{x} = Ac*x  + Bwc*w_c
%   x^+    = Ad*x  + Bwd*w_d    + Bud*\hat{u}
%   z_c    = Czc*x + Dzc_wc*w_c
%   z_d    = Czd*x + Dzd_wd*w_d + Dzd_u*\hat{u}
%   y      = Cy*x  + Dy_wd*w_d
%
%   The closed-loop jump flow system has the following state-space
%   realization
%
%  \xi   = Ac*\xi  + Bc*w_c
%  \xi^+ = Ad*\xi  + Bd*w_d
%   z_c  = Cc*\xi + Dc*w_c
%   z_d  = Cd*\zi + Dd*w_d

ni = nargin;

checkOL = strcmpi(JFSystem.Loop, 'Open');
if ~checkOL
    error('Closed-loop flow matrices can only be determine for a open-loop system');
end

%dimensions
nx = size(JFSystem.Ac, 1); %This is equal to nx of the sampled-data system plus nu
n_wc = size(JFSystem.Bwc, 2);
n_zc = size(JFSystem.Czc, 1);

if ni<2
    nc = nx;
end

A = blkdiag(JFSystem.Ac, zeros(nc));
B = [JFSystem.Bwc; zeros(nc, n_wc)];
C = [JFSystem.Czc, zeros(n_zc, nc)];
D = JFSystem.Dzc_wc;

end

